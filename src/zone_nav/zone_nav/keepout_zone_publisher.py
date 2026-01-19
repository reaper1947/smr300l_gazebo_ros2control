#!/usr/bin/env python3
"""
Keepout + Speed Zone Publisher for Nav2 (Costmap Filters)

Goals:
- Publish filter masks + CostmapFilterInfo reliably (TRANSIENT_LOCAL latch)
- Avoid /map QoS mismatch issues (subscribe VOLATILE)
- Avoid race with Nav2 lifecycle/composition by waiting for costmap nodes
- Correct speed filter semantics (base + multiplier * mask_value)

Notes:
- Keepout interpretation differs across setups; toggle ZERO_MEANS_KEEPOUT.
- Speed mask values are typically 0..100 (percent of max) depending on params.
"""

import rclpy
import os
import json
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid


class ZonePublisher(Node):
    def __init__(self):
        super().__init__("zone_publisher")

        # -----------------------------
        # Toggle: what does 0 mean in your keepout mask?
        # True  => 0 = KEEP-OUT, 100 = FREE
        # False => 0 = FREE,     100 = KEEP-OUT
        # -----------------------------
        self.ZERO_MEANS_KEEPOUT = True

        # Default maximum speed (m/s) used by speed filter math (see below)
        self.MAX_SPEED_MPS = 0.30

        # Map editor layers file (written by zone_web_ui)
        self.layers_file = os.path.expanduser('~/map_layers.json')

        # Allow tuning via ROS params
        self.declare_parameter('zero_means_keepout', self.ZERO_MEANS_KEEPOUT)
        self.declare_parameter('max_speed_mps', self.MAX_SPEED_MPS)
        self.declare_parameter('enable_speed_filter', True)
        self.declare_parameter('layers_file', self.layers_file)

        self.ZERO_MEANS_KEEPOUT = bool(self.get_parameter('zero_means_keepout').value)
        self.MAX_SPEED_MPS = float(self.get_parameter('max_speed_mps').value)
        self.enable_speed_filter = bool(self.get_parameter('enable_speed_filter').value)
        self.layers_file = self.get_parameter('layers_file').value or self.layers_file

        # How many times to republish after first publish (helps discovery)
        # Set to 0 to publish exactly once (preferred to avoid costmap thrash)
        self.REPUBLISH_COUNT = 0
        self.REPUBLISH_PERIOD_SEC = 0.5

        # Track map editor layers and changes
        self._layers = {'no_go_zones': [], 'slow_zones': [], 'restricted': []}
        self._layers_mtime = None

        # ---- QoS profiles ----
        # Subscribe to /map with VOLATILE (matches nav2_map_server default)
        self.map_qos = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publish masks and filter_info latched (Nav2 can join later)
        self.latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Publishers (latched) ----
        self.keepout_filter_info_pub = self.create_publisher(
            CostmapFilterInfo, "/costmap_filter_info", self.latched_qos
        )
        self.speed_filter_info_pub = self.create_publisher(
            CostmapFilterInfo, "/speed_filter_info", self.latched_qos
        )
        self.keepout_mask_pub = self.create_publisher(
            OccupancyGrid, "/keepout_filter_mask", self.latched_qos
        )
        self.speed_mask_pub = self.create_publisher(
            OccupancyGrid, "/speed_filter_mask", self.latched_qos
        )

        # ---- Subscriber ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, self.map_qos
        )

        self._map_msg = None
        self._published_once = False
        self._republish_left = 0

        # Check readiness frequently
        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info("ZonePublisher: started")
        self.get_logger().info("Waiting for /map + Nav2 costmaps to appear...")
        self.get_logger().info(f"Keepout semantics: ZERO_MEANS_KEEPOUT={self.ZERO_MEANS_KEEPOUT}")
        self.get_logger().info(f"Speed filter enabled: {self.enable_speed_filter} (max_speed_mps={self.MAX_SPEED_MPS})")
        self.get_logger().info(f"Using map layers file: {self.layers_file}")

    def on_map(self, msg: OccupancyGrid):
        # Store the first good map
        if self._map_msg is None and msg.info.width > 0 and msg.info.height > 0:
            self._map_msg = msg
            self.get_logger().info(
                f"Got /map: frame={msg.header.frame_id} {msg.info.width}x{msg.info.height}, "
                f"res={msg.info.resolution}, origin=({msg.info.origin.position.x:.3f},{msg.info.origin.position.y:.3f})"
            )

    def nav2_costmaps_present(self) -> bool:
        """
        Avoid publishing before Nav2 costmaps exist.
        This helps the 'composition stuck / nothing comes up' timing race.
        """
        names = self.get_node_names()
        return ("/local_costmap/local_costmap" in names) or ("/global_costmap/global_costmap" in names)

    def tick(self):
        if self._map_msg is None:
            return

        if not self.nav2_costmaps_present():
            # Nav2 not ready yet; keep waiting
            return

        layers_changed = self._load_layers_if_changed()
        if layers_changed:
            self._published_once = False

        if not self._published_once:
            self.publish_all()
            self._published_once = True
            self._republish_left = self.REPUBLISH_COUNT
            self.get_logger().info("Published filter infos + masks (latched).")
            return

        if self._republish_left > 0:
            # Republish a few times to make DDS discovery more robust
            self._republish_left -= 1
            self.publish_all()
            # slow down republish rate
            self.timer.cancel()
            self.timer = self.create_timer(self.REPUBLISH_PERIOD_SEC, self.tick)

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        map_frame = self._map_msg.header.frame_id if self._map_msg.header.frame_id else "map"

        keepout_mask = self.build_keepout_mask(self._map_msg, self._layers)
        keepout_mask.header.stamp = now
        keepout_mask.header.frame_id = map_frame
        self.keepout_mask_pub.publish(keepout_mask)

        # Publish speed mask/info only if enabled (disabled by default to avoid unintended clamping)
        if self.enable_speed_filter:
            speed_mask = self.build_speed_mask(self._map_msg, self._layers)
            speed_mask.header.stamp = now
            speed_mask.header.frame_id = map_frame
            self.speed_mask_pub.publish(speed_mask)

        # ---- Keepout filter info ----
        keepout_info = CostmapFilterInfo()
        keepout_info.header.stamp = now
        keepout_info.header.frame_id = map_frame
        keepout_info.type = 0  # KEEPOUT_FILTER
        keepout_info.filter_mask_topic = "/keepout_filter_mask"

        # For keepout filter: typically base/multiplier not used, but keep defaults
        keepout_info.base = 0.0
        keepout_info.multiplier = 1.0
        self.keepout_filter_info_pub.publish(keepout_info)

        # ---- Speed filter info ----
        if self.enable_speed_filter:
            speed_info = CostmapFilterInfo()
            speed_info.header.stamp = now
            speed_info.header.frame_id = map_frame
            speed_info.type = 1  # SPEED_FILTER
            speed_info.filter_mask_topic = "/speed_filter_mask"

            # Nav2 speed filter commonly computes:
            #   speed_limit = base + multiplier * mask_value
            #
            # If mask_value is 0..100 (%), this makes:
            #   mask=0   -> base
            #   mask=100 -> base + 100*multiplier
            #
            # We'll set it so:
            #   mask=0   -> MAX_SPEED_MPS  (no restriction)
            #   mask=100 -> 0.0           (full restriction)
            #
            # That means multiplier must be negative:
            speed_info.base = float(self.MAX_SPEED_MPS)
            speed_info.multiplier = -float(self.MAX_SPEED_MPS) / 100.0
            self.speed_filter_info_pub.publish(speed_info)
        else:
            # Speed publishing disabled by default; do not advertise speed filter
            pass

    def create_blank_keepout_mask_like_map(self, map_msg: OccupancyGrid) -> OccupancyGrid:
        mask = OccupancyGrid()
        mask.info = map_msg.info  # copy metadata (width/height/res/origin)

        size = map_msg.info.width * map_msg.info.height

        # Blank should mean "fully free" everywhere.
        # If 0 means keepout in your setup -> blank must be 100.
        # If 100 means keepout -> blank must be 0.
        blank_val = 100 if self.ZERO_MEANS_KEEPOUT else 0
        mask.data = [blank_val] * size
        return mask

    def create_blank_speed_mask_like_map(self, map_msg: OccupancyGrid) -> OccupancyGrid:
        mask = OccupancyGrid()
        mask.info = map_msg.info

        size = map_msg.info.width * map_msg.info.height

        # Blank means "no speed restriction" everywhere.
        # With our speed_info math above:
        # mask=0 => MAX_SPEED_MPS
        mask.data = [0] * size
        return mask

    def _load_layers_if_changed(self) -> bool:
        """Reload map editor layers when the file changes."""
        try:
            mtime = os.path.getmtime(self.layers_file)
        except FileNotFoundError:
            mtime = None

        if mtime == self._layers_mtime:
            return False

        self._layers_mtime = mtime
        self._layers = self._load_layers()
        total = sum(len(v) for v in self._layers.values())
        self.get_logger().info(f"Loaded map layers: {total} objects")
        return True

    def _load_layers(self):
        if self.layers_file and os.path.exists(self.layers_file):
            try:
                with open(self.layers_file, 'r') as f:
                    data = json.load(f)
                return {
                    'no_go_zones': data.get('no_go_zones', []),
                    'slow_zones': data.get('slow_zones', []),
                    'restricted': data.get('restricted', []),
                }
            except Exception as e:
                self.get_logger().error(f"Failed to load layers from {self.layers_file}: {e}")
        return {'no_go_zones': [], 'slow_zones': [], 'restricted': []}

    def build_keepout_mask(self, map_msg: OccupancyGrid, layers: dict) -> OccupancyGrid:
        mask = self.create_blank_keepout_mask_like_map(map_msg)
        keepout_value = 0 if self.ZERO_MEANS_KEEPOUT else 100

        for layer_name in ('no_go_zones', 'restricted'):
            for obj in layers.get(layer_name, []):
                self._apply_shape(mask, obj, keepout_value, combine='override')

        return mask

    def build_speed_mask(self, map_msg: OccupancyGrid, layers: dict) -> OccupancyGrid:
        mask = self.create_blank_speed_mask_like_map(map_msg)
        for obj in layers.get('slow_zones', []):
            speed_limit = obj.get('speed_limit', self.MAX_SPEED_MPS)
            mask_value = self._speed_limit_to_mask(speed_limit)
            if mask_value <= 0:
                continue
            self._apply_shape(mask, obj, mask_value, combine='max')
        return mask

    def _speed_limit_to_mask(self, speed_limit) -> int:
        try:
            limit = float(speed_limit)
        except (TypeError, ValueError):
            return 0

        if self.MAX_SPEED_MPS <= 0.0:
            return 100

        limit = max(0.0, min(limit, self.MAX_SPEED_MPS))
        mask_value = int(round((self.MAX_SPEED_MPS - limit) * 100.0 / self.MAX_SPEED_MPS))
        return max(0, min(mask_value, 100))

    def _apply_shape(self, mask: OccupancyGrid, obj: dict, value: int, combine: str):
        info = mask.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution
        width = info.width
        height = info.height

        def set_cell(gx, gy):
            if gx < 0 or gy < 0 or gx >= width or gy >= height:
                return
            idx = gx + gy * width
            if combine == 'max':
                mask.data[idx] = max(mask.data[idx], value)
            else:
                mask.data[idx] = value

        shape_type = obj.get('type')
        if shape_type == 'rectangle':
            minx = min(obj.get('x1', 0.0), obj.get('x2', 0.0))
            maxx = max(obj.get('x1', 0.0), obj.get('x2', 0.0))
            miny = min(obj.get('y1', 0.0), obj.get('y2', 0.0))
            maxy = max(obj.get('y1', 0.0), obj.get('y2', 0.0))
            self._fill_rect(mask, minx, miny, maxx, maxy, set_cell, origin_x, origin_y, resolution)
        elif shape_type == 'circle':
            cx = obj.get('x', 0.0)
            cy = obj.get('y', 0.0)
            radius = obj.get('radius', 0.0)
            self._fill_circle(mask, cx, cy, radius, set_cell, origin_x, origin_y, resolution)
        elif shape_type == 'polygon':
            points = obj.get('points', [])
            if points:
                poly = [(p.get('x', 0.0), p.get('y', 0.0)) for p in points]
                self._fill_polygon(mask, poly, set_cell, origin_x, origin_y, resolution)

    def _fill_rect(self, mask, minx, miny, maxx, maxy, set_cell, origin_x, origin_y, resolution):
        gx_min = int(math.floor((minx - origin_x) / resolution))
        gx_max = int(math.floor((maxx - origin_x) / resolution))
        gy_min = int(math.floor((miny - origin_y) / resolution))
        gy_max = int(math.floor((maxy - origin_y) / resolution))

        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                set_cell(gx, gy)

    def _fill_circle(self, mask, cx, cy, radius, set_cell, origin_x, origin_y, resolution):
        if radius <= 0.0:
            return
        gx_min = int(math.floor((cx - radius - origin_x) / resolution))
        gx_max = int(math.floor((cx + radius - origin_x) / resolution))
        gy_min = int(math.floor((cy - radius - origin_y) / resolution))
        gy_max = int(math.floor((cy + radius - origin_y) / resolution))
        r2 = radius * radius

        for gy in range(gy_min, gy_max + 1):
            wy = origin_y + (gy + 0.5) * resolution
            for gx in range(gx_min, gx_max + 1):
                wx = origin_x + (gx + 0.5) * resolution
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= r2:
                    set_cell(gx, gy)

    def _fill_polygon(self, mask, poly, set_cell, origin_x, origin_y, resolution):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)

        gx_min = int(math.floor((minx - origin_x) / resolution))
        gx_max = int(math.floor((maxx - origin_x) / resolution))
        gy_min = int(math.floor((miny - origin_y) / resolution))
        gy_max = int(math.floor((maxy - origin_y) / resolution))

        for gy in range(gy_min, gy_max + 1):
            wy = origin_y + (gy + 0.5) * resolution
            for gx in range(gx_min, gx_max + 1):
                wx = origin_x + (gx + 0.5) * resolution
                if self._point_in_polygon(wx, wy, poly):
                    set_cell(gx, gy)

    def _point_in_polygon(self, x, y, poly):
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            )
            if intersects:
                inside = not inside
            j = i
        return inside


def main(args=None):
    rclpy.init(args=args)
    node = ZonePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Already shut down


if __name__ == "__main__":
    main()
