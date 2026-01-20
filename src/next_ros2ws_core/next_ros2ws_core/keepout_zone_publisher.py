#!/usr/bin/env python3
"""
Zone Publisher for Nav2 Costmap Filters (Keepout + Speed)

This node:
- Subscribes to /map (OccupancyGrid) to match its size/resolution/origin
- Loads zones from a JSON file (default: ~/map_layers.json)
- Rasterizes zones into two masks:
    /keepout_filter_mask   (0 free, 100 keepout)
    /speed_filter_mask     (1..100 speed percent; 100 = full speed)
- Publishes CostmapFilterInfo:
    Keepout: /costmap_filter_info  (type=0)  <-- matches your Nav2 keepout_filter.filter_info_topic
    Speed:   /speed_filter_info    (type=1)

Important:
- Your Nav2 params currently show:
    keepout_filter.filter_info_topic = /costmap_filter_info
    speed_filter.filter_info_topic   = /speed_filter_info
  So we publish exactly those topics.
"""

import os
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid


class ZonePublisher(Node):
    def __init__(self):
        super().__init__("zone_publisher")

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter("layers_file", os.path.expanduser("~/map_layers.json"))
        self.declare_parameter("enable_speed_filter", True)
        self.declare_parameter("max_speed_mps", 0.30)   # used to convert % -> m/s
        self.declare_parameter("publish_rate_hz", 1.0)  # re-publish periodically for robustness

        layers_param = self.get_parameter("layers_file").value
        self.layers_file = os.path.expanduser(layers_param or "~/map_layers.json")
        self.enable_speed_filter = bool(self.get_parameter("enable_speed_filter").value)
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # -----------------------
        # QoS
        # -----------------------
        # /map is typically latched by map_server
        self.map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # costmap filters should be latched so Nav2 can join later
        self.latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -----------------------
        # Publishers
        # -----------------------
        # Keepout info MUST be on /costmap_filter_info because your keepout_filter listens there
        self.keepout_info_pub = self.create_publisher(
            CostmapFilterInfo, "/costmap_filter_info", self.latched_qos
        )

        # Speed info is already on /speed_filter_info
        self.speed_info_pub = self.create_publisher(
            CostmapFilterInfo, "/speed_filter_info", self.latched_qos
        )

        self.keepout_mask_pub = self.create_publisher(
            OccupancyGrid, "/keepout_filter_mask", self.latched_qos
        )
        self.speed_mask_pub = self.create_publisher(
            OccupancyGrid, "/speed_filter_mask", self.latched_qos
        )

        # -----------------------
        # Subscriber
        # -----------------------
        self._map_msg = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, self.map_qos)

        # -----------------------
        # Layers tracking
        # -----------------------
        self._layers = {"no_go_zones": [], "restricted": [], "slow_zones": []}
        self._layers_mtime = None

        # Periodic publish (helps even if Nav2 restarts later)
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info("ZonePublisher running.")
        self.get_logger().info(f"layers_file: {self.layers_file}")
        self.get_logger().info(f"speed_filter: {self.enable_speed_filter} (max_speed_mps={self.max_speed_mps})")
        self.get_logger().info("Publishing keepout info on /costmap_filter_info (type=0)")
        self.get_logger().info("Publishing speed info   on /speed_filter_info (type=1)")

    # -----------------------
    # Callbacks / main loop
    # -----------------------
    def _on_map(self, msg: OccupancyGrid):
        if self._map_msg is None and msg.info.width > 0 and msg.info.height > 0:
            self._map_msg = msg
            self.get_logger().info(
                f"Got /map: {msg.info.width}x{msg.info.height} res={msg.info.resolution} frame={msg.header.frame_id or 'map'}"
            )

    def _nav2_costmaps_present(self) -> bool:
        # Correct check: name + namespace
        for name, ns in self.get_node_names_and_namespaces():
            if name == "local_costmap" and ns == "/local_costmap":
                return True
            if name == "global_costmap" and ns == "/global_costmap":
                return True
        return False

    def _tick(self):
        if self._map_msg is None:
            return
        if not self._nav2_costmaps_present():
            # Wait until Nav2 costmaps exist (avoids timing weirdness)
            return

        self._reload_layers_if_changed()
        self._publish_all()

    # -----------------------
    # Publish
    # -----------------------
    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        map_frame = self._map_msg.header.frame_id or "map"

        keepout_mask = self._build_keepout_mask(self._map_msg, self._layers)
        keepout_mask.header.stamp = now
        keepout_mask.header.frame_id = map_frame
        self.keepout_mask_pub.publish(keepout_mask)

        if self.enable_speed_filter:
            speed_mask = self._build_speed_mask(self._map_msg, self._layers)
            speed_mask.header.stamp = now
            speed_mask.header.frame_id = map_frame
            self.speed_mask_pub.publish(speed_mask)

        # Keepout FilterInfo (type=0)
        k = CostmapFilterInfo()
        k.header.stamp = now
        k.header.frame_id = map_frame
        k.type = 0
        k.filter_mask_topic = "/keepout_filter_mask"
        k.base = 0.0
        k.multiplier = 1.0
        self.keepout_info_pub.publish(k)

        # Speed FilterInfo (type=1) => mask value is % of speed.
        # We convert percent -> m/s using multiplier.
        if self.enable_speed_filter:
            s = CostmapFilterInfo()
            s.header.stamp = now
            s.header.frame_id = map_frame
            s.type = 1
            s.filter_mask_topic = "/speed_filter_mask"
            s.base = 0.0
            s.multiplier = self.max_speed_mps / 100.0
            self.speed_info_pub.publish(s)

    # -----------------------
    # Layers file
    # -----------------------
    def _reload_layers_if_changed(self):
        try:
            mtime = os.path.getmtime(self.layers_file)
        except FileNotFoundError:
            mtime = None

        if mtime == self._layers_mtime:
            return

        self._layers_mtime = mtime
        self._layers = self._load_layers()
        total = sum(len(v) for v in self._layers.values())
        self.get_logger().info(f"Loaded layers: {total} objects")

    def _load_layers(self):
        if self.layers_file and os.path.exists(self.layers_file):
            try:
                with open(self.layers_file, "r") as f:
                    data = json.load(f)
                return {
                    "no_go_zones": data.get("no_go_zones", []),
                    "restricted": data.get("restricted", []),
                    "slow_zones": data.get("slow_zones", []),
                }
            except Exception as e:
                self.get_logger().error(f"Failed to load {self.layers_file}: {e}")
        return {"no_go_zones": [], "restricted": [], "slow_zones": []}

    # -----------------------
    # Mask building
    # -----------------------
    def _blank_mask_like_map(self, map_msg: OccupancyGrid, fill_val: int) -> OccupancyGrid:
        mask = OccupancyGrid()
        mask.info = map_msg.info
        size = map_msg.info.width * map_msg.info.height
        mask.data = [int(fill_val)] * size
        return mask

    def _build_keepout_mask(self, map_msg: OccupancyGrid, layers: dict) -> OccupancyGrid:
        # 0 free everywhere by default
        mask = self._blank_mask_like_map(map_msg, 0)

        # Mark keepout cells as 100
        for layer_name in ("no_go_zones", "restricted"):
            for obj in layers.get(layer_name, []):
                self._apply_shape(mask, obj, 100, combine="override")

        return mask

    def _build_speed_mask(self, map_msg: OccupancyGrid, layers: dict) -> OccupancyGrid:
        # 100% speed everywhere by default
        mask = self._blank_mask_like_map(map_msg, 100)

        # In slow zones, set a smaller percent (e.g., 30 = 30% of max speed)
        for obj in layers.get("slow_zones", []):
            pct = int(obj.get("speed_percent", 50))
            pct = max(1, min(pct, 100))
            self._apply_shape(mask, obj, pct, combine="min")  # stricter (lower) wins

        return mask

    # -----------------------
    # Rasterization
    # -----------------------
    def _apply_shape(self, mask: OccupancyGrid, obj: dict, value: int, combine: str):
        info = mask.info
        ox, oy = info.origin.position.x, info.origin.position.y
        res = info.resolution
        w, h = info.width, info.height

        def set_cell(gx, gy):
            if gx < 0 or gy < 0 or gx >= w or gy >= h:
                return
            idx = gx + gy * w
            if combine == "min":
                mask.data[idx] = min(mask.data[idx], value)
            elif combine == "max":
                mask.data[idx] = max(mask.data[idx], value)
            else:
                mask.data[idx] = value

        t = obj.get("type", "")
        if t == "rectangle":
            x1, y1 = float(obj.get("x1", 0.0)), float(obj.get("y1", 0.0))
            x2, y2 = float(obj.get("x2", 0.0)), float(obj.get("y2", 0.0))
            self._fill_rect(set_cell, ox, oy, res, x1, y1, x2, y2)

        elif t == "circle":
            cx, cy = float(obj.get("x", 0.0)), float(obj.get("y", 0.0))
            r = float(obj.get("radius", 0.0))
            self._fill_circle(set_cell, ox, oy, res, cx, cy, r)

        elif t == "polygon":
            points = obj.get("points", [])
            poly = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in points]
            if len(poly) >= 3:
                self._fill_polygon(set_cell, ox, oy, res, poly)

    def _fill_rect(self, set_cell, ox, oy, res, x1, y1, x2, y2):
        minx, maxx = min(x1, x2), max(x1, x2)
        miny, maxy = min(y1, y2), max(y1, y2)

        gx_min = int(math.floor((minx - ox) / res))
        gx_max = int(math.floor((maxx - ox) / res))
        gy_min = int(math.floor((miny - oy) / res))
        gy_max = int(math.floor((maxy - oy) / res))

        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                set_cell(gx, gy)

    def _fill_circle(self, set_cell, ox, oy, res, cx, cy, radius):
        if radius <= 0.0:
            return

        gx_min = int(math.floor((cx - radius - ox) / res))
        gx_max = int(math.floor((cx + radius - ox) / res))
        gy_min = int(math.floor((cy - radius - oy) / res))
        gy_max = int(math.floor((cy + radius - oy) / res))
        r2 = radius * radius

        for gy in range(gy_min, gy_max + 1):
            wy = oy + (gy + 0.5) * res
            for gx in range(gx_min, gx_max + 1):
                wx = ox + (gx + 0.5) * res
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= r2:
                    set_cell(gx, gy)

    def _fill_polygon(self, set_cell, ox, oy, res, poly):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)

        gx_min = int(math.floor((minx - ox) / res))
        gx_max = int(math.floor((maxx - ox) / res))
        gy_min = int(math.floor((miny - oy) / res))
        gy_max = int(math.floor((maxy - oy) / res))

        for gy in range(gy_min, gy_max + 1):
            wy = oy + (gy + 0.5) * res
            for gx in range(gx_min, gx_max + 1):
                wx = ox + (gx + 0.5) * res
                if self._point_in_poly(wx, wy, poly):
                    set_cell(gx, gy)

    def _point_in_poly(self, x, y, poly):
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            ):
                inside = not inside
            j = i
        return inside


def main():
    rclpy.init()
    node = ZonePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
