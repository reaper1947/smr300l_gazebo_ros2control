#!/usr/bin/env python3
"""
CSM-style global relocalization using a distance transform, with 2 LiDARs (front + rear).

Algorithm (coarse -> refine):
1) Build distance transform D(x,y) from OccupancyGrid: distance-to-nearest-nonfree (in meters).
2) Convert each LiDAR scan into hit points in base_frame (front + rear), downsample beams.
3) Coarse global search:
   - Sample M candidate (x,y) from free cells
   - Evaluate yaw grid (coarse) for each candidate
   - Score pose by projecting scan hit points into map and averaging exp(-D^2/(2*sigma^2))
   - Fuse front+rear scores by weighted average
   - Keep Top-K pose hypotheses
4) Refinement:
   - Around each Top-K, sample local (x,y) jitter + finer yaw grid
   - Re-score and pick best
5) Push to AMCL using /amcl/set_pose (nav2_msgs/SetInitialPose). Fallback: /initialpose publish.
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException

from nav2_msgs.srv import SetInitialPose


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def tf_to_xy_yaw(tf):
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tx, ty, yaw


class CorrelativeRelocalizer(Node):
    def __init__(self):
        super().__init__('correlative_relocalizer')

        # ---------------- Params ----------------
        self.declare_parameter('map_topic', '/map')

        # 2 lidars (front + rear)
        self.declare_parameter('front_scan_topic', '/scan')
        self.declare_parameter('rear_scan_topic', '/scan2')
        self.declare_parameter('front_weight', 1.0)
        self.declare_parameter('rear_weight', 1.0)

        self.declare_parameter('base_frame', 'base_link')

        # Scoring
        self.declare_parameter('sigma', 0.20)          # meters
        self.declare_parameter('scan_downsample', 4)   # every Nth beam
        self.declare_parameter('max_points', 90)       # cap per scan

        # Coarse global search
        self.declare_parameter('num_xy_samples', 1800) # global candidates
        self.declare_parameter('yaw_steps', 36)        # coarse yaw grid
        self.declare_parameter('top_k', 30)            # keep top K hypotheses (coarse)

        # Refinement
        self.declare_parameter('refine_enable', True)
        self.declare_parameter('refine_xy_std', 0.30)          # meters (jitter)
        self.declare_parameter('refine_xy_per_hyp', 60)        # samples per hypothesis
        self.declare_parameter('refine_yaw_range_deg', 15.0)   # +/- range around coarse yaw
        self.declare_parameter('refine_yaw_steps', 21)         # finer yaw grid

        # Output behavior
        self.declare_parameter('publish_initialpose', False)
        self.declare_parameter('min_score', 0.0)

        self.declare_parameter('amcl_set_pose_service', '/amcl/set_pose')
        self.declare_parameter('amcl_set_pose_fallback', '/amcl/set_initial_pose')

        # ---------------- Subs / pubs / srv ----------------
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self.map_cb, 1)

        self.front_scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('front_scan_topic').value, self.front_scan_cb, 10)

        self.rear_scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('rear_scan_topic').value, self.rear_scan_cb, 10)

        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.set_pose_service = self.get_parameter('amcl_set_pose_service').value
        self.set_pose_fallback_service = self.get_parameter('amcl_set_pose_fallback').value
        self.set_pose_cli = self.create_client(SetInitialPose, self.set_pose_service)
        self.set_pose_fallback_cli = self.create_client(SetInitialPose, self.set_pose_fallback_service)

        self.srv = self.create_service(Trigger, '/auto_relocate', self.handle_trigger)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Cached data
        self.map_msg = None
        self.dist_m = None
        self.free_mask = None
        self.front_scan = None
        self.rear_scan = None

        self.get_logger().info("Ready: call /auto_relocate (std_srvs/Trigger)")

    # ---------------- Callbacks ----------------
    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        # free=1 only where occupancy==0. Unknown(-1) and occupied(>=50/100) -> 0
        free = (data == 0).astype(np.uint8)

        # distanceTransform: distance to nearest zero pixel (non-free). So free pixels get >0.
        dist_px = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        self.dist_m = dist_px * float(msg.info.resolution)
        self.free_mask = free

    def front_scan_cb(self, msg: LaserScan):
        self.front_scan = msg

    def rear_scan_cb(self, msg: LaserScan):
        self.rear_scan = msg

    # ---------------- Scan processing ----------------
    def scan_to_points_base(self, scan: LaserScan):
        """Return Nx2 hit points in base_frame (meters)."""
        if scan is None:
            return np.zeros((0, 2), dtype=np.float32)

        base_frame = self.get_parameter('base_frame').value
        laser_frame = scan.header.frame_id

        # Prefer time-aligned TF if possible; fall back to latest.
        try:
            t = rclpy.time.Time.from_msg(scan.header.stamp)
            tf = self.tf_buffer.lookup_transform(base_frame, laser_frame, t)
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(base_frame, laser_frame, rclpy.time.Time())
            except TransformException as e:
                self.get_logger().warn(f"TF {base_frame}<-{laser_frame} not available: {e}")
                return np.zeros((0, 2), dtype=np.float32)

        lx, ly, lyaw = tf_to_xy_yaw(tf)

        ds = int(self.get_parameter('scan_downsample').value)
        max_pts = int(self.get_parameter('max_points').value)

        angles = scan.angle_min + np.arange(len(scan.ranges), dtype=np.float32) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float32)

        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        idx = np.nonzero(valid)[0][::ds]
        if idx.size == 0:
            return np.zeros((0, 2), dtype=np.float32)
        if idx.size > max_pts:
            idx = idx[:max_pts]

        a = angles[idx]
        r = ranges[idx]
        pts_l = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)  # laser frame

        # laser->base (planar)
        c, s = math.cos(lyaw), math.sin(lyaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        pts_b = (pts_l @ R.T) + np.array([lx, ly], dtype=np.float32)
        return pts_b.astype(np.float32)

    # ---------------- Scoring ----------------
    def _score_pose_set(self, pts_b: np.ndarray, cand_xy: np.ndarray, yaw: float):
        """
        Score many candidates at a single yaw for one point-set.
        pts_b: Px2 in base frame
        cand_xy: Mx2 in map frame
        returns: scores M
        """
        msg = self.map_msg
        dist = self.dist_m
        h, w = dist.shape
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)

        sigma = float(self.get_parameter('sigma').value)
        inv2sig2 = 1.0 / (2.0 * sigma * sigma)

        px = pts_b[:, 0][None, :]  # 1xP
        py = pts_b[:, 1][None, :]

        c, s = math.cos(float(yaw)), math.sin(float(yaw))

        # rotate points by yaw
        rx = c * px - s * py
        ry = s * px + c * py

        gx = cand_xy[:, 0:1] + rx
        gy = cand_xy[:, 1:2] + ry

        ix = np.floor((gx - ox) / res).astype(np.int32)
        iy = np.floor((gy - oy) / res).astype(np.int32)

        oob = (ix < 0) | (ix >= w) | (iy < 0) | (iy >= h)
        ix = np.clip(ix, 0, w - 1)
        iy = np.clip(iy, 0, h - 1)

        lin = iy * w + ix
        d = dist.reshape(-1)[lin]
        # Out-of-bounds treated as "far" (low score)
        d = np.where(oob, sigma * 3.0, d)

        # endpoints should lie near obstacles => smaller d => higher exp score
        return np.exp(-(d * d) * inv2sig2).mean(axis=1)  # M

    def score_candidates_multi(self, pts_sets, weights, cand_xy: np.ndarray, yaw_list: np.ndarray, top_k: int):
        """
        Multi-lidar fused scoring.
        pts_sets: list of Px2 point arrays (base frame)
        weights: list of weights per lidar
        Returns:
          best = (x,y,yaw,score)
          top  = list of (score, x, y, yaw) length <= top_k
        """
        # Filter empty sets
        filtered = [(p, w) for p, w in zip(pts_sets, weights) if p is not None and p.shape[0] > 0 and w > 0.0]
        if not filtered:
            return None, []

        pts_sets = [p for p, _ in filtered]
        weights = np.array([w for _, w in filtered], dtype=np.float32)
        weights = weights / (weights.sum() + 1e-9)

        best = None
        best_score = -1.0
        top = []

        # For each yaw, compute fused scores over all cand_xy
        for yaw in yaw_list:
            fused = None
            for p, w in zip(pts_sets, weights):
                sc = self._score_pose_set(p, cand_xy, float(yaw))
                fused = sc * float(w) if fused is None else fused + sc * float(w)

            # Track best
            k = int(np.argmax(fused))
            sc_best = float(fused[k])
            if sc_best > best_score:
                best_score = sc_best
                best = (float(cand_xy[k, 0]), float(cand_xy[k, 1]), float(yaw), best_score)

            # Collect top-K hypotheses cheaply
            if top_k > 0:
                # take a small batch per yaw to avoid huge list
                per_yaw = max(1, int(math.ceil(top_k / max(1, len(yaw_list)))))
                idx = np.argpartition(fused, -per_yaw)[-per_yaw:]
                for i in idx:
                    top.append((float(fused[i]), float(cand_xy[i, 0]), float(cand_xy[i, 1]), float(yaw)))

        if top_k > 0 and top:
            top.sort(key=lambda t: t[0], reverse=True)
            top = top[:top_k]

        return best, top

    # ---------------- Output helpers ----------------
    def make_pose_msg(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        msg.pose.covariance = [
            0.25,   0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.25,   0.0,   0.0,   0.0,   0.0,
            0.0,    0.0, 99999.0, 0.0,   0.0,   0.0,
            0.0,    0.0,   0.0, 99999.0, 0.0,   0.0,
            0.0,    0.0,   0.0,   0.0, 99999.0, 0.0,
            0.0,    0.0,   0.0,   0.0,   0.0,   0.0685
        ]
        return msg

    def call_amcl_set_pose(self, pose_msg: PoseWithCovarianceStamped) -> bool:
        client = self.set_pose_cli
        service_name = self.set_pose_service

        if not client.wait_for_service(timeout_sec=0.5):
            if self.set_pose_fallback_service and self.set_pose_fallback_cli.wait_for_service(timeout_sec=0.5):
                client = self.set_pose_fallback_cli
                service_name = self.set_pose_fallback_service
            else:
                self.get_logger().warn("AMCL set_pose services unavailable; publishing /initialpose instead.")
                self.initpose_pub.publish(pose_msg)
                return True

        req = SetInitialPose.Request()
        req.pose = pose_msg

        fut = client.call_async(req)
        for _ in range(30):  # ~1.5s max
            if fut.done():
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        if not fut.done():
            self.get_logger().error(f"AMCL {service_name} call timed out")
            return False

        try:
            _ = fut.result()
        except Exception as e:
            self.get_logger().error(f"AMCL {service_name} call failed: {e}")
            return False

        return True

    # ---------------- Main service ----------------
    def handle_trigger(self, req, resp):
        if self.map_msg is None or self.dist_m is None or self.free_mask is None:
            resp.success = False
            resp.message = "No /map yet."
            return resp

        # Get points from BOTH lidars (front + rear)
        pts_front = self.scan_to_points_base(self.front_scan)
        pts_rear = self.scan_to_points_base(self.rear_scan)

        if pts_front.shape[0] == 0 and pts_rear.shape[0] == 0:
            resp.success = False
            resp.message = "No usable scan points from front/rear (TF or scan invalid)."
            return resp

        # Sample candidate positions from free space
        free_yx = np.argwhere(self.free_mask > 0)
        if free_yx.shape[0] < 200:
            resp.success = False
            resp.message = "Map has too little free space?"
            return resp

        M = int(self.get_parameter('num_xy_samples').value)
        sel_n = min(M, free_yx.shape[0])
        sel = free_yx[np.random.choice(free_yx.shape[0], size=sel_n, replace=False)]

        reso = float(self.map_msg.info.resolution)
        ox = float(self.map_msg.info.origin.position.x)
        oy = float(self.map_msg.info.origin.position.y)

        cand_x = ox + (sel[:, 1].astype(np.float32) + 0.5) * reso
        cand_y = oy + (sel[:, 0].astype(np.float32) + 0.5) * reso
        cand_xy = np.stack([cand_x, cand_y], axis=1).astype(np.float32)

        # Coarse yaw grid
        yaw_steps = int(self.get_parameter('yaw_steps').value)
        yaw_list = np.linspace(-math.pi, math.pi, yaw_steps, endpoint=False).astype(np.float32)

        pts_sets = [pts_front, pts_rear]
        weights = [
            float(self.get_parameter('front_weight').value),
            float(self.get_parameter('rear_weight').value),
        ]
        top_k = int(self.get_parameter('top_k').value)

        best, top = self.score_candidates_multi(pts_sets, weights, cand_xy, yaw_list, top_k)

        if best is None:
            resp.success = False
            resp.message = "Scoring failed (no usable point sets)."
            return resp

        bx, by, byaw, bscore = best

        # Refinement around Top-K (recommended)
        if bool(self.get_parameter('refine_enable').value) and top:
            xy_std = float(self.get_parameter('refine_xy_std').value)
            per_hyp = int(self.get_parameter('refine_xy_per_hyp').value)

            yaw_range = float(self.get_parameter('refine_yaw_range_deg').value) * math.pi / 180.0
            yaw_fine_steps = int(self.get_parameter('refine_yaw_steps').value)

            refined_xy = []
            refined_yaw_centers = []

            for sc, x, y, yaw0 in top:
                # local xy samples
                dx = np.random.normal(0.0, xy_std, size=per_hyp).astype(np.float32)
                dy = np.random.normal(0.0, xy_std, size=per_hyp).astype(np.float32)
                rxy = np.stack([x + dx, y + dy], axis=1).astype(np.float32)
                refined_xy.append(rxy)

                refined_yaw_centers.append(float(yaw0))

            refined_xy = np.vstack(refined_xy).astype(np.float32)

            # fine yaw list is built around the best coarse yaw (cheaper) OR around each hyp (expensive).
            # Cheap + effective: refine around current best yaw only.
            yaw_fine = np.linspace(byaw - yaw_range, byaw + yaw_range, yaw_fine_steps).astype(np.float32)

            best_ref, _ = self.score_candidates_multi(pts_sets, weights, refined_xy, yaw_fine, top_k=0)
            if best_ref is not None and best_ref[3] >= bscore:
                bx, by, byaw, bscore = best_ref

        # Reject if score too low
        min_score = float(self.get_parameter('min_score').value)
        if bscore < min_score:
            resp.success = False
            resp.message = f"Rejected: best score {bscore:.4f} < min_score {min_score:.4f}"
            return resp

        pose_msg = self.make_pose_msg(bx, by, byaw)

        # Optional /initialpose publish (debug)
        if bool(self.get_parameter('publish_initialpose').value):
            self.initpose_pub.publish(pose_msg)

        ok = self.call_amcl_set_pose(pose_msg)
        if not ok:
            resp.success = False
            resp.message = "Failed to call AMCL set_pose (service unavailable or timed out)."
            return resp

        resp.success = True
        resp.message = f"Relocalized (front+rear fused): x={bx:.2f}, y={by:.2f}, yaw={byaw:.2f} rad, score={bscore:.4f}"
        return resp


def main():
    rclpy.init()
    node = CorrelativeRelocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
