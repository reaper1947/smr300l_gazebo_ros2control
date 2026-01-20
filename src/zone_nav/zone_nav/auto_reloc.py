#!/usr/bin/env python3
import math
import random
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


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def tf_to_xy_yaw(tf):
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    # yaw from quaternion (z,w only if planar, but do full)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tx, ty, yaw


class CorrelativeRelocalizer(Node):
    def __init__(self):
        super().__init__('correlative_relocalizer')

        # ---- Parameters you’ll tune ----
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan2_topic', '/scan2')
        self.declare_parameter('base_frame', 'base_link')  # or 'chassis'
        self.declare_parameter('sigma', 0.20)              # meters
        self.declare_parameter('num_xy_samples', 1800)     # global candidates
        self.declare_parameter('yaw_steps', 36)            # 10° steps
        self.declare_parameter('scan_downsample', 4)       # every Nth beam
        self.declare_parameter('max_points', 90)           # cap per scan

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self.map_cb, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        self.scan2_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan2_topic').value, self.scan2_cb, 10)

        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.srv = self.create_service(Trigger, '/auto_relocate', self.handle_trigger)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Cached data
        self.map_msg = None
        self.dist_m = None
        self.free_mask = None  # uint8 image: free==1
        self.scan = None
        self.scan2 = None

        self.get_logger().info("Correlative relocalizer ready: call /auto_relocate (std_srvs/Trigger)")

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        # Treat unknown as NOT free (0), free as 1
        free = (data == 0).astype(np.uint8)

        # OpenCV distanceTransform computes distance to nearest zero pixel for non-zero pixels
        # We want distance from free cells (1) to nearest non-free (0).
        dist_px = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        self.dist_m = dist_px * float(msg.info.resolution)
        self.free_mask = free

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def scan2_cb(self, msg: LaserScan):
        self.scan2 = msg

    def scan_to_points_base(self, scan: LaserScan, laser_frame: str):
        """Return Nx2 points in base frame."""
        if scan is None:
            return np.zeros((0, 2), dtype=np.float32)

        base_frame = self.get_parameter('base_frame').value

        try:
            tf = self.tf_buffer.lookup_transform(base_frame, laser_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"TF {base_frame}<-{laser_frame} not available: {e}")
            return np.zeros((0, 2), dtype=np.float32)

        lx, ly, lyaw = tf_to_xy_yaw(tf)

        ds = int(self.get_parameter('scan_downsample').value)
        max_pts = int(self.get_parameter('max_points').value)

        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
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

        # transform laser->base (planar): rotate then translate
        c, s = math.cos(lyaw), math.sin(lyaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        pts_b = (pts_l @ R.T) + np.array([lx, ly], dtype=np.float32)
        return pts_b.astype(np.float32)

    def score_candidates(self, pts_b: np.ndarray, cand_xy: np.ndarray, yaw_list: np.ndarray):
        """Return best (x,y,yaw,score). Vectorized per yaw."""
        if pts_b.shape[0] == 0:
            return None

        msg = self.map_msg
        dist = self.dist_m
        h, w = dist.shape
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)

        sigma = float(self.get_parameter('sigma').value)
        inv2sig2 = 1.0 / (2.0 * sigma * sigma)

        best_score = -1.0
        best = None

        # Pre-split points
        px = pts_b[:, 0][None, :]  # 1xP
        py = pts_b[:, 1][None, :]

        dist_flat = dist.reshape(-1)

        for yaw in yaw_list:
            c, s = math.cos(yaw), math.sin(yaw)
            # rotate points in base by yaw into map frame (relative)
            rx = c * px - s * py  # 1xP
            ry = s * px + c * py

            # Translate by each candidate position: MxP
            gx = cand_xy[:, 0:1] + rx
            gy = cand_xy[:, 1:2] + ry

            ix = np.floor((gx - ox) / res).astype(np.int32)
            iy = np.floor((gy - oy) / res).astype(np.int32)

            oob = (ix < 0) | (ix >= w) | (iy < 0) | (iy >= h)
            ix = np.clip(ix, 0, w - 1)
            iy = np.clip(iy, 0, h - 1)

            lin = iy * w + ix
            d = dist_flat[lin]  # MxP
            # penalize out-of-bounds by making distance big
            d = np.where(oob, sigma * 3.0, d)

            score = np.exp(-(d * d) * inv2sig2).mean(axis=1)  # M
            k = int(np.argmax(score))
            if float(score[k]) > best_score:
                best_score = float(score[k])
                best = (float(cand_xy[k, 0]), float(cand_xy[k, 1]), float(yaw), best_score)

        return best

    def handle_trigger(self, req, resp):
        if self.map_msg is None or self.dist_m is None or self.free_mask is None:
            resp.success = False
            resp.message = "No /map yet."
            return resp
        if self.scan is None:
            resp.success = False
            resp.message = "No /scan yet."
            return resp

        # Build scan points in base frame (use scan frame_id)
        pts1 = self.scan_to_points_base(self.scan, self.scan.header.frame_id)
        pts2 = self.scan_to_points_base(self.scan2, self.scan2.header.frame_id) if self.scan2 else np.zeros((0, 2), np.float32)

        # Sample candidate positions from free cells
        free_yx = np.argwhere(self.free_mask > 0)
        if free_yx.shape[0] < 200:
            resp.success = False
            resp.message = "Map has too little free space?"
            return resp

        M = int(self.get_parameter('num_xy_samples').value)
        sel = free_yx[np.random.choice(free_yx.shape[0], size=min(M, free_yx.shape[0]), replace=False)]
        # y,x -> world (cell center)
        reso = float(self.map_msg.info.resolution)
        ox = float(self.map_msg.info.origin.position.x)
        oy = float(self.map_msg.info.origin.position.y)
        cand_x = ox + (sel[:, 1].astype(np.float32) + 0.5) * reso
        cand_y = oy + (sel[:, 0].astype(np.float32) + 0.5) * reso
        cand_xy = np.stack([cand_x, cand_y], axis=1).astype(np.float32)

        # Yaw grid
        yaw_steps = int(self.get_parameter('yaw_steps').value)
        yaw_list = np.linspace(-math.pi, math.pi, yaw_steps, endpoint=False).astype(np.float32)

        best1 = self.score_candidates(pts1, cand_xy, yaw_list)
        best2 = self.score_candidates(pts2, cand_xy, yaw_list) if pts2.shape[0] else None

        if best1 is None and best2 is None:
            resp.success = False
            resp.message = "No usable scan points (TF/scan invalid)."
            return resp

        # Combine by preferring sum score if both exist (simple but effective)
        if best2 is None:
            bx, by, byaw, bscore = best1
        elif best1 is None:
            bx, by, byaw, bscore = best2
        else:
            # re-score around best1 yaw for scan2 quickly would be nicer, but keep it simple:
            # choose whichever is higher score; in practice you can also sum by evaluating both at same pose.
            bx, by, byaw, bscore = best1 if best1[3] >= best2[3] else best2

        # Publish /initialpose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = bx
        msg.pose.pose.position.y = by
        qx, qy, qz, qw = yaw_to_quat(byaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covariance (tune tighter if score is strong)
        msg.pose.covariance = [0.25,0,0,0,0,0,
                              0,0.25,0,0,0,0,
                              0,0,99999,0,0,0,
                              0,0,0,99999,0,0,
                              0,0,0,0,99999,0,
                              0,0,0,0,0,0.0685]

        self.initpose_pub.publish(msg)

        resp.success = True
        resp.message = f"Best pose: x={bx:.2f}, y={by:.2f}, yaw={byaw:.2f} rad, score={bscore:.4f}"
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
