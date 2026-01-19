#!/usr/bin/env python3
"""
Scan Merger Node - Combines /scan and /scan2 into /scan_combined for AMCL localization
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        
        # Publisher for merged scan
        self.merged_pub = self.create_publisher(LaserScan, 'scan_combined', 10)
        
        # Storage for latest scans
        self.front_scan = None
        self.rear_scan = None
        
        # Subscribe to both lidars
        self.front_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.front_callback,
            10
        )
        
        self.rear_sub = self.create_subscription(
            LaserScan,
            '/scan2',
            self.rear_callback,
            10
        )
        
        # Timer to publish merged scan at fixed rate
        self.timer = self.create_timer(0.1, self.publish_merged)  # 10 Hz
        
        self.get_logger().info('Scan Merger Node started')
        self.get_logger().info('Merging /scan (front) + /scan2 (rear) → /scan_combined')
    
    def front_callback(self, msg):
        """Store front lidar scan"""
        self.front_scan = msg
    
    def rear_callback(self, msg):
        """Store rear lidar scan"""
        self.rear_scan = msg
    
    def publish_merged(self):
        """Merge and publish combined scan"""
        if not self.front_scan or not self.rear_scan:
            return
        
        # Create merged scan message
        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'base_link'
        
        # Full 360 degree coverage
        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = self.front_scan.angle_increment
        merged.time_increment = self.front_scan.time_increment
        merged.scan_time = self.front_scan.scan_time
        merged.range_min = min(self.front_scan.range_min, self.rear_scan.range_min)
        merged.range_max = max(self.front_scan.range_max, self.rear_scan.range_max)
        
        # Calculate number of points for 360 degrees
        num_points = int((merged.angle_max - merged.angle_min) / merged.angle_increment)
        
        # Initialize with max range (no obstacle)
        merged.ranges = [merged.range_max] * num_points
        merged.intensities = [0.0] * num_points
        
        # Add front scan data (covers -90 to +90 degrees relative to front)
        front_start_angle = self.front_scan.angle_min
        for i, r in enumerate(self.front_scan.ranges):
            angle = front_start_angle + i * self.front_scan.angle_increment
            # Map to merged scan indices
            merged_idx = int((angle - merged.angle_min) / merged.angle_increment)
            if 0 <= merged_idx < num_points:
                if self.front_scan.range_min < r < self.front_scan.range_max:
                    merged.ranges[merged_idx] = r
                    if i < len(self.front_scan.intensities):
                        merged.intensities[merged_idx] = self.front_scan.intensities[i]
        
        # Add rear scan data (covers rear hemisphere, rotated 180 degrees)
        # Rear lidar is mounted facing backwards, so we need to add PI offset
        rear_start_angle = self.rear_scan.angle_min + math.pi
        for i, r in enumerate(self.rear_scan.ranges):
            angle = rear_start_angle + i * self.rear_scan.angle_increment
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            # Map to merged scan indices
            merged_idx = int((angle - merged.angle_min) / merged.angle_increment)
            if 0 <= merged_idx < num_points:
                if self.rear_scan.range_min < r < self.rear_scan.range_max:
                    # Only overwrite if closer than existing value
                    if r < merged.ranges[merged_idx]:
                        merged.ranges[merged_idx] = r
                        if i < len(self.rear_scan.intensities):
                            merged.intensities[merged_idx] = self.rear_scan.intensities[i]
        
        # Publish merged scan
        self.merged_pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
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


if __name__ == '__main__':
    main()
