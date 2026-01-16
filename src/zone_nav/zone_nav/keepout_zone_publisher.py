#!/usr/bin/env python3
"""
Keepout Zone Publisher for Nav2
Publishes keepout zones from map editor to Nav2 costmap filters
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CostmapFilterInfo
from geometry_msgs.msg import Polygon, Point32
import yaml
import os


class KeepoutZonePublisher(Node):
    def __init__(self):
        super().__init__('keepout_zone_publisher')
        
        # Publisher for costmap filter info
        self.filter_info_pub = self.create_publisher(
            CostmapFilterInfo,
            '/costmap_filter_info',
            10
        )
        
        # Load keepout zones from map editor export
        self.zones_file = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/keepout_zones.yaml')
        
        # Publish filter info periodically
        self.timer = self.create_timer(1.0, self.publish_filter_info)
        
        self.get_logger().info('Keepout Zone Publisher started')
        self.get_logger().info(f'Loading zones from: {self.zones_file}')
    
    def publish_filter_info(self):
        """Publish costmap filter info for keepout zones"""
        if not os.path.exists(self.zones_file):
            return
        
        try:
            with open(self.zones_file, 'r') as f:
                data = yaml.safe_load(f)
            
            zones = data.get('keepout_zones', [])
            
            if not zones:
                return
            
            # Create filter info message
            msg = CostmapFilterInfo()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.type = CostmapFilterInfo.KEEPOUT_FILTER
            msg.filter_mask_topic = '/keepout_filter_mask'
            msg.base = 0.0
            msg.multiplier = 1.0
            
            self.filter_info_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish filter info: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutZonePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
