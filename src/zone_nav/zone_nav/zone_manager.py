#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from zone_nav_interfaces.srv import SaveZone, GoToZone
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import os


class ZoneManager(Node):
    def __init__(self):
        super().__init__('zone_manager')
        
        # Subscriber to /goal_pose topic
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Service servers
        self.save_zone_srv = self.create_service(
            SaveZone,
            '/save_zone',
            self.save_zone_callback
        )
        
        self.go_to_zone_srv = self.create_service(
            GoToZone,
            '/go_to_zone',
            self.go_to_zone_callback
        )
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Store zones in memory and file
        self.zones = {"zones": {}}
        self.zones_file = os.path.expanduser('~/zones.yaml')
        self.load_zones()
        
        # Store the last received goal pose
        self.last_goal_pose = None
        
        self.get_logger().info('Zone Manager node started')
        self.get_logger().info(f'Zones file: {self.zones_file}')
    
    def goal_pose_callback(self, msg):
        """Store the last goal pose received from RViz"""
        # Ensure frame_id is set to 'map' if empty
        if msg.header.frame_id == "":
            msg.header.frame_id = "map"
        
        self.last_goal_pose = msg
        self.get_logger().info(f'Received goal pose [{msg.header.frame_id}]: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
    
    def save_zone_callback(self, request, response):
        """Save the last goal pose as a named zone"""
        zone_name = request.name
        
        if not self.last_goal_pose:
            response.ok = False
            response.message = 'No goal pose received yet. Please set a goal in RViz first.'
            self.get_logger().warn(response.message)
            return response
        
        # Store zone in nested structure
        self.zones.setdefault("zones", {})
        self.zones["zones"][zone_name] = {
            'position': {
                'x': self.last_goal_pose.pose.position.x,
                'y': self.last_goal_pose.pose.position.y,
                'z': self.last_goal_pose.pose.position.z
            },
            'orientation': {
                'x': self.last_goal_pose.pose.orientation.x,
                'y': self.last_goal_pose.pose.orientation.y,
                'z': self.last_goal_pose.pose.orientation.z,
                'w': self.last_goal_pose.pose.orientation.w
            },
            'frame_id': self.last_goal_pose.header.frame_id
        }
        
        # Save to file
        self.save_zones()
        
        response.ok = True
        response.message = f'Zone "{zone_name}" saved successfully at ({self.last_goal_pose.pose.position.x:.2f}, {self.last_goal_pose.pose.position.y:.2f})'
        self.get_logger().info(response.message)
        
        return response
    
    def go_to_zone_callback(self, request, response):
        """Navigate to a saved zone"""
        zone_name = request.name
        
        # Get zones dict
        zones = self.zones.get("zones", {})
        
        if zone_name not in zones:
            response.ok = False
            response.message = f'Zone "{zone_name}" not found. Available zones: {list(zones.keys())}'
            self.get_logger().warn(response.message)
            return response
        
        # Get zone data
        zone = zones[zone_name]
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            response.ok = False
            response.message = 'Nav2 action server not available'
            self.get_logger().error(response.message)
            return response
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = zone.get('frame_id', 'map')
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = zone['position']['x']
        goal_msg.pose.pose.position.y = zone['position']['y']
        goal_msg.pose.pose.position.z = zone['position']['z']
        
        goal_msg.pose.pose.orientation.x = zone['orientation']['x']
        goal_msg.pose.pose.orientation.y = zone['orientation']['y']
        goal_msg.pose.pose.orientation.z = zone['orientation']['z']
        goal_msg.pose.pose.orientation.w = zone['orientation']['w']
        
        # Send goal
        self.get_logger().info(f'Sending navigation goal to zone "{zone_name}"')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda fut: self._goal_response_cb(fut, zone_name))
        
        response.ok = True
        response.message = f'Navigation to zone "{zone_name}" initiated'
        self.get_logger().info(response.message)
        
        return response
    
    def _goal_response_cb(self, future, zone_name):
        """Handle Nav2 goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Nav2 rejected goal for zone "{zone_name}"')
            return
        self.get_logger().info(f'Nav2 accepted goal for zone "{zone_name}"')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, zone_name))
    
    def _result_cb(self, future, zone_name):
        """Handle Nav2 navigation result"""
        result = future.result().result
        self.get_logger().info(f'Nav2 finished zone "{zone_name}" with result: {result}')
    
    def load_zones(self):
        """Load zones from file"""
        if os.path.exists(self.zones_file):
            try:
                with open(self.zones_file, 'r') as f:
                    loaded = yaml.safe_load(f) or {}
                
                # Handle nested format (new) or flat format (backward compat)
                if isinstance(loaded, dict) and "zones" in loaded:
                    self.zones = loaded
                elif isinstance(loaded, dict):
                    # Old format: treat as flat zones dict, convert to nested
                    self.zones = {"zones": loaded}
                else:
                    self.zones = {"zones": {}}
                
                zone_count = len(self.zones.get("zones", {}))
                self.get_logger().info(f'Loaded {zone_count} zones from file')
            except Exception as e:
                self.get_logger().error(f'Failed to load zones: {e}')
        else:
            self.get_logger().info('No existing zones file found')
    
    def save_zones(self):
        """Save zones to file"""
        try:
            with open(self.zones_file, 'w') as f:
                yaml.dump(self.zones, f, default_flow_style=False)
            zone_count = len(self.zones.get("zones", {}))
            self.get_logger().info(f'Saved {zone_count} zones to file')
        except Exception as e:
            self.get_logger().error(f'Failed to save zones: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ZoneManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
