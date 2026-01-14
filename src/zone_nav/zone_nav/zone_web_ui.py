#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from zone_nav_interfaces.srv import SaveZone, GoToZone
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from flask import Flask, render_template, jsonify, request, send_from_directory
from flask_cors import CORS
import threading
import yaml
import os
import math
from PIL import Image
import io
import base64


class ZoneWebServer(Node):
    def __init__(self):
        super().__init__('zone_web_server')
        
        # Service clients
        self.save_zone_client = self.create_client(SaveZone, '/save_zone')
        self.go_to_zone_client = self.create_client(GoToZone, '/go_to_zone')
        
        # Safety controller services
        self.safety_estop_client = self.create_client(SetBool, 'safety/emergency_stop')
        self.safety_override_client = self.create_client(SetBool, 'safety/override')
        self.safety_status_client = self.create_client(Trigger, 'safety/status')
        
        # Publisher for goal poses
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Publisher for manual velocity control and emergency stop
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for control mode switching
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        
        # Track current control mode
        self.current_mode = 'manual'  # Start in manual mode
        self.mode_sub = self.create_subscription(String, '/control_mode', self.mode_status_callback, 10)
        
        # Action client for Nav2 navigation (to cancel goals)
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_nav_goal_handle = None
        
        # Timer for continuous safety stop commands
        self.safety_stop_timer = None
        
        # Safety parameters
        self.confidence_threshold = 30.0  # Stop if confidence drops below this
        self.confidence_resume_threshold = 35.0  # Resume when confidence goes above this (hysteresis)
        self.safety_stop_active = False
        self.safety_override_active = False  # Manual override flag
        self.last_confidence_warning = 0.0
        self.estop_active = False  # Emergency stop flag
        # Robot dimensions: 0.80m × 0.54m, half-width = 0.27m
        # Safety radius = robot half-width (0.27m) + clearance margin (0.08m) = 0.35m
        self.safety_radius = 0.35  # Minimum distance to obstacles (meters)
        
        # Sequential navigation state
        self.sequence_active = False
        self.sequence_zones = []
        self.current_sequence_index = 0
        self.sequence_goal_handle = None
        
        # Path following state
        self.path_following_active = False
        self.path_waypoints = []
        self.current_waypoint_index = 0
        self.path_goal_handle = None
        
        # Subscribe to robot position - match AMCL's QoS profile
        self.robot_pose = None
        self.localization_confidence = 0.0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos
        )
        
        self.get_logger().info('Subscribed to /amcl_pose with RELIABLE + TRANSIENT_LOCAL QoS')
        
        # Subscribe to navigation path
        self.current_path = None
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        self.get_logger().info('Subscribed to /plan for navigation path')
        
        # Subscribe to lidar scan
        self.laser_scan = None
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Subscribed to /scan for lidar data')
        
        # Load map info
        self.map_path = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/smr_map.yaml')
        self.zones_file = os.path.expanduser('~/zones.yaml')
        self.map_info = self.load_map_info()
        
        self.get_logger().info('Zone Web Server node started')
        self.get_logger().info(f'Map: {self.map_info.get("image_path", "Not found")}')
    
    def mode_status_callback(self, msg):
        """Track current control mode"""
        self.current_mode = msg.data.lower()
        self.get_logger().debug(f'Mode updated: {self.current_mode}')
    
    def pose_callback(self, msg):
        """Store robot position from AMCL"""
        # AMCL publishes PoseWithCovarianceStamped - access nested pose.pose
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        }
        
        # Calculate localization confidence from covariance
        # Lower covariance = higher confidence
        # Covariance matrix is 6x6, we look at x, y position variance (indices 0, 7)
        cov_x = msg.pose.covariance[0]  # Variance in x
        cov_y = msg.pose.covariance[7]  # Variance in y
        cov_theta = msg.pose.covariance[35]  # Variance in theta
        
        # Average position variance
        pos_variance = (cov_x + cov_y) / 2.0
        
        # Convert to confidence (0-100%)
        # Lower variance = higher confidence
        # Using exponential decay: confidence = 100 * e^(-k * variance)
        k = 5.0  # Tuning factor
        self.localization_confidence = min(100.0, max(0.0, 100.0 * math.exp(-k * pos_variance)))
        
        # Safety check: Stop robot if confidence is too low
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # If confidence improved naturally, clear override
        if self.localization_confidence > self.confidence_resume_threshold:
            self.safety_override_active = False
        
        if self.localization_confidence < self.confidence_threshold:
            if not self.safety_stop_active:
                # Don't stop if override is active, but warn
                if self.safety_override_active:
                    if (current_time - self.last_confidence_warning) > 5.0:
                        self.get_logger().warn(f'⚠️  WARNING: Confidence still low ({self.localization_confidence:.1f}%) but override is active')
                        self.last_confidence_warning = current_time
                else:
                    self.safety_stop_active = True
                    self.get_logger().warn(f'⚠️  SAFETY STOP: Localization confidence too low ({self.localization_confidence:.1f}% < {self.confidence_threshold}%). Stopping robot.')
                    self._emergency_stop()
                    # Start continuous stop timer
                    if self.safety_stop_timer is None:
                        self.safety_stop_timer = self.create_timer(0.1, self._publish_stop_command)
        elif self.localization_confidence > self.confidence_resume_threshold:
            if self.safety_stop_active:
                self.safety_stop_active = False
                self.get_logger().info(f'✓ Confidence restored ({self.localization_confidence:.1f}% > {self.confidence_resume_threshold}%). Robot can resume navigation.')
                # Stop the continuous stop timer
                if self.safety_stop_timer is not None:
                    self.safety_stop_timer.cancel()
                    self.safety_stop_timer = None
        
        # Log less frequently to avoid spam
        self.get_logger().debug(f'Robot pose updated: x={self.robot_pose["x"]:.2f}, y={self.robot_pose["y"]:.2f}, theta={self.robot_pose["theta"]:.2f}, confidence={self.localization_confidence:.1f}%')
    
    def path_callback(self, msg):
        """Store the current navigation path"""
        if len(msg.poses) > 0:
            self.current_path = [
                {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y
                }
                for pose in msg.poses
            ]
            self.get_logger().debug(f'Path updated with {len(self.current_path)} waypoints')
        else:
            self.current_path = None
    
    def scan_callback(self, msg):
        """Store lidar scan data"""
        if not self.robot_pose:
            return
        
        # Sample every Nth point to reduce data size
        sample_rate = 5
        points = []
        
        # Minimum range filter - ignore points closer than this (likely the robot itself)
        min_valid_range = 0.35  # Ignore obstacles closer than 35cm (smaller than robot footprint)
        
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            # Filter: valid range AND minimum distance AND within sensor limits
            if i % sample_rate == 0 and r > min_valid_range and msg.range_min < r < msg.range_max:
                # Convert polar to cartesian relative to robot
                x_local = r * math.cos(angle)
                y_local = r * math.sin(angle)
                
                # Transform to world coordinates
                robot_theta = self.robot_pose['theta']
                x_world = self.robot_pose['x'] + x_local * math.cos(robot_theta) - y_local * math.sin(robot_theta)
                y_world = self.robot_pose['y'] + x_local * math.sin(robot_theta) + y_local * math.cos(robot_theta)
                
                points.append({'x': x_world, 'y': y_world})
            
            angle += msg.angle_increment
        
        self.laser_scan = points
    
    def validate_zone_position(self, x, y):
        """Validate if zone position is safe and reachable.
        
        Returns:
            tuple: (is_valid, warning_message)
        """
        # Check if we have laser scan data
        if not self.laser_scan or len(self.laser_scan) == 0:
            return True, None  # If no scan data, allow placement (trust the user)
        
        # Only check obstacles within a reasonable radius around the zone (not the entire map)
        check_radius = 2.0  # Only check obstacles within 2 meters of the zone
        min_distance = float('inf')
        obstacles_found = 0
        
        for point in self.laser_scan:
            # First check if this obstacle is even near the zone we're trying to place
            dist_to_zone = math.sqrt((point['x'] - x)**2 + (point['y'] - y)**2)
            
            # Only consider obstacles within check_radius of the zone
            if dist_to_zone < check_radius:
                obstacles_found += 1
                if dist_to_zone < min_distance:
                    min_distance = dist_to_zone
        
        # If no obstacles found nearby, zone is safe
        if obstacles_found == 0:
            return True, None
        
        # Validate safety radius (robot half-width + margin)
        if min_distance < self.safety_radius:
            return False, f"Zone too close to obstacle ({min_distance:.2f}m). Minimum safe distance is {self.safety_radius}m. Found {obstacles_found} nearby obstacles."
        
        # Additional validation: Check if zone is within reasonable map bounds
        max_range = 50.0  # Maximum reasonable distance from origin
        distance_from_origin = math.sqrt(x**2 + y**2)
        
        if distance_from_origin > max_range:
            return False, f"Zone too far from origin ({distance_from_origin:.1f}m). Maximum distance is {max_range}m."
        
        return True, None
    
    def _publish_stop_command(self):
        """Continuously publish stop commands while safety stop is active"""
        if self.safety_stop_active:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
    
    def _emergency_stop(self):
        """Send zero velocity command and cancel Nav2 goals to stop the robot immediately"""
        # Send stop command to high-priority joystick topic (overrides Nav2)
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        
        # Cancel any active Nav2 navigation goals
        if self.current_nav_goal_handle is not None:
            try:
                self.get_logger().warn('Canceling active navigation goal due to low confidence')
                cancel_future = self.current_nav_goal_handle.cancel_goal_async()
                self.current_nav_goal_handle = None
            except Exception as e:
                self.get_logger().error(f'Failed to cancel navigation goal: {e}')
    
    def load_map_info(self):
        """Load map YAML configuration"""
        try:
            with open(self.map_path, 'r') as f:
                map_data = yaml.safe_load(f)
            
            # Get absolute path to map image
            image_path = map_data.get('image', '')
            if not os.path.isabs(image_path):
                map_dir = os.path.dirname(self.map_path)
                image_path = os.path.join(map_dir, image_path)
            
            return {
                'image_path': image_path,
                'resolution': map_data.get('resolution', 0.05),
                'origin': map_data.get('origin', [0, 0, 0]),
                'negate': map_data.get('negate', 0)
            }
        except Exception as e:
            self.get_logger().error(f'Failed to load map info: {e}')
            return {}
    
    def load_zones(self):
        """Load zones from YAML file"""
        if os.path.exists(self.zones_file):
            try:
                with open(self.zones_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                return data.get('zones', {})
            except Exception as e:
                self.get_logger().error(f'Failed to load zones: {e}')
                return {}
        return {}
    
    def delete_zone(self, name):
        """Delete a zone from the zones file"""
        try:
            zones = self.load_zones()
            
            if name not in zones:
                return {'ok': False, 'message': f'Zone "{name}" not found'}
            
            # Remove the zone
            del zones[name]
            
            # Save back to file
            with open(self.zones_file, 'w') as f:
                yaml.dump({'zones': zones}, f, default_flow_style=False)
            
            self.get_logger().info(f'Zone "{name}" deleted successfully')
            return {'ok': True, 'message': f'Zone "{name}" deleted successfully'}
            
        except Exception as e:
            self.get_logger().error(f'Failed to delete zone: {e}')
            return {'ok': False, 'message': f'Error deleting zone: {str(e)}'}
    
    def force_resume_navigation(self):
        """Manually override safety stop and resume navigation"""
        if self.safety_stop_active:
            self.safety_override_active = True
            self.safety_stop_active = False
            
            # Stop the continuous stop timer
            if self.safety_stop_timer is not None:
                self.safety_stop_timer.cancel()
                self.safety_stop_timer = None
            
            self.get_logger().warn(f'⚠️  MANUAL OVERRIDE: Safety stop overridden by user. Confidence: {self.localization_confidence:.1f}%')
            return {'ok': True, 'message': f'Safety stop overridden. Robot can now navigate (confidence: {self.localization_confidence:.1f}%)'}
        else:
            return {'ok': False, 'message': 'Safety stop is not active'}
    
    def follow_drawn_path(self, path_points):
        """Follow a drawn path by navigating through waypoints sequentially using Nav2 action"""
        # Check if in manual mode - block autonomous navigation
        if self.current_mode == 'manual':
            return {'ok': False, 'message': '⚠️  Cannot follow path: Robot is in MANUAL MODE. Deactivate manual mode first.'}
        
        if not path_points or len(path_points) < 2:
            return {'ok': False, 'message': 'Path must have at least 2 points'}
        
        if self.path_following_active:
            return {'ok': False, 'message': 'Path following already active. Stop it first.'}
        
        # Switch to path mode
        mode_msg = String()
        mode_msg.data = 'path'
        self.mode_pub.publish(mode_msg)
        self.current_mode = 'path'  # Update immediately
        self.get_logger().info('🛤️  Switching to PATH mode')
        
        # Check safety
        if self.safety_stop_active and not self.safety_override_active:
            return {'ok': False, 'message': f'Cannot navigate: Localization confidence too low ({self.localization_confidence:.1f}%). Improve robot localization first or use "Force Resume".'}
        
        self.path_following_active = True
        self.path_waypoints = path_points
        self.current_waypoint_index = 0
        
        self.get_logger().info(f'Starting path following with {len(path_points)} waypoints')
        
        # Start with first waypoint using Nav2 action
        self._navigate_to_next_waypoint()
        
        return {'ok': True, 'message': f'Path following started: {len(path_points)} waypoints'}
    
    def _navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the path using Nav2 action"""
        import math
        
        if not self.path_following_active or self.current_waypoint_index >= len(self.path_waypoints):
            return
        
        point = self.path_waypoints[self.current_waypoint_index]
        
        # Check safety
        if self.safety_stop_active and not self.safety_override_active:
            self.get_logger().warn(f'Cannot continue path: Safety stop active (confidence: {self.localization_confidence:.1f}%)')
            self.stop_path_following()
            return
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.path_waypoints)}: ({point["x"]:.2f}, {point["y"]:.2f})')
        
        # Wait for Nav2 action server
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.stop_path_following()
            return
        
        # Calculate orientation to next point or use provided theta
        if 'theta' in point:
            theta = float(point['theta'])
        elif self.current_waypoint_index < len(self.path_waypoints) - 1:
            # Calculate angle to next point
            next_point = self.path_waypoints[self.current_waypoint_index + 1]
            dx = next_point['x'] - point['x']
            dy = next_point['y'] - point['y']
            theta = math.atan2(dy, dx)
        else:
            # Last point, face forward
            theta = 0.0
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(point['x'])
        goal_msg.pose.pose.position.y = float(point['y'])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Send goal and wait for acceptance
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._path_goal_response_callback)
    
    def stop_path_following(self):
        """Stop the current path following"""
        if not self.path_following_active:
            return {'ok': False, 'message': 'No path following is active'}
        
        self.path_following_active = False
        self.path_waypoints = []
        self.current_waypoint_index = 0
        
        # Cancel active navigation goal
        if self.path_goal_handle is not None:
            try:
                self.path_goal_handle.cancel_goal_async()
                self.path_goal_handle = None
            except Exception as e:
                self.get_logger().error(f'Failed to cancel path goal: {e}')
        
        self.get_logger().info('Path following stopped')
        return {'ok': True, 'message': 'Path following stopped'}
    
    def _path_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance for path waypoint"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Path waypoint goal rejected by Nav2. Stopping path following.')
            self.stop_path_following()
            return
        
        self.path_goal_handle = goal_handle
        self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1}/{len(self.path_waypoints)} accepted by Nav2')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._path_result_callback)
    
    def _path_result_callback(self, future):
        """Handle completion of a waypoint in the path"""
        if not self.path_following_active:
            return
        
        try:
            result = future.result()
            status = result.status
            
            # Status codes: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status == 4:  # SUCCEEDED
                self.get_logger().info(f'✓ Reached waypoint {self.current_waypoint_index + 1}/{len(self.path_waypoints)}')
                
                # Move to next waypoint
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index >= len(self.path_waypoints):
                    # Path complete
                    self.get_logger().info('✓ Path following completed successfully!')
                    self.path_following_active = False
                    self.path_waypoints = []
                    self.current_waypoint_index = 0
                    self.path_goal_handle = None
                else:
                    # Continue to next waypoint after a brief pause
                    self.get_logger().info('Moving to next waypoint...')
                    pause_timer = self.create_timer(1.0, self._path_timer_callback)
                    # Store timer to prevent garbage collection
                    self._path_pause_timer = pause_timer
            else:
                # Navigation failed or was canceled
                self.get_logger().error(f'Navigation to waypoint failed with status {status}. Stopping path following.')
                self.stop_path_following()
                
        except Exception as e:
            self.get_logger().error(f'Path result callback error: {e}. Stopping path following.')
            self.stop_path_following()
    
    def _path_timer_callback(self):
        """Timer callback to continue path after pause"""
        if hasattr(self, '_path_pause_timer'):
            self._path_pause_timer.cancel()
            self._path_pause_timer.destroy()
            delattr(self, '_path_pause_timer')
        
        if self.path_following_active:
            self._navigate_to_next_waypoint()
    
    def save_zone(self, name, x, y, theta):
        """Call save_zone service"""
        if not self.save_zone_client.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'message': 'Save zone service not available'}
        
        # First publish the goal pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import math
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.goal_pub.publish(pose_msg)
        
        # Give it a moment to be received
        import time
        time.sleep(0.1)
        
        # Call service asynchronously (non-blocking)
        request = SaveZone.Request()
        request.name = name
        future = self.save_zone_client.call_async(request)
        
        # Add callback to log result without blocking
        future.add_done_callback(lambda f: self._save_zone_done_callback(f, name))
        
        # Return immediately so UI stays responsive
        return {'ok': True, 'message': f'Saving zone "{name}"...'}
    
    def _save_zone_done_callback(self, future, name):
        """Handle save zone service response"""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f'Zone "{name}" saved successfully')
            else:
                self.get_logger().error(f'Failed to save zone "{name}": {response.message}')
        except Exception as e:
            self.get_logger().error(f'Save zone service call failed: {e}')
    
    def start_sequence(self, zone_names):
        """Start navigating through zones in sequence"""
        # Check if in manual mode - block autonomous navigation
        if self.current_mode == 'manual':
            return {'ok': False, 'message': '⚠️  Cannot start sequence: Robot is in MANUAL MODE. Deactivate manual mode button first.'}
        
        if not zone_names or len(zone_names) == 0:
            return {'ok': False, 'message': 'No zones provided for sequence'}
        
        if self.sequence_active:
            return {'ok': False, 'message': 'Sequence already running. Stop it first.'}
        
        # Switch to sequence mode
        mode_msg = String()
        mode_msg.data = 'sequence'
        self.mode_pub.publish(mode_msg)
        self.current_mode = 'sequence'  # Update immediately
        self.get_logger().info('📝 Switching to SEQUENCE mode')
        
        # Load zones to get coordinates
        all_zones = self.load_zones()
        
        # Validate all zones exist
        for zone_name in zone_names:
            if zone_name not in all_zones:
                return {'ok': False, 'message': f'Zone "{zone_name}" not found'}
        
        self.sequence_active = True
        self.sequence_zones = zone_names
        self.current_sequence_index = 0
        
        self.get_logger().info(f'Starting sequence navigation through {len(zone_names)} zones: {zone_names}')
        
        # Start with first zone using Nav2 action
        self._navigate_to_next_zone()
        
        return {'ok': True, 'message': f'Sequence started: {len(zone_names)} zones'}
    
    def _navigate_to_next_zone(self):
        """Navigate to the next zone in the sequence using Nav2 action"""
        if not self.sequence_active or self.current_sequence_index >= len(self.sequence_zones):
            return
        
        zone_name = self.sequence_zones[self.current_sequence_index]
        all_zones = self.load_zones()
        
        if zone_name not in all_zones:
            self.get_logger().error(f'Zone "{zone_name}" not found. Stopping sequence.')
            self.stop_sequence()
            return
        
        zone = all_zones[zone_name]
        
        # Check safety
        if self.safety_stop_active and not self.safety_override_active:
            self.get_logger().warn(f'Cannot continue sequence: Safety stop active (confidence: {self.localization_confidence:.1f}%)')
            self.stop_sequence()
            return
        
        self.get_logger().info(f'Navigating to zone {self.current_sequence_index + 1}/{len(self.sequence_zones)}: "{zone_name}"')
        
        # Wait for Nav2 action server
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.stop_sequence()
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = zone.get('frame_id', 'map')
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(zone['position']['x'])
        goal_msg.pose.pose.position.y = float(zone['position']['y'])
        goal_msg.pose.pose.position.z = float(zone['position']['z'])
        goal_msg.pose.pose.orientation.x = float(zone['orientation']['x'])
        goal_msg.pose.pose.orientation.y = float(zone['orientation']['y'])
        goal_msg.pose.pose.orientation.z = float(zone['orientation']['z'])
        goal_msg.pose.pose.orientation.w = float(zone['orientation']['w'])
        
        # Send goal and wait for acceptance
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._sequence_goal_response_callback)
    
    def stop_sequence(self):
        """Stop the current sequence navigation"""
        if not self.sequence_active:
            return {'ok': False, 'message': 'No sequence is running'}
        
        self.sequence_active = False
        self.sequence_zones = []
        self.current_sequence_index = 0
        
        # Cancel active navigation goal
        if self.sequence_goal_handle is not None:
            try:
                self.sequence_goal_handle.cancel_goal_async()
                self.sequence_goal_handle = None
            except Exception as e:
                self.get_logger().error(f'Failed to cancel sequence goal: {e}')
        
        self.get_logger().info('Sequence navigation stopped')
        return {'ok': True, 'message': 'Sequence stopped'}
    
    def get_sequence_status(self):
        """Get current sequence status"""
        return {
            'active': self.sequence_active,
            'zones': self.sequence_zones,
            'current_index': self.current_sequence_index,
            'current_zone': self.sequence_zones[self.current_sequence_index] if self.sequence_active and self.current_sequence_index < len(self.sequence_zones) else None,
            'total_zones': len(self.sequence_zones)
        }
    
    def _sequence_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance for sequence"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Sequence goal rejected by Nav2. Stopping sequence.')
            self.stop_sequence()
            return
        
        self.sequence_goal_handle = goal_handle
        self.get_logger().info(f'Goal accepted by Nav2 for zone {self.current_sequence_index + 1}/{len(self.sequence_zones)}')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._sequence_result_callback)
    
    def _sequence_result_callback(self, future):
        """Handle completion of a zone navigation in the sequence"""
        if not self.sequence_active:
            return
        
        try:
            result = future.result()
            status = result.status
            
            # Status codes: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status == 4:  # SUCCEEDED
                current_zone = self.sequence_zones[self.current_sequence_index]
                self.get_logger().info(f'✓ Reached zone {self.current_sequence_index + 1}/{len(self.sequence_zones)}: "{current_zone}"')
                
                # Move to next zone
                self.current_sequence_index += 1
                
                if self.current_sequence_index >= len(self.sequence_zones):
                    # Sequence complete
                    self.get_logger().info('✓ Sequence navigation completed successfully!')
                    self.sequence_active = False
                    self.sequence_zones = []
                    self.current_sequence_index = 0
                    self.sequence_goal_handle = None
                else:
                    # Continue to next zone after a brief pause
                    self.get_logger().info('Pausing briefly before next zone...')
                    pause_timer = self.create_timer(2.0, self._sequence_timer_callback)
                    # Store timer to prevent it from being garbage collected
                    self._sequence_pause_timer = pause_timer
            else:
                # Navigation failed or was canceled
                self.get_logger().error(f'Navigation to zone failed with status {status}. Stopping sequence.')
                self.stop_sequence()
                
        except Exception as e:
            self.get_logger().error(f'Sequence result callback error: {e}. Stopping sequence.')
            self.stop_sequence()
    
    def _sequence_timer_callback(self):
        """Timer callback to continue sequence after pause"""
        if hasattr(self, '_sequence_pause_timer'):
            self._sequence_pause_timer.cancel()
            self._sequence_pause_timer.destroy()
            delattr(self, '_sequence_pause_timer')
        
        if self.sequence_active:
            self._navigate_to_next_zone()
    
    def go_to_zone(self, name, is_sequence=False):
        """Call go_to_zone service"""
        # Check if in manual mode - block autonomous navigation
        if self.current_mode == 'manual' and not is_sequence:
            return {'ok': False, 'message': '⚠️  Cannot navigate: Robot is in MANUAL MODE. Deactivate manual mode first.'}
        
        if not self.go_to_zone_client.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'message': 'Go to zone service not available'}
        
        # Switch to zones mode (unless this is part of a sequence)
        if not is_sequence:
            mode_msg = String()
            mode_msg.data = 'zones'
            self.mode_pub.publish(mode_msg)
            self.current_mode = 'zones'  # Update immediately
            self.get_logger().info('🎯 Switching to ZONES mode')
        
        # Clear safety stop if confidence is good
        if self.localization_confidence > self.confidence_resume_threshold:
            self.safety_stop_active = False
            self.safety_override_active = False
        
        # Check if safety stop is active (unless overridden)
        if self.safety_stop_active and not self.safety_override_active:
            if is_sequence:
                self.stop_sequence()
            return {'ok': False, 'message': f'Cannot navigate: Localization confidence too low ({self.localization_confidence:.1f}%). Improve robot localization first or use "Force Resume" if obstacles are cleared.'}
        
        request = GoToZone.Request()
        request.name = name
        # Send the request asynchronously without blocking
        future = self.go_to_zone_client.call_async(request)
        
        # If part of sequence, add callback to continue to next zone
        if is_sequence:
            future.add_done_callback(self._sequence_zone_done_callback)
        
        # Don't wait for completion - return immediately so pose updates continue
        # The navigation will proceed in the background
        return {'ok': True, 'message': f'Navigation to zone "{name}" started'}
    
    def convert_map_to_png(self):
        """Convert PGM map to PNG for web display"""
        try:
            img_path = self.map_info.get('image_path')
            if not img_path or not os.path.exists(img_path):
                return None
            
            # Open PGM and convert to PNG
            img = Image.open(img_path)
            
            # Invert colors if negate is set
            if self.map_info.get('negate', 0) == 0:
                # Make occupied (0) black and free (255) white
                img = Image.eval(img, lambda x: 255 - x)
            
            # Convert to RGB
            img = img.convert('RGB')
            
            # Save to bytes
            img_io = io.BytesIO()
            img.save(img_io, 'PNG')
            img_io.seek(0)
            
            # Encode to base64
            img_base64 = base64.b64encode(img_io.getvalue()).decode('utf-8')
            
            return {
                'data': img_base64,
                'width': img.width,
                'height': img.height
            }
        except Exception as e:
            self.get_logger().error(f'Failed to convert map: {e}')
            return None


# Create Flask app
app = Flask(__name__, 
            template_folder='/home/aun/Downloads/smr300l_gazebo_ros2control-main/src/zone_nav/zone_nav/web/templates',
            static_folder='/home/aun/Downloads/smr300l_gazebo_ros2control-main/src/zone_nav/zone_nav/web/static')
CORS(app)

# Global node reference
ros_node = None


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/map')
def get_map():
    """Get map image and metadata"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    map_img = ros_node.convert_map_to_png()
    if map_img is None:
        return jsonify({'error': 'Failed to load map'}), 500
    
    return jsonify({
        'image': map_img['data'],
        'width': map_img['width'],
        'height': map_img['height'],
        'resolution': ros_node.map_info.get('resolution', 0.05),
        'origin': ros_node.map_info.get('origin', [0, 0, 0])
    })


@app.route('/api/zones')
def get_zones():
    """Get all saved zones"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    zones = ros_node.load_zones()
    return jsonify({'zones': zones})


@app.route('/api/robot/pose')
def get_robot_pose():
    """Get current robot position"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    return jsonify({
        'pose': ros_node.robot_pose,
        'confidence': ros_node.localization_confidence,
        'safety_stop': ros_node.safety_stop_active,
        'override_active': ros_node.safety_override_active
    })


@app.route('/api/path')
def get_path():
    """Get current navigation path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    return jsonify({'path': ros_node.current_path})


@app.route('/api/scan')
def get_scan():
    """Get current lidar scan points"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    return jsonify({'scan': ros_node.laser_scan})


@app.route('/api/zones/save', methods=['POST'])
def save_zone():
    """Save a new zone"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    x = data.get('x')
    y = data.get('y')
    theta = data.get('theta', 0.0)
    zone_type = data.get('type', 'normal')  # 'normal', 'no-go', 'speed'
    speed_limit = data.get('speed_limit', None)
    
    if not name or x is None or y is None:
        return jsonify({'error': 'Missing required fields'}), 400
    
    # Validate zone position for safety
    is_valid, warning = ros_node.validate_zone_position(x, y)
    
    if not is_valid:
        return jsonify({
            'ok': False,
            'message': warning,
            'warning_type': 'safety'
        })
    
    result = ros_node.save_zone(name, x, y, theta)
    
    # Store zone metadata if save was successful
    if result.get('ok') and (zone_type != 'normal' or speed_limit is not None):
        # TODO: Implement zone metadata storage
        pass
    
    return jsonify(result)


@app.route('/api/zones/goto', methods=['POST'])
def goto_zone():
    """Navigate to a zone"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    
    if not name:
        return jsonify({'error': 'Missing zone name'}), 400
    
    result = ros_node.go_to_zone(name)
    return jsonify(result)


@app.route('/api/zones/delete', methods=['POST'])
def delete_zone():
    """Delete a zone"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    
    if not name:
        return jsonify({'error': 'Missing zone name'}), 400
    
    result = ros_node.delete_zone(name)
    return jsonify(result)


@app.route('/api/zones/update', methods=['POST'])
def update_zone():
    """Update a zone's position"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    x = data.get('x')
    y = data.get('y')
    theta = data.get('theta', 0.0)
    
    if not name or x is None or y is None:
        return jsonify({'error': 'Name, x, and y are required'}), 400
    
    # Save updated zone position
    result = ros_node.save_zone(name, x, y, theta)
    return jsonify(result)


@app.route('/api/path/follow', methods=['POST'])
def follow_path():
    """Follow a drawn path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    path_points = data.get('path', [])
    
    if not path_points or len(path_points) < 2:
        return jsonify({'error': 'Path must have at least 2 points'}), 400
    
    result = ros_node.follow_drawn_path(path_points)
    return jsonify(result)


@app.route('/api/path/stop', methods=['POST'])
def stop_path():
    """Stop the current path following"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.stop_path_following()
    return jsonify(result)


@app.route('/api/path/status')
def path_status():
    """Get current path following status"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    return jsonify({
        'active': ros_node.path_following_active,
        'current_index': ros_node.current_waypoint_index,
        'total_waypoints': len(ros_node.path_waypoints),
        'current_waypoint': ros_node.path_waypoints[ros_node.current_waypoint_index] if ros_node.path_following_active and ros_node.current_waypoint_index < len(ros_node.path_waypoints) else None
    })


@app.route('/api/safety/force_resume', methods=['POST'])
def force_resume():
    """Force resume navigation by overriding safety stop"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.force_resume_navigation()
    return jsonify(result)


@app.route('/api/sequence/start', methods=['POST'])
def start_sequence():
    """Start sequential navigation through all zones"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    zone_names = data.get('zones', [])
    
    if not zone_names:
        return jsonify({'error': 'No zones provided'}), 400
    
    result = ros_node.start_sequence(zone_names)
    return jsonify(result)


@app.route('/api/sequence/stop', methods=['POST'])
def stop_sequence():
    """Stop the current sequence navigation"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.stop_sequence()
    return jsonify(result)


@app.route('/api/sequence/status')
def sequence_status():
    """Get current sequence status"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    status = ros_node.get_sequence_status()
    return jsonify(status)


@app.route('/api/manual/velocity', methods=['POST'])
def manual_velocity():
    """Send manual velocity command for joystick control"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    if ros_node.estop_active:
        return jsonify({'ok': False, 'message': 'E-STOP is active - manual control disabled'})
    
    data = request.json
    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)
    
    # Create and publish Twist message on high-priority joystick topic
    twist_msg = Twist()
    twist_msg.linear.x = float(linear)
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = float(angular)
    
    ros_node.cmd_vel_pub.publish(twist_msg)
    
    return jsonify({'ok': True, 'linear': linear, 'angular': angular})


@app.route('/api/estop/activate', methods=['POST'])
def estop_activate():
    """Activate emergency stop via safety controller"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        if not ros_node.safety_estop_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'error': 'Safety controller not available'}), 503
        
        request = SetBool.Request()
        request.data = True
        future = ros_node.safety_estop_client.call_async(request)
        rclpy.spin_until_future_complete(ros_node, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            ros_node.get_logger().warn('🛑 EMERGENCY STOP ACTIVATED via safety controller')
            
            # Cancel all active navigation
            if ros_node.path_following_active:
                ros_node.stop_path_following()
            if ros_node.sequence_active:
                ros_node.stop_sequence()
            
            return jsonify({'ok': True, 'message': response.message})
        else:
            return jsonify({'error': 'E-stop service call failed'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/estop/reset', methods=['POST'])
def estop_reset():
    """Reset emergency stop via safety controller"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        if not ros_node.safety_estop_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'error': 'Safety controller not available'}), 503
        
        request = SetBool.Request()
        request.data = False
        future = ros_node.safety_estop_client.call_async(request)
        rclpy.spin_until_future_complete(ros_node, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            ros_node.get_logger().info('✓ Emergency stop reset via safety controller')
            return jsonify({'ok': True, 'message': response.message})
        else:
            return jsonify({'error': 'E-stop reset service call failed'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/estop/status', methods=['GET'])
def estop_status():
    """Get safety controller status"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        if not ros_node.safety_status_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'error': 'Safety controller not available'}), 503
        
        request = Trigger.Request()
        future = ros_node.safety_status_client.call_async(request)
        rclpy.spin_until_future_complete(ros_node, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            return jsonify({'ok': True, 'status': response.message})
        else:
            return jsonify({'error': 'Status service call failed'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/mode/manual', methods=['POST'])
def set_manual_mode():
    """Switch to manual control mode"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    mode_msg = String()
    mode_msg.data = 'manual'
    ros_node.mode_pub.publish(mode_msg)
    ros_node.get_logger().info('🕹️  Switching to MANUAL mode')
    
    return jsonify({'ok': True, 'message': 'Switched to manual control mode'})


@app.route('/api/mode/auto', methods=['POST'])
def set_auto_mode():
    """Switch to autonomous navigation mode (zones)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    mode_msg = String()
    mode_msg.data = 'zones'  # Default autonomous mode
    ros_node.mode_pub.publish(mode_msg)
    ros_node.get_logger().info('🤖 Switching to AUTO mode (zones)')
    
    return jsonify({'ok': True, 'message': 'Switched to autonomous navigation mode'})


@app.route('/api/diagnostics/nav2', methods=['GET'])
def nav2_diagnostics():
    """Get Nav2 diagnostics - lidar status, costmap info, robot footprint"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    diagnostics = {
        'robot_footprint': "[[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]]",
        'robot_dimensions': {
            'length': '0.80m',
            'width': '0.50m (wheelbase: 0.54m)',
            'safety_radius': f'{ros_node.safety_radius}m'
        },
        'lidar_config': {
            'front_lidar': {
                'topic': '/scan',
                'frame': 'laser_frame',
                'position': 'x=0.75m, y=0, z=0.10m',
                'fov': '180° (-90° to +90°)',
                'range': '0.3m - 12m',
                'samples': 180,
                'active': ros_node.laser_scan is not None
            },
            'rear_lidar': {
                'topic': '/scan2',
                'frame': 'laser_frame2',
                'position': 'x=0.05m, y=0, z=0.10m (rotated 180°)',
                'fov': '180° (rear coverage)',
                'range': '0.3m - 12m',
                'samples': 180,
                'note': 'NOW ENABLED IN COSTMAPS'
            }
        },
        'costmap_config': {
            'local_costmap': {
                'size': '10m x 10m',
                'resolution': '0.05m',
                'footprint': 'RECTANGULAR (0.80m x 0.50m)',
                'inflation_radius': '0.55m',
                'lidars_used': '/scan and /scan2'
            },
            'global_costmap': {
                'size': '25m x 25m',
                'resolution': '0.05m',
                'footprint': 'RECTANGULAR (0.80m x 0.50m)',
                'inflation_radius': '0.6m',
                'lidars_used': '/scan and /scan2'
            }
        },
        'nav_parameters': {
            'max_velocity': '0.35 m/s',
            'max_rotation': '1.5 rad/s',
            'controller_freq': '15 Hz',
            'planner': 'NavFn with A*',
            'trajectory_samples': 'vx=25, vtheta=30',
            'sim_time': '2.0s lookahead'
        },
        'current_state': {
            'localization_confidence': f'{ros_node.localization_confidence:.1f}%',
            'safety_stop_active': ros_node.safety_stop_active,
            'estop_active': ros_node.estop_active,
            'path_following': ros_node.path_following_active,
            'has_lidar_data': ros_node.laser_scan is not None,
            'has_position': ros_node.robot_pose is not None
        }
    }
    
    return jsonify(diagnostics)


@app.route('/api/diagnostics/obstacles', methods=['GET'])
def check_obstacles():
    """Check for obstacles around robot"""
    if ros_node is None or ros_node.laser_scan is None or ros_node.robot_pose is None:
        return jsonify({'error': 'No lidar data or position available'}), 500
    
    scan = ros_node.laser_scan
    pose = ros_node.robot_pose
    
    # Count obstacles in different zones
    obstacle_zones = {
        'critical': 0,  # < 0.4m
        'warning': 0,   # 0.4m - 0.8m
        'safe': 0,      # 0.8m - 2.0m
        'clear': 0      # > 2.0m
    }
    
    angle = scan['angle_min']
    for r in scan['ranges']:
        if scan['range_min'] < r < scan['range_max']:
            if r < 0.4:
                obstacle_zones['critical'] += 1
            elif r < 0.8:
                obstacle_zones['warning'] += 1
            elif r < 2.0:
                obstacle_zones['safe'] += 1
            else:
                obstacle_zones['clear'] += 1
        angle += scan['angle_increment']
    
    total_valid = sum(obstacle_zones.values())
    
    return jsonify({
        'obstacle_count': obstacle_zones,
        'total_valid_readings': total_valid,
        'critical_zone_pct': f'{100.0 * obstacle_zones["critical"] / max(1, total_valid):.1f}%',
        'assessment': 'BLOCKED' if obstacle_zones['critical'] > 10 else 'CLEAR'
    })


def run_flask():
    """Run Flask server in a separate thread"""
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    ros_node = ZoneWebServer()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    ros_node.get_logger().info('Web UI available at: http://localhost:5000')
    
    # Use MultiThreadedExecutor to handle ROS callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
