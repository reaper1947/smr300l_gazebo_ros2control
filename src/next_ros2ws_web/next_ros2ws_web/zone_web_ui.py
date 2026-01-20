#!/usr/bin/env python3

import rclpy
from flask import Flask, render_template, jsonify, request, send_from_directory
from flask_cors import CORS
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from next_ros2ws_interfaces.srv import (
    SaveZone, DeleteZone, UpdateZoneParams, ReorderZones,
    SavePath, DeletePath, SaveLayout, LoadLayout, DeleteLayout,
    SetMaxSpeed, SetControlMode, SetSafetyOverride, SetEStop,
    UploadMap, SetActiveMap, GetActiveMap
)
from next_ros2ws_interfaces.action import GoToZone as GoToZoneAction, FollowPath as FollowPathAction
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TransformStamped, PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
import tf2_ros
from tf2_ros import TransformException
from slam_toolbox.srv import SaveMap as SlamSaveMap
import threading
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
import queue
import yaml
import os
import math
from PIL import Image, ImageDraw
import io
import base64
import json
import numpy as np
import signal
import subprocess
import time
import uuid

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False


class ZoneWebServer(Node):
    def __init__(self, executor):
        super().__init__('next_ros2ws_server')
        # Stack manager service client
        from next_ros2ws_interfaces.srv import SetStackMode
        self.set_stack_mode_client = self.create_client(SetStackMode, '/stack/set_mode')
        self.current_stack_mode = 'stopped'
        # Store executor reference for thread-safe ROS calls from Flask
        self.executor = executor

        # Live map (OccupancyGrid) subscription
        self.latest_map_msg = None
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos
        )
        # Queue for marshaling Flask→ROS calls onto the executor thread(s)
        # Use a steady clock so this timer still runs when use_sim_time is true
        # and /clock is not yet publishing.
        self._ros_call_queue = queue.Queue()
        self._ros_call_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self._process_ros_call_queue, clock=self._ros_call_clock)
        
        # TF2 buffer and listener for proper frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Reentrant callback group for thread safety with action client
        self.cb_group = ReentrantCallbackGroup()
        
        # Service clients
        self.save_zone_client = self.create_client(SaveZone, '/save_zone')
        self.delete_zone_client = self.create_client(DeleteZone, '/delete_zone')
        self.update_zone_params_client = self.create_client(UpdateZoneParams, '/update_zone_params')
        self.reorder_zones_client = self.create_client(ReorderZones, '/reorder_zones')
        self.save_path_client = self.create_client(SavePath, '/save_path')
        self.delete_path_client = self.create_client(DeletePath, '/delete_path')
        self.save_layout_client = self.create_client(SaveLayout, '/save_layout')
        self.load_layout_client = self.create_client(LoadLayout, '/load_layout')
        self.delete_layout_client = self.create_client(DeleteLayout, '/delete_layout')
        self.set_max_speed_client = self.create_client(SetMaxSpeed, '/set_max_speed')
        self.set_control_mode_client = self.create_client(SetControlMode, '/set_control_mode')
        self.set_safety_override_client = self.create_client(SetSafetyOverride, '/set_safety_override')
        self.set_estop_client = self.create_client(SetEStop, '/set_estop')
        # Use ReentrantCallbackGroup to avoid deadlock when called from _process_ros_call_queue (which uses default group)
        self.safety_status_client = self.create_client(Trigger, 'safety/status', callback_group=self.cb_group)
        self.safety_override_client = self.create_client(SetBool, 'safety/override', callback_group=self.cb_group)
        self.auto_reloc_client = self.create_client(Trigger, '/auto_relocate', callback_group=self.cb_group)

        # Map manager service clients
        self.upload_map_client = self.create_client(UploadMap, '/map_manager/upload_map')
        self.set_active_map_client = self.create_client(SetActiveMap, '/map_manager/set_active_map')
        self.get_active_map_client = self.create_client(GetActiveMap, '/map_manager/get_active_map')
        self.slam_save_map_client = self.create_client(SlamSaveMap, '/slam_toolbox/save_map')
        
        # Publisher for goal poses
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Publisher for initial pose estimate (2D Pose Estimate like RViz)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Publisher for manual velocity control (high priority via twist_mux)
        self.cmd_vel_manual_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        
        # Publisher for safety emergency stop (highest priority via twist_mux)
        self.cmd_vel_safety_pub = self.create_publisher(Twist, 'cmd_vel_safety', 10)
        
        # Publisher for control mode switching
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        
        # Track current control mode
        self.current_mode = 'manual'  # Start in manual mode
        self.mode_sub = self.create_subscription(String, '/control_mode', self.mode_status_callback, 10)
        
        # Action client for Nav2 navigation with ReentrantCallbackGroup for thread safety
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.cb_group
        )
        self.current_nav_goal_handle = None

        # Action clients for high-level navigation
        self.go_to_zone_action_client = ActionClient(
            self,
            GoToZoneAction,
            '/go_to_zone',
            callback_group=self.cb_group
        )

        self.follow_path_action_client = ActionClient(
            self,
            FollowPathAction,
            '/follow_path',
            callback_group=self.cb_group
        )
        
        # Timer for continuous safety stop commands
        self.safety_stop_timer = None
        
        # E-STOP timer (separate from safety stop)
        self.estop_timer = None
        
        # Safety parameters
        self.confidence_threshold = 10.0  # Stop if confidence drops below this
        self.confidence_resume_threshold = 11.0  # Resume when confidence goes above this (hysteresis)
        self.safety_stop_active = False
        self.safety_override_active = False  # Manual override flag
        self.last_confidence_warning = 0.0
        self.estop_active = False  # Emergency stop flag
        
        # TF lookup warning throttling
        self._last_tf_warn_front = 0.0
        self._last_tf_warn_rear = 0.0
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
        
        # Mission state tracking for resume functionality
        self.mission_state = {
            'active': False,
            'type': None,  # 'path', 'sequence', 'zone'
            'data': None,  # Mission-specific data
            'progress': 0,  # Current progress index
            'interrupted_by': None,  # 'estop', 'manual', 'safety'
            'timestamp': None
        }
        self.mission_state_file = os.path.expanduser('~/mission_state.json')
        
        # Subscribe to robot position - match AMCL's QoS profile
        self.robot_pose = None
        self.localization_confidence = 0.0
        self.last_pose_time = None
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
        
        # Timer to log if we're not receiving pose updates (debugging)
        self.pose_check_timer = self.create_timer(5.0, self._check_pose_reception)
        
        # Timer to check topic availability and subscription status
        self.topic_check_timer = self.create_timer(10.0, self._check_topic_availability)
        
        # Subscribe to navigation path
        self.current_path = None
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        self.get_logger().info('Subscribed to /plan for navigation path')
        
        # Subscribe to FRONT lidar scan
        self.laser_scan_front = None
        self.scan_front_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_front_callback,
            10
        )
        
        # Subscribe to REAR lidar scan
        self.laser_scan_rear = None
        self.scan_rear_sub = self.create_subscription(
            LaserScan,
            '/scan2',
            self.scan_rear_callback,
            10
        )
        
        # Combined scan for visualization
        self.laser_scan = None
        
        self.get_logger().info('Subscribed to /scan (FRONT) and /scan2 (REAR) for lidar data')
        
        # Load map info - get active map from map manager
        self.map_path = self._get_active_map_path_from_manager()
        self.zones_file = os.path.expanduser('~/zones.yaml')
        self.paths_file = os.path.expanduser('~/paths.yaml')
        self.layouts_file = os.path.expanduser('~/layouts.yaml')
        self.current_layout = None  # Track active layout
        self.map_info = self.load_map_info()
        
        # Load map image for editing
        self.map_img = None
        self._load_map_image()
        
        # Map editor layers (separate from navigation)
        self.map_layers = {
            'obstacles': [],
            'no_go_zones': [],
            'slow_zones': [],
            'restricted': []
        }
        self.declare_parameter('layers_file', '~/map_layers.json')
        layers_param = self.get_parameter('layers_file').get_parameter_value().string_value
        self.layers_file = os.path.expanduser(layers_param or '~/map_layers.json')
        self.load_map_layers()
        self._ensure_map_layers_defaults()
        
        self.get_logger().info('Zone Web Server node started')
        self.get_logger().info(f'Map: {self.map_info.get("image_path", "Not found")}')

    def _map_callback(self, msg):
        self.latest_map_msg = msg

    def _lookup_tf_pose(self):
        """Get robot pose from TF (map -> base_link/base_footprint)."""
        base_frames = ['base_link', 'base_footprint']
        for base_frame in base_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    base_frame,
                    Time(),
                    timeout=Duration(seconds=0.2)
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                # Yaw from quaternion
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return {'x': t.x, 'y': t.y, 'theta': yaw}
            except (TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        return None
    
    def call_ros_from_flask(self, ros_func, timeout=2.0):
        """Schedule ros_func on ROS executor via queue, wait in Flask thread."""
        fut = Future()
        self._ros_call_queue.put((ros_func, fut))
        return fut.result(timeout=timeout)

    def _process_ros_call_queue(self):
        """Run queued Flask→ROS calls on executor-managed thread(s)."""
        try:
            while True:
                func, fut = self._ros_call_queue.get_nowait()
                if fut.done():
                    continue
                try:
                    fut.set_result(func())
                except Exception as e:
                    try:
                        fut.set_exception(e)
                    except Exception:
                        pass
        except queue.Empty:
            return
    
    def mode_status_callback(self, msg):
        """Track current control mode"""
        self.current_mode = msg.data.lower()
        self.get_logger().debug(f'Mode updated: {self.current_mode}')
    
    def pose_callback(self, msg):
        """Store robot position from AMCL"""
        # Update last pose reception time
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        
        # AMCL publishes PoseWithCovarianceStamped - access nested pose.pose
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        }
        
        # Log first few callbacks to verify it's working
        if not hasattr(self, '_pose_callback_count'):
            self._pose_callback_count = 0
        self._pose_callback_count += 1
        if self._pose_callback_count <= 3:
            self.get_logger().info(f'✓ Pose callback #{self._pose_callback_count} - Robot at ({self.robot_pose["x"]:.2f}, {self.robot_pose["y"]:.2f})')
        
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
        # Tune k to avoid over-aggressive drops; typical AMCL variances ~0.01-0.2
        k = 1.0  # Previously 5.0; 1.0 provides smoother confidence behavior
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
    
    def _check_pose_reception(self):
        """Debug helper: Check if we're receiving pose updates"""
        if self.robot_pose is None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.last_pose_time is None or (current_time - self.last_pose_time) > 10.0:
                self.get_logger().warn('⚠️  No /amcl_pose messages received. Check: 1) AMCL is running, 2) QoS matches (RELIABLE + TRANSIENT_LOCAL), 3) Topic name is correct')
    
    def _check_topic_availability(self):
        """Check if topics exist and subscriptions are working"""
        try:
            topics = dict(self.get_topic_names_and_types())
            
            # Check pose topic
            has_amcl_pose = '/amcl_pose' in topics
            pose_received = self.robot_pose is not None
            if not has_amcl_pose:
                self.get_logger().warn('⚠️  /amcl_pose topic not found - AMCL may not be running')
            elif not pose_received:
                self.get_logger().warn('⚠️  /amcl_pose exists but no messages received - check QoS or topic publisher')
            else:
                self.get_logger().debug(f'✓ /amcl_pose: topic exists, pose received (x={self.robot_pose["x"]:.2f}, y={self.robot_pose["y"]:.2f})')
            
            # Check path topic
            has_plan = '/plan' in topics
            path_received = self.current_path is not None
            if not has_plan:
                self.get_logger().warn('⚠️  /plan topic not found - Nav2 planner may not be running')
            elif not path_received:
                self.get_logger().debug('✓ /plan: topic exists but no active path (normal when not navigating)')
            else:
                self.get_logger().debug(f'✓ /plan: topic exists, path received ({len(self.current_path)} waypoints)')
                
        except Exception as e:
            self.get_logger().error(f'Error checking topic availability: {e}')
    
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
            # Log first few callbacks to verify it's working
            if not hasattr(self, '_path_callback_count'):
                self._path_callback_count = 0
            self._path_callback_count += 1
            if self._path_callback_count <= 3:
                self.get_logger().info(f'✓ Path callback #{self._path_callback_count} - Received {len(self.current_path)} waypoints')
            else:
                self.get_logger().debug(f'Path updated with {len(self.current_path)} waypoints')
        else:
            self.current_path = None
            if not hasattr(self, '_path_callback_count'):
                self._path_callback_count = 0
            self._path_callback_count += 1
            if self._path_callback_count <= 3:
                self.get_logger().info(f'✓ Path callback #{self._path_callback_count} - Empty path (no navigation active)')
    
    def _scan_to_map_points(self, msg, sample_rate=3, min_valid_range=0.15):
        pts = []
        try:
            stamp = Time.from_msg(msg.header.stamp)
            angle = msg.angle_min
            for i, r in enumerate(msg.ranges):
                if i % sample_rate == 0 and r > min_valid_range and msg.range_min < r < msg.range_max:
                    ps = PointStamped()
                    ps.header.frame_id = msg.header.frame_id
                    ps.header.stamp = stamp.to_msg()
                    ps.point.x = float(r * math.cos(angle))
                    ps.point.y = float(r * math.sin(angle))
                    ps.point.z = 0.0

                    pmap = self.tf_buffer.transform(ps, "map", timeout=Duration(seconds=0.05))
                    pts.append({"x": pmap.point.x, "y": pmap.point.y})

                angle += msg.angle_increment
        except Exception as e:
            return None, e

        return pts, None

    def scan_front_callback(self, msg):
        pts, err = self._scan_to_map_points(msg)
        if err:
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self._last_tf_warn_front) > 5.0:
                self.get_logger().warn(f"TF transform failed FRONT ({msg.header.frame_id} -> map): {err}")
                self._last_tf_warn_front = now
            return

        self.laser_scan_front = pts
        self._merge_scans()

    def scan_rear_callback(self, msg):
        pts, err = self._scan_to_map_points(msg)
        if err:
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self._last_tf_warn_rear) > 5.0:
                self.get_logger().warn(f"TF transform failed REAR ({msg.header.frame_id} -> map): {err}")
                self._last_tf_warn_rear = now
            return

        self.laser_scan_rear = pts
        self._merge_scans()
    
    def _merge_scans(self):
        """Merge front and rear lidar scans for visualization"""
        merged = []
        if self.laser_scan_front:
            merged.extend(self.laser_scan_front)
        if self.laser_scan_rear:
            merged.extend(self.laser_scan_rear)
        self.laser_scan = merged if merged else None
    
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
        """Continuously publish stop commands while automatic safety stop is active"""
        if self.safety_stop_active:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_safety_pub.publish(stop_msg)
    
    def _publish_estop_command(self):
        """Continuously publish stop commands while E-STOP is active (MASTER KILL SWITCH)"""
        if self.estop_active:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_safety_pub.publish(stop_msg)
    
    def _emergency_stop(self):
        """Send zero velocity command and cancel Nav2 goals to stop the robot immediately"""
        # Send stop command to highest-priority safety topic (overrides everything)
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_safety_pub.publish(stop_msg)
        
        # Cancel any active Nav2 navigation goals
        if self.path_following_active:
            self.stop_path_following(save_state=True)
        if self.sequence_active:
            self.stop_sequence(save_state=True)
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
    
    def load_paths(self):
        """Load saved paths from YAML file"""
        if os.path.exists(self.paths_file):
            try:
                with open(self.paths_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                return data.get('paths', {})
            except Exception as e:
                self.get_logger().error(f'Failed to load paths: {e}')
                return {}
        return {}
    
    def save_path(self, name, path_points):
        """Save a path to YAML file"""
        try:
            # Load existing paths
            paths = self.load_paths()
            
            # Add/update the path
            paths[name] = path_points
            
            # Save to file
            with open(self.paths_file, 'w') as f:
                yaml.dump({'paths': paths}, f, default_flow_style=False)
            
            self.get_logger().info(f'Path "{name}" saved with {len(path_points)} waypoints')
            return {'ok': True, 'message': f'Path "{name}" saved successfully'}
        except Exception as e:
            self.get_logger().error(f'Failed to save path: {e}')
            return {'ok': False, 'message': f'Failed to save path: {str(e)}'}
    
    def delete_path(self, name):
        """Delete a saved path"""
        try:
            paths = self.load_paths()
            if name in paths:
                del paths[name]
                with open(self.paths_file, 'w') as f:
                    yaml.dump({'paths': paths}, f, default_flow_style=False)
                self.get_logger().info(f'Path "{name}" deleted')
                return {'ok': True, 'message': f'Path "{name}" deleted'}
            else:
                return {'ok': False, 'message': f'Path "{name}" not found'}
        except Exception as e:
            self.get_logger().error(f'Failed to delete path: {e}')
            return {'ok': False, 'message': f'Failed to delete path: {str(e)}'}
    
    def load_saved_path(self, name):
        """Load a specific saved path"""
        paths = self.load_paths()
        if name in paths:
            return {'ok': True, 'path': paths[name], 'message': f'Path "{name}" loaded'}
        else:
            return {'ok': False, 'message': f'Path "{name}" not found'}
    
    def load_layouts(self):
        """Load all saved layouts from YAML file"""
        if os.path.exists(self.layouts_file):
            try:
                with open(self.layouts_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                return data.get('layouts', {})
            except Exception as e:
                self.get_logger().error(f'Failed to load layouts: {e}')
                return {}
        return {}
    
    def save_layout(self, name, description=''):
        """Save current configuration as a layout"""
        try:
            # Load existing layouts
            layouts = self.load_layouts()
            
            # Get current zones and paths
            zones = self.load_zones()
            paths = self.load_paths()
            
            # Create layout entry
            layouts[name] = {
                'description': description,
                'map_path': self.map_path,
                'zones': zones,
                'paths': paths,
                'filters': self._filters_only(),
                'created': self.get_clock().now().to_msg().sec
            }
            
            # Save to file
            with open(self.layouts_file, 'w') as f:
                yaml.dump({'layouts': layouts}, f, default_flow_style=False)
            
            self.current_layout = name
            self.get_logger().info(f'Layout "{name}" saved with {len(zones)} zones and {len(paths)} paths')
            return {
                'ok': True, 
                'message': f'Layout "{name}" saved successfully',
                'zones_count': len(zones),
                'paths_count': len(paths)
            }
        except Exception as e:
            self.get_logger().error(f'Failed to save layout: {e}')
            return {'ok': False, 'message': f'Failed to save layout: {str(e)}'}
    
    def load_layout(self, name):
        """Load a layout and restore zones and paths"""
        try:
            layouts = self.load_layouts()
            if name not in layouts:
                return {'ok': False, 'message': f'Layout "{name}" not found'}
            
            layout = layouts[name]
            
            # Restore zones
            with open(self.zones_file, 'w') as f:
                yaml.dump({'zones': layout.get('zones', {})}, f, default_flow_style=False)
            
            # Restore paths
            with open(self.paths_file, 'w') as f:
                yaml.dump({'paths': layout.get('paths', {})}, f, default_flow_style=False)
            
            # Update map path if different
            if layout.get('map_path') and layout['map_path'] != self.map_path:
                self.map_path = layout['map_path']
                self.map_info = self.load_map_info()
                self._load_map_image()

            filters = layout.get('filters', {
                'no_go_zones': [],
                'restricted': [],
                'slow_zones': []
            })
            self._set_filters_only(filters)
            
            self.current_layout = name
            zones_count = len(layout.get('zones', {}))
            paths_count = len(layout.get('paths', {}))
            
            self.get_logger().info(f'Layout "{name}" loaded: {zones_count} zones, {paths_count} paths')
            return {
                'ok': True,
                'message': f'Layout "{name}" loaded successfully',
                'zones_count': zones_count,
                'paths_count': paths_count,
                'description': layout.get('description', '')
            }
        except Exception as e:
            self.get_logger().error(f'Failed to load layout: {e}')
            return {'ok': False, 'message': f'Failed to load layout: {str(e)}'}
    
    def delete_layout(self, name):
        """Delete a saved layout"""
        try:
            layouts = self.load_layouts()
            if name in layouts:
                del layouts[name]
                with open(self.layouts_file, 'w') as f:
                    yaml.dump({'layouts': layouts}, f, default_flow_style=False)
                if self.current_layout == name:
                    self.current_layout = None
                self.get_logger().info(f'Layout "{name}" deleted')
                return {'ok': True, 'message': f'Layout "{name}" deleted'}
            else:
                return {'ok': False, 'message': f'Layout "{name}" not found'}
        except Exception as e:
            self.get_logger().error(f'Failed to delete layout: {e}')
            return {'ok': False, 'message': f'Failed to delete layout: {str(e)}'}
    
    def get_layout_info(self):
        """Get information about current layout and available layouts"""
        layouts = self.load_layouts()
        return {
            'current': self.current_layout,
            'layouts': {name: {'description': data.get('description', ''), 
                               'zones_count': len(data.get('zones', {})),
                               'paths_count': len(data.get('paths', {})),
                               'created': data.get('created', 0)}
                        for name, data in layouts.items()}
        }
    
    def save_mission_state(self, mission_type, mission_data, progress, interrupted_by=None):
        """Save current mission state for potential resume"""
        try:
            self.mission_state = {
                'active': True,
                'type': mission_type,
                'data': mission_data,
                'progress': progress,
                'interrupted_by': interrupted_by,
                'timestamp': self.get_clock().now().to_msg().sec
            }
            
            with open(self.mission_state_file, 'w') as f:
                json.dump(self.mission_state, f, indent=2)
            
            self.get_logger().info(f'Mission state saved: {mission_type} at progress {progress}')
        except Exception as e:
            self.get_logger().error(f'Failed to save mission state: {e}')
    
    def load_mission_state(self):
        """Load saved mission state"""
        if os.path.exists(self.mission_state_file):
            try:
                with open(self.mission_state_file, 'r') as f:
                    self.mission_state = json.load(f)
                return self.mission_state
            except Exception as e:
                self.get_logger().error(f'Failed to load mission state: {e}')
                return {'active': False}
        return {'active': False}
    def clear_mission_state(self):
        """Clear saved mission state after completion or cancellation"""
        self.mission_state = {
            'active': False,
            'type': None,
            'data': None,
            'progress': 0,
            'interrupted_by': None,
            'timestamp': None
        }
        try:
            if os.path.exists(self.mission_state_file):
                os.remove(self.mission_state_file)
            self.get_logger().info('Mission state cleared')
        except Exception as e:
            self.get_logger().error(f'Failed to clear mission state: {e}')
    
    def resume_mission(self):
        """Resume interrupted mission from saved state"""
        state = self.load_mission_state()
        if not state.get('active'):
            return {'ok': False, 'message': 'No mission to resume'}
        
        mission_type = state.get('type')
        mission_data = state.get('data')
        progress = state.get('progress', 0)
        interrupted_by = state.get('interrupted_by', 'unknown')
        
        self.get_logger().info(f'Resuming {mission_type} mission from progress {progress} (interrupted by: {interrupted_by})')
        
        if mission_type == 'path':
            # Resume path following from remaining waypoints
            remaining_points = mission_data[progress:]
            if not remaining_points:
                return {'ok': False, 'message': 'No remaining waypoints to resume'}
            return self.follow_drawn_path(remaining_points)
        
        elif mission_type == 'sequence':
            # Resume sequence navigation
            self.sequence_zones = mission_data
            self.current_sequence_index = progress
            self.sequence_active = True
            self._send_sequence_zone_goal()
            return {'ok': True, 'message': f'Resuming sequence from zone {progress + 1}/{len(mission_data)}', 'type': 'sequence'}
        
        elif mission_type == 'zone':
            # Resume single zone navigation
            zone_name = mission_data.get('name')
            return self.go_to_zone(zone_name)
        
        else:
            return {'ok': False, 'message': f'Unknown mission type: {mission_type}'}
    
    def _load_map_image(self):
        """Load the map image file for editing"""
        try:
            img_path = self.map_info.get('image_path')
            if not img_path or not os.path.exists(img_path):
                self.get_logger().warn(f'Map image not found: {img_path}')
                return
            
            # Load the PGM/PNG image
            self.map_img = Image.open(img_path)
            
            # Invert only when the map is marked as negated.
            if self.map_info.get('negate', 0) == 1:
                self.map_img = Image.eval(self.map_img, lambda x: 255 - x)
            
            # Ensure it's in grayscale mode for editing
            self.map_img = self.map_img.convert('L')
            
            self.get_logger().info(f'Loaded map image: {self.map_img.width}x{self.map_img.height}')
        except Exception as e:
            self.get_logger().error(f'Failed to load map image: {e}')
            self.map_img = None
    
    def load_map_layers(self):
        """Load saved map editing layers"""
        if os.path.exists(self.layers_file):
            try:
                with open(self.layers_file, 'r') as f:
                    self.map_layers = json.load(f)
                self._ensure_map_layers_defaults()
                if self._ensure_filter_ids():
                    self.save_map_layers()
                self.get_logger().info(f'Loaded {sum(len(v) for v in self.map_layers.values())} map layer objects')
            except Exception as e:
                self.get_logger().error(f'Failed to load map layers: {e}')

    def _ensure_map_layers_defaults(self):
        if not isinstance(self.map_layers, dict):
            self.map_layers = {}
        for key in ('obstacles', 'no_go_zones', 'slow_zones', 'restricted'):
            if not isinstance(self.map_layers.get(key), list):
                self.map_layers[key] = []

    def _ensure_filter_ids(self):
        changed = False
        for key in ('no_go_zones', 'restricted', 'slow_zones'):
            for obj in self.map_layers.get(key, []):
                if isinstance(obj, dict) and 'id' not in obj:
                    obj['id'] = uuid.uuid4().hex
                    changed = True
        return changed
    
    def save_map_layers(self):
        """Save map editing layers to file"""
        try:
            with open(self.layers_file, 'w') as f:
                json.dump(self.map_layers, f, indent=2)
            self.get_logger().info('Map layers saved')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save map layers: {e}')
            return False

    def _filters_only(self):
        self._ensure_map_layers_defaults()
        return {
            'no_go_zones': list(self.map_layers.get('no_go_zones', [])),
            'restricted': list(self.map_layers.get('restricted', [])),
            'slow_zones': list(self.map_layers.get('slow_zones', []))
        }

    def _set_filters_only(self, filters):
        self._ensure_map_layers_defaults()
        self.map_layers['no_go_zones'] = filters.get('no_go_zones', [])
        self.map_layers['restricted'] = filters.get('restricted', [])
        self.map_layers['slow_zones'] = filters.get('slow_zones', [])
        self._ensure_filter_ids()
        self.save_map_layers()

    def add_filter_object(self, layer_type, obj):
        if layer_type not in ('no_go_zones', 'restricted', 'slow_zones'):
            return False, 'Invalid filter layer'
        if not isinstance(obj, dict):
            return False, 'Invalid object'
        self._ensure_map_layers_defaults()
        if 'id' not in obj:
            obj['id'] = uuid.uuid4().hex
        self.map_layers[layer_type].append(obj)
        self.save_map_layers()
        return True, obj['id']

    def delete_filter_object(self, layer_type, obj_id):
        if layer_type not in ('no_go_zones', 'restricted', 'slow_zones'):
            return False, 'Invalid filter layer'
        if not obj_id:
            return False, 'Missing id'
        self._ensure_map_layers_defaults()
        before = len(self.map_layers[layer_type])
        self.map_layers[layer_type] = [
            o for o in self.map_layers[layer_type]
            if o.get('id') != obj_id
        ]
        after = len(self.map_layers[layer_type])
        if after == before:
            return False, 'Not found'
        self.save_map_layers()
        return True, 'Deleted'

    def clear_filter_layer(self, layer_type):
        self._ensure_map_layers_defaults()
        if layer_type == 'all':
            self.map_layers['no_go_zones'] = []
            self.map_layers['restricted'] = []
            self.map_layers['slow_zones'] = []
        elif layer_type in ('no_go_zones', 'restricted', 'slow_zones'):
            self.map_layers[layer_type] = []
        else:
            return False, 'Invalid layer'
        self.save_map_layers()
        return True, 'Cleared'

    # ==== Map manager - Simple threadsafe wrapper ====
    def _get_active_map_path_from_manager(self):
        """Get active map path from map manager at startup"""
        # Wait for service
        if not self.get_active_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Map Manager not available at startup, using default map')
            return os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/smr_map.yaml')
        
        try:
            req = GetActiveMap.Request()
            result = self.call_map_service(self.get_active_map_client, req, timeout=3.0)
            
            if result and result.success and result.map_yaml_path:
                self.get_logger().info(f'Loaded active map from manager: {result.map_yaml_path}')
                return result.map_yaml_path
            else:
                self.get_logger().warn('Map Manager returned no active map, using default')
                return os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/smr_map.yaml')
        except Exception as e:
            self.get_logger().error(f'Failed to get active map: {e}, using default')
            return os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/smr_map.yaml')
    
    def call_map_service(self, client, request, timeout=5.0):
        """Generic service caller - thread-safe, works from any thread"""
        from concurrent.futures import Future as ConcurrentFuture, TimeoutError as ConcurrentTimeoutError
        import threading
        
        result_future = ConcurrentFuture()
        
        def done_callback(ros_future):
            try:
                result_future.set_result(ros_future.result())
            except Exception as e:
                result_future.set_exception(e)
        
        ros_future = client.call_async(request)
        ros_future.add_done_callback(done_callback)
        
        try:
            return result_future.result(timeout=timeout)
        except ConcurrentTimeoutError:
            return None
    
    def render_map_with_layers(self):
        """Render map with all editing layers overlaid"""
        if self.map_img is None:
            return None
        
        # Start with original map
        img = self.map_img.copy().convert('RGB')
        draw = ImageDraw.Draw(img, 'RGBA')
        
        resolution = self.map_info.get('resolution', 0.05)
        origin = self.map_info.get('origin', [0, 0, 0])
        height = img.height
        
        def world_to_pixel(wx, wy):
            px = int((wx - origin[0]) / resolution)
            py = int(height - (wy - origin[1]) / resolution)
            return (px, py)
        
        # Draw obstacles (black)
        for obj in self.map_layers.get('obstacles', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(0, 0, 0, 255))
            elif obj['type'] == 'circle':
                center = world_to_pixel(obj['x'], obj['y'])
                radius_px = int(obj['radius'] / resolution)
                bbox = [center[0] - radius_px, center[1] - radius_px,
                       center[0] + radius_px, center[1] + radius_px]
                draw.ellipse(bbox, fill=(0, 0, 0, 255))
            elif obj['type'] == 'polygon':
                points = [world_to_pixel(p['x'], p['y']) for p in obj['points']]
                draw.polygon(points, fill=(0, 0, 0, 255))
        
        # Draw no-go zones (red transparent)
        for obj in self.map_layers.get('no_go_zones', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(255, 0, 0, 100), outline=(255, 0, 0, 255), width=2)
            elif obj['type'] == 'polygon':
                points = [world_to_pixel(p['x'], p['y']) for p in obj['points']]
                draw.polygon(points, fill=(255, 0, 0, 100), outline=(255, 0, 0, 255))
        
        # Draw slow zones (yellow transparent)
        for obj in self.map_layers.get('slow_zones', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(255, 255, 0, 80), outline=(255, 200, 0, 255), width=2)
        
        # Draw restricted zones (orange transparent)
        for obj in self.map_layers.get('restricted', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(255, 165, 0, 100), outline=(255, 140, 0, 255), width=2)
        
        return img
    
    def export_edited_map(self, output_path):
        """Export edited map as new PGM file"""
        try:
            rendered = self.render_map_with_layers()
            if rendered is None:
                return False
            
            # Convert back to grayscale
            gray_map = rendered.convert('L')
            
            # Save as PGM
            gray_map.save(output_path)
            
            # Create corresponding YAML
            yaml_path = output_path.replace('.pgm', '.yaml')
            yaml_data = {
                'image': os.path.basename(output_path),
                'resolution': self.map_info.get('resolution', 0.05),
                'origin': self.map_info.get('origin', [0, 0, 0]),
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }
            
            with open(yaml_path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
            
            self.get_logger().info(f'Exported edited map to {output_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to export map: {e}')
            return False
    
    def save_to_current_map(self):
        """Save edited map directly to the current navigation map (replaces it)"""
        try:
            # Get current map path
            current_map_path = self.map_info.get('image_path')
            if not current_map_path:
                return False
            
            # Backup original
            backup_path = current_map_path.replace('.pgm', '_backup.pgm')
            if os.path.exists(current_map_path) and not os.path.exists(backup_path):
                import shutil
                shutil.copy(current_map_path, backup_path)
                self.get_logger().info(f'Created backup: {backup_path}')
            
            # Render map with ONLY obstacles baked in (not transparent zones)
            if self.map_img is None:
                return False
            
            img = self.map_img.copy()
            draw = ImageDraw.Draw(img)
            
            resolution = self.map_info.get('resolution', 0.05)
            origin = self.map_info.get('origin', [0, 0, 0])
            height = img.height
            
            def world_to_pixel(wx, wy):
                px = int((wx - origin[0]) / resolution)
                py = int(height - (wy - origin[1]) / resolution)
                return (px, py)
            
            # Draw ONLY obstacles as black (occupied space)
            for obj in self.map_layers.get('obstacles', []):
                if obj['type'] == 'rectangle':
                    p1 = world_to_pixel(obj['x1'], obj['y1'])
                    p2 = world_to_pixel(obj['x2'], obj['y2'])
                    draw.rectangle([p1, p2], fill=0)  # Black = occupied
                elif obj['type'] == 'circle':
                    center = world_to_pixel(obj['x'], obj['y'])
                    radius_px = int(obj['radius'] / resolution)
                    bbox = [center[0] - radius_px, center[1] - radius_px,
                           center[0] + radius_px, center[1] + radius_px]
                    draw.ellipse(bbox, fill=0)  # Black = occupied
                elif obj['type'] == 'polygon':
                    points = [world_to_pixel(p['x'], p['y']) for p in obj['points']]
                    draw.polygon(points, fill=0)  # Black = occupied
            
            # Save the map with obstacles
            img.save(current_map_path)
            
            # Export keepout zones for Nav2
            self._export_keepout_zones()
            
            # Export speed restricted zones for Nav2
            self._export_speed_zones()
            
            # Reload the map image
            self._load_map_image()
            
            self.get_logger().info(f'✓ Saved obstacles to map: {current_map_path}')
            self.get_logger().info(f'✓ Exported keepout zones (no-go + restricted)')
            self.get_logger().info(f'✓ Exported speed limit zones')
            self.get_logger().info('⚠️  Restart Nav2 to use the updated map and zones')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save to current map: {e}')
            return False
    
    def _export_keepout_zones(self):
        """Export no-go and restricted zones as Nav2 keepout filter masks"""
        try:
            map_dir = os.path.dirname(self.map_info.get('image_path', ''))
            if not map_dir:
                return
            
            # Create keepout zones YAML for Nav2
            keepout_zones = []
            
            # Add no-go zones
            for obj in self.map_layers.get('no_go_zones', []):
                if obj['type'] == 'rectangle':
                    zone = {
                        'type': 'rectangle',
                        'points': [
                            [obj['x1'], obj['y1']],
                            [obj['x2'], obj['y1']],
                            [obj['x2'], obj['y2']],
                            [obj['x1'], obj['y2']]
                        ]
                    }
                    keepout_zones.append(zone)
                elif obj['type'] == 'polygon':
                    zone = {
                        'type': 'polygon',
                        'points': [[p['x'], p['y']] for p in obj['points']]
                    }
                    keepout_zones.append(zone)
            
            # Add restricted zones
            for obj in self.map_layers.get('restricted', []):
                if obj['type'] == 'rectangle':
                    zone = {
                        'type': 'rectangle',
                        'points': [
                            [obj['x1'], obj['y1']],
                            [obj['x2'], obj['y1']],
                            [obj['x2'], obj['y2']],
                            [obj['x1'], obj['y2']]
                        ]
                    }
                    keepout_zones.append(zone)
            
            # Save to YAML
            keepout_file = os.path.join(map_dir, 'keepout_zones.yaml')
            with open(keepout_file, 'w') as f:
                yaml.dump({'keepout_zones': keepout_zones}, f, default_flow_style=False)
            
            self.get_logger().info(f'Exported {len(keepout_zones)} keepout zones to {keepout_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to export keepout zones: {e}')
    
    def _export_speed_zones(self):
        """Export slow zones as Nav2 speed restricted zones"""
        try:
            map_dir = os.path.dirname(self.map_info.get('image_path', ''))
            if not map_dir:
                return
            
            speed_zones = []
            
            for obj in self.map_layers.get('slow_zones', []):
                if obj['type'] == 'rectangle':
                    zone = {
                        'type': 'rectangle',
                        'points': [
                            [obj['x1'], obj['y1']],
                            [obj['x2'], obj['y1']],
                            [obj['x2'], obj['y2']],
                            [obj['x1'], obj['y2']]
                        ],
                        'speed_limit': obj.get('speed_limit', 0.2)
                    }
                    speed_zones.append(zone)
            
            # Save to YAML
            speed_file = os.path.join(map_dir, 'speed_zones.yaml')
            with open(speed_file, 'w') as f:
                yaml.dump({'speed_zones': speed_zones}, f, default_flow_style=False)
            
            self.get_logger().info(f'Exported {len(speed_zones)} speed zones to {speed_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to export speed zones: {e}')
    
    def delete_zone(self, name):
        """Call DeleteZone service"""
        if not self.delete_zone_client.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'message': 'DeleteZone service not available'}
        
        request = DeleteZone.Request()
        request.name = name
        try:
            result = self.call_map_service(self.delete_zone_client, request, timeout=2.0)
            if result is None:
                return {'ok': False, 'message': 'DeleteZone timed out'}
            return {'ok': bool(result.ok), 'message': str(result.message)}
        except Exception as e:
            return {'ok': False, 'message': f'Service call failed: {str(e)}'}


    def update_zone_params_service(self, name, zone_type='normal', speed=0.5, action='', charge_duration=0):
        """Call UpdateZoneParams service"""
        if not self.update_zone_params_client.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'message': 'UpdateZoneParams service not available'}
        
        request = UpdateZoneParams.Request()
        request.name = name
        request.type = zone_type
        request.speed = float(speed)
        request.action = action or ''
        request.charge_duration = int(charge_duration) if charge_duration is not None else 0
        try:
            result = self.call_map_service(self.update_zone_params_client, request, timeout=2.0)
            if result is None:
                return {'ok': False, 'message': 'UpdateZoneParams timed out'}
            return {'ok': bool(result.ok), 'message': str(result.message)}
        except Exception as e:
            return {'ok': False, 'message': f'Service call failed: {str(e)}'}
    
    def reorder_zones_service(self, zone_names):
        """Call ReorderZones service"""
        if not self.reorder_zones_client.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'ReorderZones service not available'}
        
        request = ReorderZones.Request()
        request.zone_names = zone_names
        
        try:
            result = self.call_map_service(self.reorder_zones_client, request, timeout=3.0)
            if result is None:
                return {'ok': False, 'message': 'ReorderZones timed out'}
            return {'ok': bool(result.ok), 'message': str(result.message)}
        except Exception as e:
            return {'ok': False, 'message': f'Service call failed: {str(e)}'}
    
    def reorder_zones(self, reordered_zones):
        """Reorder zones by saving them in the new order"""
        try:
            # Save zones with the new order
            with open(self.zones_file, 'w') as f:
                yaml.dump({'zones': reordered_zones}, f, default_flow_style=False)
            
            self.get_logger().info(f'Zones reordered successfully')
            return {'ok': True, 'message': 'Zones reordered successfully'}
            
        except Exception as e:
            self.get_logger().error(f'Failed to reorder zones: {e}')
            return {'ok': False, 'message': f'Error reordering zones: {str(e)}'}
    
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
        """Follow a drawn path using the FollowPath action server"""
        # Block if e-stop is latched
        if self.estop_active:
            return {'ok': False, 'message': '🛑 E-STOP ACTIVE. Reset before following a path.'}

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
        self.current_mode = 'path'
        self.get_logger().info('🛤️  Switching to PATH mode')
        
        # Check safety
        if self.safety_stop_active and not self.safety_override_active:
            return {'ok': False, 'message': f'Cannot navigate: Localization confidence too low ({self.localization_confidence:.1f}%). Improve robot localization first or use "Force Resume".'}

        # Wait for action server
        if not self.follow_path_action_client.wait_for_server(timeout_sec=5.0):
            return {'ok': False, 'message': 'FollowPath action server not available'}
        
        # Build PoseStamped waypoints with simple heading toward next point
        import math
        waypoints = []
        for idx, point in enumerate(path_points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point['x'])
            pose.pose.position.y = float(point['y'])
            pose.pose.position.z = 0.0

            if idx < len(path_points) - 1:
                next_point = path_points[idx + 1]
                dx = next_point['x'] - point['x']
                dy = next_point['y'] - point['y']
                theta = math.atan2(dy, dx)
            else:
                theta = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            waypoints.append(pose)
        
        self.path_following_active = True
        self.path_waypoints = path_points
        self.current_waypoint_index = 0
        self.save_mission_state('path', path_points, 0, None)
        
        goal_msg = FollowPathAction.Goal()
        goal_msg.waypoints = waypoints

        self.get_logger().info(f'🚀 Sending FollowPath action with {len(waypoints)} waypoints')
        send_goal_future = self.follow_path_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._follow_path_feedback_callback
        )
        send_goal_future.add_done_callback(self._follow_path_goal_response_callback)
        
        return {'ok': True, 'message': f'Path following started: {len(path_points)} waypoints'}
    
    def _follow_path_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'FollowPath goal dispatch failed: {e}')
            self.path_following_active = False
            return

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected by server')
            self.path_following_active = False
            return

        self.path_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_path_result_callback)

    def _follow_path_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_waypoint_index = feedback.current_index
        status_note = feedback.status if feedback.status else 'Navigating'
        self.get_logger().info(f'Path feedback: {status_note} ({feedback.current_index + 1}/{feedback.total})')
        # Persist progress so resume works
        if self.path_following_active:
            self.save_mission_state('path', self.path_waypoints, feedback.current_index, None)

    def _follow_path_result_callback(self, future):
        self.path_goal_handle = None
        try:
            result = future.result()
            action_result = result.result
            status = result.status
        except Exception as e:
            self.get_logger().error(f'FollowPath action failed: {e}')
            self.path_following_active = False
            return

        if action_result.success:
            self.get_logger().info(f'🎉 Path complete! {action_result.completed} waypoints reached')
            self.clear_mission_state()
        else:
            self.get_logger().error(f'FollowPath failed (status={status}): {action_result.message}')

        self.path_following_active = False
        self.path_waypoints = []
        self.current_waypoint_index = 0

    def stop_path_following(self, save_state=False):
        """Stop the current path following"""
        if not self.path_following_active:
            return {'ok': False, 'message': 'No path following is active'}
        
        # Save state if interrupted (not completed)
        if save_state and self.current_waypoint_index < len(self.path_waypoints):
            self.save_mission_state('path', self.path_waypoints, self.current_waypoint_index, 'manual')
        
        self.path_following_active = False
        
        if self.path_goal_handle is not None:
            try:
                self.path_goal_handle.cancel_goal_async()
                self.path_goal_handle = None
            except Exception as e:
                self.get_logger().error(f'Failed to cancel FollowPath goal: {e}')
        
        self.path_waypoints = []
        self.current_waypoint_index = 0
        
        self.get_logger().info('Path following stopped')
        return {'ok': True, 'message': 'Path following stopped'}
    
    def save_zone(self, name, x, y, theta, zone_type='normal', speed=0.5, action=None, charge_duration=None):
        """Call save_zone service and store zone metadata"""
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
        
        # Store zone metadata in zones.yaml
        try:
            zones = self.load_zones()
            if name not in zones:
                # Wait briefly for the service to save the zone
                time.sleep(0.2)
                zones = self.load_zones()
            
            if name in zones:
                # Add metadata to existing zone
                zones[name]['type'] = zone_type
                zones[name]['speed'] = speed
                
                if zone_type == 'action' and action:
                    zones[name]['action'] = action
                elif zone_type == 'charge' and charge_duration is not None:
                    zones[name]['charge_duration'] = charge_duration
                
                # Save back to file
                with open(self.zones_file, 'w') as f:
                    yaml.dump({'zones': zones}, f, default_flow_style=False)
                    
                self.get_logger().info(f'Zone "{name}" metadata saved: type={zone_type}, speed={speed}')
        except Exception as e:
            self.get_logger().error(f'Failed to save zone metadata: {e}')
        
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
    
    def update_zone_params_local(self, name, zone_type='normal', speed=0.5, action=None, charge_duration=None):
        """Update zone parameters (type, speed, action, charge_duration)"""
        try:
            zones = self.load_zones()
            
            if name not in zones:
                return {'ok': False, 'message': f'Zone "{name}" not found'}
            
            # Update metadata
            zones[name]['type'] = zone_type
            zones[name]['speed'] = speed
            
            # Remove old parameters
            zones[name].pop('action', None)
            zones[name].pop('charge_duration', None)
            
            # Add type-specific parameters
            if zone_type == 'action' and action:
                zones[name]['action'] = action
            elif zone_type == 'charge' and charge_duration is not None:
                zones[name]['charge_duration'] = charge_duration
            
            # Save back to file
            with open(self.zones_file, 'w') as f:
                yaml.dump({'zones': zones}, f, default_flow_style=False)
            
            self.get_logger().info(f'Zone "{name}" parameters updated: type={zone_type}, speed={speed}')
            return {'ok': True, 'message': f'Zone "{name}" parameters updated'}
            
        except Exception as e:
            self.get_logger().error(f'Failed to update zone params: {e}')
            return {'ok': False, 'message': f'Error updating zone: {str(e)}'}
    
    def start_sequence(self, zone_names):
        """Start navigating through zones in sequence"""
        if self.estop_active:
            return {'ok': False, 'message': '🛑 E-STOP ACTIVE. Reset before starting sequence.'}

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
        self.current_mode = 'sequence'
        self.get_logger().info('📝 Switching to SEQUENCE mode')
        
        # Validate zones exist
        all_zones = self.load_zones()
        for zone_name in zone_names:
            if zone_name not in all_zones:
                return {'ok': False, 'message': f'Zone "{zone_name}" not found'}
        
        self.sequence_active = True
        self.sequence_zones = zone_names
        self.current_sequence_index = 0
        
        self.get_logger().info(f'Starting sequence: {len(zone_names)} zones')
        
        # Navigate to first zone
        self._send_sequence_zone_goal()
        
        return {'ok': True, 'message': f'Sequence started: {len(zone_names)} zones'}
    
    def _send_sequence_zone_goal(self):
        """Send goal for current zone"""
        if not self.sequence_active or self.current_sequence_index >= len(self.sequence_zones):
            return
        
        zone_name = self.sequence_zones[self.current_sequence_index]
        all_zones = self.load_zones()
        
        if zone_name not in all_zones:
            self.get_logger().error(f'Zone "{zone_name}" not found')
            self.stop_sequence()
            return
        
        zone = all_zones[zone_name]
        self.get_logger().info(f'Sending goal for zone {self.current_sequence_index + 1}/{len(self.sequence_zones)}: "{zone_name}"')
        
        # Wait for Nav2 action server
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
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
        
        # Send goal with feedback
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=None
        )
        send_goal_future.add_done_callback(self._sequence_goal_response_callback)
    
    def stop_sequence(self, save_state=False):
        """Stop the current sequence navigation"""
        if not self.sequence_active:
            return {'ok': False, 'message': 'No sequence is running'}
        
        # Save state if interrupted (not completed)
        if save_state and self.current_sequence_index < len(self.sequence_zones):
            self.save_mission_state('sequence', self.sequence_zones, self.current_sequence_index, 'manual')
        
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
        """Handle completion of a zone"""
        if not self.sequence_active:
            return
        
        try:
            result = future.result()
            
            if result.status == 4:  # SUCCEEDED
                zone_name = self.sequence_zones[self.current_sequence_index]
                self.get_logger().info(f'✓ Reached zone {self.current_sequence_index + 1}/{len(self.sequence_zones)}: "{zone_name}"')
                self.current_sequence_index += 1
                
                if self.current_sequence_index >= len(self.sequence_zones):
                    self.get_logger().info('✓ Sequence complete!')
                    self.clear_mission_state()  # Clear saved state on successful completion
                    self.sequence_active = False
                    self.sequence_zones = []
                    self.current_sequence_index = 0
                else:
                    # Send next zone immediately
                    self._send_sequence_zone_goal()
            else:
                self.get_logger().error(f'Zone failed')
                self.stop_sequence()
        except Exception as e:
            self.get_logger().error(f'Sequence error: {e}')
            self.stop_sequence()
    
    def go_to_zone(self, name, is_sequence=False):
        """Send a GoToZone action goal"""
        # Block if e-stop is latched
        if self.estop_active:
            return {'ok': False, 'message': '🛑 E-STOP ACTIVE. Reset before navigating.'}

        # Check if in manual mode - block autonomous navigation
        if self.current_mode == 'manual' and not is_sequence:
            return {'ok': False, 'message': '⚠️  Cannot navigate: Robot is in MANUAL MODE. Deactivate manual mode first.'}

        # Wait for action server
        ready = self.go_to_zone_action_client.wait_for_server(timeout_sec=5.0)
        if not ready:
            return {'ok': False, 'message': 'GoToZone action server not available'}
        
        # Switch to zones mode (unless this is part of a sequence)
        if not is_sequence:
            mode_msg = String()
            mode_msg.data = 'zones'
            self.mode_pub.publish(mode_msg)
            self.current_mode = 'zones'
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

        goal_msg = GoToZoneAction.Goal()
        goal_msg.name = name

        # Save mission state for resume (only for standalone zone nav)
        if not is_sequence:
            self.save_mission_state('zone', {'name': name}, 0, None)

        send_goal_future = self.go_to_zone_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._go_to_zone_feedback_callback
        )
        send_goal_future.add_done_callback(lambda fut: self._go_to_zone_goal_response_callback(fut, name))

        return {'ok': True, 'message': f'Navigation to zone "{name}" started'}

    def _go_to_zone_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        progress_pct = int(feedback.progress * 100)
        status_note = feedback.status if feedback.status else 'Navigating'
        self.get_logger().info(f'GoToZone feedback: {status_note} ({progress_pct}%)')

    def _go_to_zone_goal_response_callback(self, future, name):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to send GoToZone goal: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('GoToZone goal rejected by server')
            return

        self.current_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._go_to_zone_result_callback(f, name))

    def _go_to_zone_result_callback(self, future, name):
        self.current_nav_goal_handle = None
        try:
            result = future.result()
            action_result = result.result
        except Exception as e:
            self.get_logger().error(f'GoToZone result error: {e}')
            return

        if action_result.success:
            self.get_logger().info(f'✓ Reached zone "{name}": {action_result.message}')
            self.clear_mission_state()
        else:
            self.get_logger().error(f'GoToZone failed: {action_result.message}')
    
    def convert_map_to_png(self):
        """Convert PGM map to PNG for web display"""
        try:
            img_path = self.map_info.get('image_path')
            if not img_path or not os.path.exists(img_path):
                return None
            
            # Open PGM and convert to PNG
            img = Image.open(img_path)
            
            # Invert colors only when the map is marked as negated.
            if self.map_info.get('negate', 0) == 1:
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
            template_folder='/home/aun/Downloads/smr300l_gazebo_ros2control-main/src/next_ros2ws_web/next_ros2ws_web/web/templates',
            static_folder='/home/aun/Downloads/smr300l_gazebo_ros2control-main/src/next_ros2ws_web/next_ros2ws_web/web/static')
CORS(app)

# Global node reference
ros_node = None


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/editor')
def editor():
    return render_template('map_editor.html')


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
    
    # Add console logging for debugging
    ros_node.get_logger().debug(f'Returning zones: {zones}')
    
    return jsonify({'zones': zones})


@app.route('/api/robot/pose')
def get_robot_pose():
    """Get current robot position"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Debug: Log if pose is None (throttled)
    pose = ros_node.robot_pose
    pose_source = 'amcl' if pose is not None else None

    if pose is None:
        if not hasattr(ros_node, '_last_pose_api_warn') or (time.time() - ros_node._last_pose_api_warn) > 10.0:
            ros_node.get_logger().warn('API: robot_pose is None - AMCL may not be publishing yet. Check: 1) AMCL is running, 2) /amcl_pose topic exists, 3) QoS matches')
            ros_node._last_pose_api_warn = time.time()
        try:
            pose = ros_node.call_ros_from_flask(ros_node._lookup_tf_pose, timeout=0.6)
            if pose:
                pose_source = 'tf'
        except Exception as e:
            ros_node.get_logger().debug(f'API: TF pose lookup failed: {e}')
    
    return jsonify({
        'pose': pose,
        'confidence': ros_node.localization_confidence,
        'safety_stop': ros_node.safety_stop_active,
        'override_active': ros_node.safety_override_active,
        'has_pose': pose is not None,  # Help frontend know if pose is available
        'pose_source': pose_source
    })


@app.route('/api/path')
def get_path():
    """Get current navigation path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Debug: Log path status (throttled)
    if ros_node.current_path is None:
        if not hasattr(ros_node, '_last_path_api_warn') or (time.time() - ros_node._last_path_api_warn) > 10.0:
            ros_node.get_logger().debug('API: current_path is None - No active navigation path (normal when not navigating)')
            ros_node._last_path_api_warn = time.time()
    else:
        if not hasattr(ros_node, '_last_path_api_log') or (time.time() - ros_node._last_path_api_log) > 5.0:
            ros_node.get_logger().debug(f'API: Returning path with {len(ros_node.current_path)} waypoints')
            ros_node._last_path_api_log = time.time()
    
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
    zone_type = data.get('type', 'normal')  # 'normal', 'action', 'charge'
    speed = data.get('speed', 0.5)
    action = data.get('action', None)
    charge_duration = data.get('charge_duration', None)
    
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
    
    # Direct call - save_zone uses async service call pattern (non-blocking)
    result = ros_node.save_zone(name, x, y, theta, zone_type, speed, action, charge_duration)
    
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
    
    # Call GoToZone service
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
    
    # Call DeleteZone service
    result = ros_node.delete_zone(name)
    return jsonify(result)


@app.route('/api/zones/reorder', methods=['POST'])
def reorder_zones():
    """Reorder zones"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    zones_dict = data.get('zones')
    
    if not zones_dict:
        return jsonify({'error': 'Missing zones data'}), 400
    
    # Extract zone names in order
    zone_names = list(zones_dict.keys())
    
    # Call ReorderZones service
    result = ros_node.reorder_zones_service(zone_names)
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
    # Direct call - save_zone uses async service call pattern (non-blocking)
    result = ros_node.save_zone(name, x, y, theta)
    return jsonify(result)


@app.route('/api/zones/update-params', methods=['POST'])
def update_zone_params():
    """Update zone parameters (type, speed, action, charge_duration)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    zone_type = data.get('type', 'normal')
    speed = data.get('speed', 0.5)
    action = data.get('action', '')
    charge_duration = data.get('charge_duration', 0.0)
    
    if not name:
        return jsonify({'error': 'Zone name is required'}), 400
    
    # Call UpdateZoneParams service
    result = ros_node.update_zone_params_service(name, zone_type, speed, action, charge_duration)
    if result.get('ok'):
        ros_node.update_zone_params_local(name, zone_type, speed, action, charge_duration)
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
    
    # Direct call is safe - method is async and non-blocking
    result = ros_node.follow_drawn_path(path_points)
    return jsonify(result)


@app.route('/api/path/stop', methods=['POST'])
def stop_path():
    """Stop the current path following"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Direct call is safe - method is async and non-blocking
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


@app.route('/api/path/save', methods=['POST'])
def save_path():
    """Save a drawn path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    path_points = data.get('path', [])
    
    if not name:
        return jsonify({'error': 'Path name is required'}), 400
    
    if not path_points or len(path_points) < 2:
        return jsonify({'error': 'Path must have at least 2 points'}), 400
    
    result = ros_node.save_path(name, path_points)
    return jsonify(result)


@app.route('/api/paths')
def get_paths():
    """Get all saved paths"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    paths = ros_node.load_paths()
    return jsonify({'paths': paths})


@app.route('/api/path/load/<path_name>')
def load_path(path_name):
    """Load a specific saved path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.load_saved_path(path_name)
    return jsonify(result)


@app.route('/api/path/delete/<path_name>', methods=['DELETE'])
def delete_path(path_name):
    """Delete a saved path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.delete_path(path_name)
    return jsonify(result)


@app.route('/api/mission/status')
def get_mission_status():
    """Get current mission state"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    state = ros_node.load_mission_state()
    return jsonify(state)


@app.route('/api/mission/resume', methods=['POST'])
def resume_mission():
    """Resume interrupted mission"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.resume_mission()
    return jsonify(result)


@app.route('/api/mission/clear', methods=['POST'])
def clear_mission():
    """Clear saved mission state"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    ros_node.clear_mission_state()
    return jsonify({'ok': True, 'message': 'Mission state cleared'})


@app.route('/api/layouts')
def get_layouts():
    """Get all saved layouts with metadata"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    info = ros_node.get_layout_info()
    return jsonify(info)


@app.route('/api/layout/save', methods=['POST'])
def save_layout():
    """Save current configuration as a layout"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    name = data.get('name')
    description = data.get('description', '')
    
    if not name:
        return jsonify({'error': 'Layout name is required'}), 400
    
    result = ros_node.save_layout(name, description)
    return jsonify(result)


@app.route('/api/layout/load/<layout_name>', methods=['POST'])
def load_layout(layout_name):
    """Load a saved layout"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.load_layout(layout_name)
    return jsonify(result)


@app.route('/api/layout/delete/<layout_name>', methods=['DELETE'])
def delete_layout(layout_name):
    """Delete a saved layout"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.delete_layout(layout_name)
    return jsonify(result)


@app.route('/api/safety/force_resume', methods=['POST'])
def force_resume():
    """Force resume navigation by overriding safety stop"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Direct call is safe - simple state modification
    result = ros_node.force_resume_navigation()

    override_result = None

    def call_override():
        if not ros_node.safety_override_client.wait_for_service(timeout_sec=0.5):
            return {'ok': False, 'message': 'Safety override service not available'}
        req = SetBool.Request()
        req.data = True
        future = ros_node.safety_override_client.call_async(req)
        done_event = threading.Event()

        def _on_done(_):
            done_event.set()

        future.add_done_callback(_on_done)
        if not done_event.wait(timeout=1.0):
            return {'ok': False, 'message': 'Timeout waiting for safety override'}
        res = future.result()
        return {'ok': bool(res.success), 'message': str(res.message)}

    try:
        override_result = ros_node.call_ros_from_flask(call_override, timeout=3.0)
    except Exception:
        override_result = None

    ok = bool(result.get('ok')) or (override_result and override_result.get('ok'))
    message_parts = [result.get('message', '')] if result else []
    if override_result:
        message_parts.append(override_result.get('message', ''))
    message = ' '.join(part for part in message_parts if part)

    return jsonify({'ok': ok, 'message': message or 'Safety override requested'})


@app.route('/api/safety/override', methods=['POST'])
def set_safety_override():
    """Enable/disable safety override in SafetyController"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500

    data = request.get_json(force=True) or {}
    enable = bool(data.get('enable', True))

    def call_override():
        if not ros_node.safety_override_client.wait_for_service(timeout_sec=0.5):
            return {'ok': False, 'message': 'Safety override service not available'}
        req = SetBool.Request()
        req.data = enable
        future = ros_node.safety_override_client.call_async(req)
        done_event = threading.Event()

        def _on_done(_):
            done_event.set()

        future.add_done_callback(_on_done)
        if not done_event.wait(timeout=1.0):
            return {'ok': False, 'message': 'Timeout waiting for safety override'}
        res = future.result()
        return {'ok': bool(res.success), 'message': str(res.message)}

    try:
        result = ros_node.call_ros_from_flask(call_override, timeout=3.0)
        return jsonify(result), 200 if result.get('ok') else 500
    except Exception as e:
        return jsonify({'ok': False, 'message': str(e)}), 500


@app.route('/api/safety/status', methods=['GET'])
def get_safety_status():
    """Get SafetyController status (low confidence/obstacle stops)."""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500

    def call_status():
        if not ros_node.safety_status_client.wait_for_service(timeout_sec=0.5):
            return {'ok': False, 'message': 'Safety status service not available'}
        req = Trigger.Request()
        future = ros_node.safety_status_client.call_async(req)
        done_event = threading.Event()

        def _on_done(_):
            done_event.set()

        future.add_done_callback(_on_done)
        if not done_event.wait(timeout=1.0):
            return {'ok': False, 'message': 'Timeout waiting for safety status'}
        res = future.result()
        return {'ok': bool(res.success), 'message': str(res.message)}

    try:
        result = ros_node.call_ros_from_flask(call_status, timeout=3.0)
    except Exception as e:
        return jsonify({'ok': False, 'message': str(e)}), 500

    if not result.get('ok'):
        return jsonify(result), 500

    message = result.get('message', '')
    msg_lower = message.lower()
    low_conf = 'low confidence stop: active' in msg_lower
    obstacle = 'obstacle stop: active' in msg_lower
    manual_override = 'manual override: enabled' in msg_lower
    estop = 'e-stop: active' in msg_lower
    active = low_conf or obstacle or estop

    return jsonify({
        'ok': True,
        'active': active,
        'low_confidence_stop': low_conf,
        'obstacle_stop': obstacle,
        'manual_override': manual_override,
        'estop_active': estop,
        'message': message
    })


@app.route('/api/sequence/start', methods=['POST'])
def start_sequence():
    """Start sequential navigation through all zones"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    zone_names = data.get('zones', [])
    
    if not zone_names:
        return jsonify({'error': 'No zones provided'}), 400
    
    # Direct call is safe - method is async and non-blocking
    result = ros_node.start_sequence(zone_names)
    return jsonify(result)


@app.route('/api/sequence/stop', methods=['POST'])
def stop_sequence():
    """Stop the current sequence navigation"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Direct call is safe - method is async and non-blocking
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
    
    # Direct publish is safe - ROS publishers are thread-safe for publish() calls
    twist_msg = Twist()
    twist_msg.linear.x = float(linear)
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = float(angular)
    ros_node.cmd_vel_manual_pub.publish(twist_msg)
    
    return jsonify({'ok': True, 'linear': linear, 'angular': angular})


@app.route('/api/estop/activate', methods=['POST'])
def estop_activate():
    """Activate E-STOP - MASTER KILL SWITCH (immediately halt all robot motion)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        ros_node.get_logger().warn('🛑 E-STOP ACTIVATED - MASTER KILL SWITCH')
        
        # Save mission state before stopping
        if ros_node.path_following_active:
            ros_node.save_mission_state('path', ros_node.path_waypoints, ros_node.current_waypoint_index, 'estop')
        elif ros_node.sequence_active:
            ros_node.save_mission_state('sequence', ros_node.sequence_zones, ros_node.current_sequence_index, 'estop')
        
        # Latch E-STOP flag ONLY (do NOT set safety_stop_active)
        ros_node.estop_active = True
        
        # Start E-STOP timer (separate from safety stop timer)
        if ros_node.estop_timer is None:
            ros_node.estop_timer = ros_node.create_timer(0.05, ros_node._publish_estop_command)
        
        # Immediately send stop command
        stop_msg = Twist()
        ros_node.cmd_vel_safety_pub.publish(stop_msg)
        
        # Cancel all active navigation
        if ros_node.path_following_active:
            ros_node.stop_path_following(save_state=False)  # Already saved above
        if ros_node.sequence_active:
            ros_node.stop_sequence(save_state=False)  # Already saved above
        if ros_node.current_nav_goal_handle is not None:
            try:
                ros_node.current_nav_goal_handle.cancel_goal_async()
                ros_node.current_nav_goal_handle = None
            except:
                pass
        
        return jsonify({'ok': True, 'message': 'E-STOP ACTIVATED - Robot halted'})
    except Exception as e:
        ros_node.get_logger().error(f'E-STOP activation error: {e}')
        return jsonify({'error': str(e)}), 500


@app.route('/api/estop/reset', methods=['POST'])
def estop_reset():
    """Reset E-STOP - allow robot motion again"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        ros_node.get_logger().info('✓ E-STOP RESET - robot enabled')
        
        # Clear E-STOP latch ONLY (do NOT touch safety_stop_active)
        ros_node.estop_active = False
        
        # Stop the E-STOP timer (do NOT touch safety_stop_timer)
        if ros_node.estop_timer is not None:
            ros_node.estop_timer.cancel()
            ros_node.estop_timer = None
        
        # Send one final zero command
        stop_msg = Twist()
        ros_node.cmd_vel_safety_pub.publish(stop_msg)
        
        # Check if there is an interrupted mission we can offer to resume
        mission_state = ros_node.load_mission_state()
        mission_summary = None
        if mission_state.get('active'):
            mission_summary = {
                'active': True,
                'type': mission_state.get('type'),
                'progress': mission_state.get('progress', 0),
                'total': len(mission_state.get('data', [])) if isinstance(mission_state.get('data'), list) else 0,
                'interrupted_by': mission_state.get('interrupted_by'),
            }
        
        return jsonify({
            'ok': True,
            'message': 'E-STOP reset - robot can now move',
            'mission': mission_summary
        })
    except Exception as e:
        ros_node.get_logger().error(f'E-STOP reset error: {e}')
        return jsonify({'error': str(e)}), 500


@app.route('/api/estop/status', methods=['GET'])
def estop_status():
    """Get E-STOP status (separate from automatic safety stop)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        status_msg = 'E-STOP ACTIVE - MASTER KILL SWITCH' if ros_node.estop_active else 'E-STOP inactive'
        return jsonify({
            'ok': True,
            'estop_active': ros_node.estop_active,
            'safety_stop_active': ros_node.safety_stop_active,
            'status': status_msg,
            'estop_timer_running': ros_node.estop_timer is not None,
            'safety_timer_running': ros_node.safety_stop_timer is not None
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/mode/manual', methods=['POST'])
def set_manual_mode():
    """Switch to manual control mode"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Direct publish is safe - ROS publishers are thread-safe
    mode_msg = String()
    mode_msg.data = 'manual'
    ros_node.mode_pub.publish(mode_msg)
    ros_node.current_mode = 'manual'
    ros_node.get_logger().info('🕹️  Switching to MANUAL mode')
    
    return jsonify({'ok': True, 'message': 'Switched to manual control mode'})


@app.route('/api/mode/auto', methods=['POST'])
def set_auto_mode():
    """Switch to autonomous navigation mode (zones)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    # Direct publish is safe - ROS publishers are thread-safe
    mode_msg = String()
    mode_msg.data = 'zones'
    ros_node.mode_pub.publish(mode_msg)
    ros_node.current_mode = 'zones'
    ros_node.get_logger().info('🤖 Switching to AUTO mode (zones)')
    
    return jsonify({'ok': True, 'message': 'Switched to autonomous navigation mode'})


@app.route('/api/robot/set_pose', methods=['POST'])
def set_initial_pose():
    """Set initial robot pose (2D Pose Estimate like RViz)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    x = data.get('x')
    y = data.get('y')
    theta = data.get('theta', 0.0)
    
    if x is None or y is None:
        return jsonify({'error': 'Missing x or y coordinates'}), 400
    
    try:
        def publish_initial_pose():
            # Create PoseWithCovarianceStamped message for AMCL
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = ros_node.get_clock().now().to_msg()
            
            # Set position
            pose_msg.pose.pose.position.x = float(x)
            pose_msg.pose.pose.position.y = float(y)
            pose_msg.pose.pose.position.z = 0.0
            
            # Set orientation (theta to quaternion)
            pose_msg.pose.pose.orientation.x = 0.0
            pose_msg.pose.pose.orientation.y = 0.0
            pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
            pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
            
            # Set covariance (small values = high confidence)
            # Format: [x, y, z, rot_x, rot_y, rot_z] - 6x6 matrix
            pose_msg.pose.covariance = [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
            ]
            
            # Publish to AMCL
            ros_node.initial_pose_pub.publish(pose_msg)
            
            ros_node.get_logger().info(f'✓ Initial pose set: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
            
            return {
                'ok': True, 
                'message': f'Robot repositioned to ({x:.2f}, {y:.2f})',
                'x': x,
                'y': y,
                'theta': theta
            }
        
        # Use thread-safe wrapper for get_clock().now() call
        result = ros_node.call_ros_from_flask(publish_initial_pose, timeout=2.0)
        return jsonify(result)
    except FutureTimeoutError:
        return jsonify({'error': 'Operation timed out'}), 500
    except Exception as e:
        ros_node.get_logger().error(f'Failed to set initial pose: {e}')
        return jsonify({'error': str(e)}), 500


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
    
    pts = ros_node.laser_scan
    pose = ros_node.robot_pose

    bins = {
        'critical': 0,  # < 0.40m
        'warning': 0,   # 0.40 - 0.80m
        'safe': 0,      # 0.80 - 2.00m
        'clear': 0      # > 2.00m
    }

    min_dist = None
    for p in pts:
        d = math.hypot(p['x'] - pose['x'], p['y'] - pose['y'])
        if min_dist is None or d < min_dist:
            min_dist = d
        if d < 0.4:
            bins['critical'] += 1
        elif d < 0.8:
            bins['warning'] += 1
        elif d < 2.0:
            bins['safe'] += 1
        else:
            bins['clear'] += 1

    total = sum(bins.values())
    blocked = bins['critical'] > 10 or (min_dist is not None and min_dist < ros_node.safety_radius)

    return jsonify({
        'obstacle_count': bins,
        'total_points': total,
        'min_distance_m': None if min_dist is None else round(min_dist, 3),
        'safety_radius_m': ros_node.safety_radius,
        'assessment': 'BLOCKED' if blocked else 'CLEAR'
    })


# ===== MAP EDITOR ROUTES =====

@app.route('/api/editor/map/preview')
def get_editor_map_preview():
    """Get rendered map with editing layers"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        rendered = ros_node.render_map_with_layers()
        if rendered is None:
            # Fall back to original map
            map_img = ros_node.convert_map_to_png()
            if map_img is None:
                return jsonify({'error': 'No map loaded'}), 500
            return jsonify({
                'image': map_img['data'],
                'width': map_img['width'],
                'height': map_img['height']
            })
        
        # Convert to base64
        img_io = io.BytesIO()
        rendered.save(img_io, 'PNG')
        img_io.seek(0)
        img_base64 = base64.b64encode(img_io.getvalue()).decode('utf-8')
        
        return jsonify({
            'image': img_base64,
            'width': rendered.width,
            'height': rendered.height
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/editor/layers', methods=['GET'])
def get_editor_layers():
    """Get all map editing layers"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    return jsonify(ros_node.map_layers)


@app.route('/api/filters', methods=['GET'])
def get_filters():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    return jsonify(ros_node._filters_only())


@app.route('/api/filters/add', methods=['POST'])
def add_filter():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    data = request.json or {}
    layer = data.get('layer')
    obj = data.get('obj', {})
    ok, result = ros_node.add_filter_object(layer, obj)
    return jsonify({'ok': ok, 'result': result}), (200 if ok else 400)


@app.route('/api/filters/delete', methods=['POST'])
def delete_filter():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    data = request.json or {}
    layer = data.get('layer')
    obj_id = data.get('id')
    ok, msg = ros_node.delete_filter_object(layer, obj_id)
    return jsonify({'ok': ok, 'message': msg}), (200 if ok else 404)


@app.route('/api/filters/clear', methods=['POST'])
def clear_filters():
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    data = request.json or {}
    layer = data.get('layer', 'all')
    ok, msg = ros_node.clear_filter_layer(layer)
    return jsonify({'ok': ok, 'message': msg}), (200 if ok else 400)


@app.route('/api/editor/layer/add', methods=['POST'])
def add_editor_layer_object():
    """Add object to an editing layer"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    layer_type = data.get('layer')
    obj = data.get('object')
    
    if not layer_type or not obj:
        return jsonify({'error': 'Missing layer or object'}), 400
    
    if layer_type in ros_node.map_layers:
        ros_node.map_layers[layer_type].append(obj)
        success = ros_node.save_map_layers()
        return jsonify({'ok': success})
    
    return jsonify({'ok': False, 'error': 'Invalid layer type'}), 400


@app.route('/api/editor/layer/clear', methods=['POST'])
def clear_editor_layer():
    """Clear all objects from an editing layer"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    layer_type = data.get('layer')
    
    if not layer_type:
        return jsonify({'error': 'Missing layer type'}), 400
    
    if layer_type in ros_node.map_layers:
        ros_node.map_layers[layer_type] = []
        success = ros_node.save_map_layers()
        return jsonify({'ok': success})
    
    return jsonify({'ok': False, 'error': 'Invalid layer type'}), 400


@app.route('/api/editor/map/export', methods=['POST'])
def export_editor_map():
    """Export edited map to new PGM file"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.json
    user_path = data.get('path', '')

    # Preferred export locations (kept short and predictable for downloads)
    maps_dir = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps')
    downloads_dir = os.path.expanduser('/home/aun/Downloads/DownloadedMaps')
    allowed_dirs = [os.path.abspath(maps_dir), os.path.abspath(downloads_dir)]

    os.makedirs(maps_dir, exist_ok=True)
    os.makedirs(downloads_dir, exist_ok=True)

    # Keep exports inside an allowed directory; normalize filenames
    if user_path:
        base_name = os.path.basename(user_path)
        if not base_name.lower().endswith('.pgm'):
            base_name = f'{base_name}.pgm'

        chosen_dir = maps_dir
        if os.path.isabs(user_path):
            user_dir = os.path.abspath(os.path.dirname(user_path))
            if any(user_dir.startswith(allowed) for allowed in allowed_dirs):
                chosen_dir = user_dir
        output_path = os.path.join(chosen_dir, base_name)
    else:
        import datetime
        stamp = datetime.datetime.now().strftime('%Y%m%d')
        output_path = os.path.join(maps_dir, f'edited_map_{stamp}.pgm')

    success = ros_node.export_edited_map(output_path)
    filename = os.path.basename(output_path)
    return jsonify({'ok': success, 'path': output_path, 'download': f'/api/map/download/{filename}'})


@app.route('/api/editor/map/save_current', methods=['POST'])
def save_to_current_map():
    """Save edited map to the currently loaded navigation map (REPLACES IT)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    success = ros_node.save_to_current_map()
    
    if success:
        return jsonify({
            'ok': True, 
            'message': 'Map saved! Restart Nav2 to use the updated map.',
            'map_path': ros_node.map_info.get('image_path')
        })
    else:
        return jsonify({'ok': False, 'error': 'Failed to save map'}), 500


@app.route('/api/editor/map/reload', methods=['POST'])
def reload_current_map():
    """Reload the map from disk (useful after external changes)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    ros_node._load_map_image()
    return jsonify({'ok': True, 'message': 'Map reloaded from disk'})


# ===== MAP MANAGER API =====
@app.route('/api/map/upload', methods=['POST'])
def upload_map():
    """Upload a new map (PGM + YAML) via Map Manager service"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500

    data = request.json
    map_name = data.get('map_name')
    pgm_base64 = data.get('pgm_base64')
    yaml_content = data.get('yaml_content')
    
    if not map_name or not pgm_base64 or not yaml_content:
        return jsonify({'success': False, 'message': 'Missing required data'}), 400
    
    try:
        # Check service availability
        if not ros_node.upload_map_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'success': False, 'message': 'Map Manager not available'}), 503
        
        # Build request
        req = UploadMap.Request()
        req.map_name = map_name
        req.pgm_base64 = pgm_base64
        req.yaml_content = yaml_content
        
        # Call service (thread-safe)
        result = ros_node.call_map_service(ros_node.upload_map_client, req, timeout=10.0)
        
        if result is None:
            return jsonify({'success': False, 'message': 'Service call timed out'}), 500
        
        return jsonify({
            'success': result.success,
            'message': result.message,
            'map_path': result.map_path if result.success else ''
        }), 200 if result.success else 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Error: {str(e)}'}), 500


@app.route('/api/map/set_active', methods=['POST'])
def set_active_map():
    """Set the active map and update all configurations"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500

    data = request.json
    map_path = data.get('map_path')
    
    if not map_path:
        return jsonify({'success': False, 'message': 'Missing map_path'}), 400
    
    try:
        # Check service availability
        if not ros_node.set_active_map_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'success': False, 'message': 'Map Manager not available'}), 503
        
        # Build request
        req = SetActiveMap.Request()
        req.map_yaml_path = map_path
        
        # Call service (thread-safe)
        result = ros_node.call_map_service(ros_node.set_active_map_client, req, timeout=5.0)
        
        if result is None:
            return jsonify({'success': False, 'message': 'Service call timed out'}), 500
        
        return jsonify({
            'success': result.success,
            'message': result.message
        }), 200 if result.success else 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Error: {str(e)}'}), 500


@app.route('/api/map/get_active', methods=['GET'])
def get_active_map():
    """Get the currently active map path"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        # Check service availability
        if not ros_node.get_active_map_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'success': False, 'message': 'Map Manager not available'}), 503
        
        # Build request
        req = GetActiveMap.Request()
        
        # Call service (thread-safe)
        result = ros_node.call_map_service(ros_node.get_active_map_client, req, timeout=5.0)
        
        if result is None:
            return jsonify({'success': False, 'message': 'Service call timed out'}), 500
        
        return jsonify({
            'success': result.success,
            'map_path': result.map_yaml_path if result.success else '',
            'message': result.message
        }), 200 if result.success else 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Error: {str(e)}'}), 500


@app.route('/api/map/list', methods=['GET'])
def list_maps():
    """List available maps (YAML + matching PGM) from allowed directories"""
    try:
        maps_dir = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps')
        downloads_dir = os.path.expanduser('/home/aun/Downloads/DownloadedMaps')
        allowed_dirs = [os.path.abspath(maps_dir), os.path.abspath(downloads_dir)]

        maps = []
        for base_dir in allowed_dirs:
            if not os.path.isdir(base_dir):
                continue
            for entry in sorted(os.listdir(base_dir)):
                if not entry.lower().endswith('.yaml'):
                    continue
                yaml_path = os.path.join(base_dir, entry)
                base_name = os.path.splitext(entry)[0]
                pgm_path = os.path.join(base_dir, f'{base_name}.pgm')
                maps.append({
                    'name': base_name,
                    'yaml_path': yaml_path,
                    'pgm_exists': os.path.exists(pgm_path),
                    'location': base_dir
                })
        return jsonify({'success': True, 'maps': maps})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


@app.route('/api/map/reload', methods=['POST'])
def reload_active_map():
    """Reload the active map from Map Manager (after switching maps)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    try:
        # Get new active map path
        new_map_path = ros_node._get_active_map_path_from_manager()
        
        # Update node's map path
        ros_node.map_path = new_map_path
        ros_node.map_info = ros_node.load_map_info()
        ros_node._load_map_image()
        
        return jsonify({
            'success': True, 
            'message': 'Map reloaded successfully',
            'map_path': new_map_path
        }), 200
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


@app.route('/api/map/download/<filename>')
def download_map_file(filename):
    """Download a map file (PGM or YAML)"""
    try:
        maps_dir = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps')
        downloads_dir = os.path.expanduser('/home/aun/Downloads/DownloadedMaps')
        allowed_dirs = [os.path.abspath(maps_dir), os.path.abspath(downloads_dir)]

        from flask import send_file

        for base_dir in allowed_dirs:
            file_path = os.path.join(base_dir, filename)
            if os.path.exists(file_path):
                return send_file(file_path, as_attachment=True, download_name=filename)
        
        return jsonify({'error': 'File not found'}), 404
    except Exception as e:
        return jsonify({'error': str(e)}), 500


def run_flask():
    """Run Flask server in a separate thread"""
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


import os, signal, time

REPO_DIR = "/home/aun/Downloads/smr300l_gazebo_ros2control-main"

def cleanup_ros_processes():
    print("\nCleaning up UI-launched ROS processes...")

    if not PSUTIL_AVAILABLE:
        print("psutil not available; can't do safe targeted cleanup.")
        return

    me = os.getpid()
    killed = []

    def looks_like_ours(proc):
        try:
            env = proc.environ()
        except Exception:
            env = {}

        try:
            cmdline = " ".join(proc.cmdline()).lower()
        except Exception:
            cmdline = ""

        # Target only UI-marked processes OR ones launched from your workspace
        if env.get("ZONE_WEB_UI_STACK") == "1":
            return True
        if REPO_DIR.lower() in cmdline:
            return True

        # Extra allowlist for known “parents” IF they’re in your workspace cmdline
        # (prevents killing unrelated system ROS)
        if any(x in cmdline for x in ["ros2 launch", "launch.py", "gzserver", "gzclient", "ros2_control_node"]):
            if REPO_DIR.lower() in cmdline:
                return True

        return False

    # 1) terminate process groups first (prevents respawn)
    for proc in psutil.process_iter(["pid", "name"]):
        pid = proc.info["pid"]
        if pid == me:
            continue

        try:
            if not looks_like_ours(proc):
                continue

            # kill entire process group if possible
            try:
                pgid = os.getpgid(pid)
                os.killpg(pgid, signal.SIGTERM)
                killed.append(("TERM_PG", pid))
            except Exception:
                proc.terminate()
                killed.append(("TERM", pid))

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    time.sleep(1.0)

    # 2) escalate to SIGKILL for survivors
    for proc in psutil.process_iter(["pid"]):
        pid = proc.info["pid"]
        if pid == me:
            continue
        try:
            if not looks_like_ours(proc):
                continue
            if proc.is_running():
                try:
                    pgid = os.getpgid(pid)
                    os.killpg(pgid, signal.SIGKILL)
                    killed.append(("KILL_PG", pid))
                except Exception:
                    proc.kill()
                    killed.append(("KILL", pid))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    # 3) refresh ROS CLI view (optional but helps)
    # os.system("ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1")

    print(f"Cleanup attempted on {len(killed)} processes/groups.")



def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    
    # Create executor first
    executor = MultiThreadedExecutor(num_threads=4)

    # Pass executor to node for thread-safe Flask→ROS calls
    ros_node = ZoneWebServer(executor)

    # Import and create StackManager node
    from next_ros2ws_core.stack_manager import StackManager
    stack_manager_node = StackManager()

    auto_reloc_node = None
    try:
        from next_ros2ws_tools.auto_reloc import CorrelativeRelocalizer
        auto_reloc_node = CorrelativeRelocalizer()
        ros_node.get_logger().info('Auto relocalizer started (/auto_relocate)')
    except Exception as e:
        ros_node.get_logger().warn(f'Auto relocalizer not started: {e}')

    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    ros_node.get_logger().info('Web UI available at: http://localhost:5000')

    # Add nodes to executor
    executor.add_node(ros_node)
    executor.add_node(stack_manager_node)
    if auto_reloc_node is not None:
        executor.add_node(auto_reloc_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n⚠️  Shutdown requested...")
    finally:
        # Stop any stack processes started by StackManager
        try:
            stack_manager_node._stop_nav()
            stack_manager_node._stop_slam()
        except Exception as e:
            print(f"  StackManager stop error: {e}")

        # Clean up ROS processes first
        cleanup_ros_processes()
        
        # Then clean up nodes
        try:
            ros_node.destroy_node()
            stack_manager_node.destroy_node()
            if auto_reloc_node is not None:
                auto_reloc_node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Already shut down
        
        print("✓ Shutdown complete")

@app.route('/api/stack/mode', methods=['POST'])
def set_stack_mode():
    """Switch between NAV, SLAM, STOP modes via ROS service (thread-safe)"""
    if ros_node is None:
        return jsonify({'ok': False, 'message': 'ROS node not initialized'}), 500

    data = request.get_json(force=True)
    mode = (data.get('mode') or '').lower()
    if mode not in ['nav', 'slam', 'stop']:
        return jsonify({'ok': False, 'message': 'Invalid mode'}), 400

    from next_ros2ws_interfaces.srv import SetStackMode
    if not ros_node.set_stack_mode_client.wait_for_service(timeout_sec=2.0):
        return jsonify({'ok': False, 'message': 'StackManager service not available'}), 500

    req = SetStackMode.Request()
    req.mode = mode
    future = ros_node.set_stack_mode_client.call_async(req)

    end_time = time.time() + 10.0
    while time.time() < end_time:
        if future.done():
            try:
                res = future.result()
            except Exception as e:
                ros_node.get_logger().error(f'/api/stack/mode result error: {e}')
                return jsonify({'ok': False, 'message': str(e)}), 500
            if res.ok:
                ros_node.current_stack_mode = mode
            return jsonify({'ok': bool(res.ok), 'message': str(res.message)}), 200 if res.ok else 500
        time.sleep(0.05)

    return jsonify({'ok': False, 'message': 'Timeout waiting for stack switch'}), 500


@app.route('/api/stack/status', methods=['GET'])
def get_stack_status():
    """Get current stack mode (nav/slam/stop)"""
    mode = getattr(ros_node, 'current_stack_mode', 'stopped')
    return jsonify({'ok': True, 'mode': mode})


@app.route('/api/slam/save_map', methods=['POST'])
def save_slam_map():
    """Save the current SLAM map to disk via slam_toolbox."""
    if ros_node is None:
        return jsonify({'ok': False, 'message': 'ROS node not initialized'}), 500

    data = request.get_json(force=True) or {}
    map_name = (data.get('map_name') or '').strip()
    set_active = bool(data.get('set_active', False))

    if not map_name:
        map_name = time.strftime('slam_map_%Y%m%d_%H%M%S')

    if '/' in map_name or '\\' in map_name:
        return jsonify({'ok': False, 'message': 'Invalid map name'}), 400

    maps_dir = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps')
    map_base = os.path.join(maps_dir, map_name)

    if not ros_node.slam_save_map_client.wait_for_service(timeout_sec=2.0):
        return jsonify({'ok': False, 'message': 'slam_toolbox save_map service not available'}), 500

    req = SlamSaveMap.Request()
    req.name.data = map_base

    future = ros_node.slam_save_map_client.call_async(req)

    end_time = time.time() + 10.0
    while time.time() < end_time:
        if future.done():
            try:
                res = future.result()
            except Exception as e:
                ros_node.get_logger().error(f'/api/slam/save_map result error: {e}')
                return jsonify({'ok': False, 'message': str(e)}), 500

            if res.result != SlamSaveMap.Response.RESULT_SUCCESS:
                return jsonify({'ok': False, 'message': f'SLAM map save failed (code {res.result})'}), 500

            map_yaml_path = map_base + '.yaml'
            map_pgm_path = map_base + '.pgm'

            active_message = None
            if set_active:
                try:
                    req_active = SetActiveMap.Request()
                    req_active.map_yaml_path = map_yaml_path
                    active_result = ros_node.call_map_service(
                        ros_node.set_active_map_client,
                        req_active,
                        timeout=3.0
                    )
                    if active_result and active_result.success:
                        active_message = active_result.message
                    else:
                        active_message = active_result.message if active_result else 'Failed to set active map'
                except Exception as e:
                    active_message = f'Failed to set active map: {e}'

            message = f'Map saved: {map_name}'
            if active_message:
                message = f'{message}. {active_message}'

            return jsonify({
                'ok': True,
                'message': message,
                'map_name': map_name,
                'map_yaml_path': map_yaml_path,
                'map_pgm_path': map_pgm_path,
                'set_active': set_active
            }), 200

        time.sleep(0.05)

    return jsonify({'ok': False, 'message': 'Timeout waiting for slam_toolbox save_map'}), 500


@app.route('/api/stack/health', methods=['GET'])
def stack_health():
    """Get stack health diagnostics - check for missing topics/nodes"""
    if ros_node is None:
        return jsonify({'ok': False, 'message': 'ROS node not initialized'}), 500

    def check():
        # Cheap checks: topics + TF
        topics = set(ros_node.get_topic_names_and_types())
        nodes = set(ros_node.get_node_names())

        health = {
            'has_amcl_pose': '/amcl_pose' in topics,
            'has_map': '/map' in topics,
            'has_tf': '/tf' in topics,
            'has_tf_static': '/tf_static' in topics,
            'has_scan': '/scan' in topics,
            'has_scan2': '/scan2' in topics,
            'robot_pose_seen': ros_node.robot_pose is not None,
            'confidence': ros_node.localization_confidence,
            'current_mode': ros_node.current_mode,
            'stack_mode': ros_node.current_stack_mode,
        }
        return health

    try:
        return jsonify(ros_node.call_ros_from_flask(check, timeout=2.0))
    except Exception as e:
        return jsonify({'ok': False, 'message': str(e)}), 500


@app.route('/api/auto_relocate', methods=['POST'])
def auto_relocate():
    """Trigger auto relocalization via /auto_relocate (std_srvs/Trigger)."""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500

    def call_auto_reloc():
        if not ros_node.auto_reloc_client.wait_for_service(timeout_sec=0.5):
            return {'ok': False, 'message': 'Auto relocalize service not available'}

        req = Trigger.Request()
        future = ros_node.auto_reloc_client.call_async(req)
        end_time = time.time() + 10.0
        while time.time() < end_time:
            if future.done():
                res = future.result()
                return {'ok': bool(res.success), 'message': str(res.message)}
            time.sleep(0.05)
        return {'ok': False, 'message': 'Auto relocalize timed out'}

    try:
        result = ros_node.call_ros_from_flask(call_auto_reloc, timeout=12.0)
        return jsonify(result), 200 if result.get('ok') else 500
    except Exception as e:
        return jsonify({'ok': False, 'message': str(e)}), 500


@app.route('/api/map/live')
def get_live_map():
    """Get the latest live occupancy grid from /map as raw data (dynamic map)"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500

    msg = ros_node.latest_map_msg
    if msg is None:
        # Return 200 with ok=False to avoid 404 console spam
        return jsonify({'ok': False, 'error': 'No live map data received yet'}), 200

    # Convert OccupancyGrid to a JSON-serializable structure compatible with frontend expectations
    return jsonify({
        'ok': True,
        'map': {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y,
                        'z': msg.info.origin.position.z,
                    },
                    'orientation': {
                        'x': msg.info.origin.orientation.x,
                        'y': msg.info.origin.orientation.y,
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w,
                    }
                }
            },
            'data': list(msg.data)  # Convert from array to list for JSON
        }
    })
