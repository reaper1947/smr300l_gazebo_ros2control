#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from next_ros2ws_interfaces.srv import (
    SaveZone, DeleteZone, UpdateZoneParams, ReorderZones,
    SavePath, DeletePath, SaveLayout, LoadLayout, DeleteLayout,
    SetMaxSpeed, SetControlMode, SetSafetyOverride, SetEStop
)
from next_ros2ws_interfaces.action import GoToZone as GoToZoneAction, FollowPath as FollowPathAction
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
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
        
        self.delete_zone_srv = self.create_service(
            DeleteZone,
            '/delete_zone',
            self.delete_zone_callback
        )
        
        self.update_zone_params_srv = self.create_service(
            UpdateZoneParams,
            '/update_zone_params',
            self.update_zone_params_callback
        )
        
        self.reorder_zones_srv = self.create_service(
            ReorderZones,
            '/reorder_zones',
            self.reorder_zones_callback
        )
        
        # Layout services
        self.save_layout_srv = self.create_service(
            SaveLayout,
            '/save_layout',
            self.save_layout_callback
        )
        
        self.load_layout_srv = self.create_service(
            LoadLayout,
            '/load_layout',
            self.load_layout_callback
        )
        
        self.delete_layout_srv = self.create_service(
            DeleteLayout,
            '/delete_layout',
            self.delete_layout_callback
        )
        
        # Control services
        self.set_max_speed_srv = self.create_service(
            SetMaxSpeed,
            '/set_max_speed',
            self.set_max_speed_callback
        )
        
        self.set_control_mode_srv = self.create_service(
            SetControlMode,
            '/set_control_mode',
            self.set_control_mode_callback
        )
        
        self.set_safety_override_srv = self.create_service(
            SetSafetyOverride,
            '/set_safety_override',
            self.set_safety_override_callback
        )
        
        self.set_estop_srv = self.create_service(
            SetEStop,
            '/set_estop',
            self.set_estop_callback
        )
        
        # Store settings in memory
        self.max_speed = 0.5
        self.control_mode = 'manual'
        self.safety_override = False
        self.estop_active = False
        self.layouts_file = os.path.expanduser('~/layouts.yaml')
        self.paths_file = os.path.expanduser('~/paths.yaml')
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Action servers bridging to Nav2
        self.go_to_zone_action_server = ActionServer(
            self,
            GoToZoneAction,
            '/go_to_zone',
            execute_callback=self.execute_go_to_zone,
            goal_callback=self.go_to_zone_goal_callback,
            cancel_callback=self.go_to_zone_cancel_callback
        )

        self.follow_path_action_server = ActionServer(
            self,
            FollowPathAction,
            '/follow_path',
            execute_callback=self.execute_follow_path,
            goal_callback=self.follow_path_goal_callback,
            cancel_callback=self.follow_path_cancel_callback
        )

        self.go_to_next_ros2ws_goal = None
        self.follow_path_nav_goal = None
        
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
        
        # Reload zones from file before saving to avoid overwriting deletions
        self.load_zones()
        
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
    
    def go_to_zone_goal_callback(self, goal_request):
        """Validate incoming GoToZone action goals"""
        self.load_zones()
        zones = self.zones.get("zones", {})

        if self.estop_active:
            self.get_logger().warn('Rejecting go_to_zone goal: E-STOP active')
            return GoalResponse.REJECT

        if goal_request.name not in zones:
            self.get_logger().warn(f'GoToZone goal rejected: zone "{goal_request.name}" not found')
            return GoalResponse.REJECT

        if self.follow_path_nav_goal is not None:
            self.get_logger().warn('GoToZone goal rejected: follow_path is in progress')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def go_to_zone_cancel_callback(self, goal_handle):
        if self.go_to_next_ros2ws_goal is not None:
            try:
                self.go_to_next_ros2ws_goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f'Failed to cancel Nav2 goal for GoToZone: {e}')
        return CancelResponse.ACCEPT

    async def execute_go_to_zone(self, goal_handle):
        zone_name = goal_handle.request.name
        result = GoToZoneAction.Result()

        # Ensure latest zones are loaded
        self.load_zones()
        zones = self.zones.get("zones", {})

        if zone_name not in zones:
            result.success = False
            result.message = f'Zone "{zone_name}" not found'
            goal_handle.abort()
            return result

        if self.estop_active:
            result.success = False
            result.message = 'E-STOP active'
            goal_handle.abort()
            return result

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = 'Nav2 action server not available'
            goal_handle.abort()
            return result

        zone = zones[zone_name]

        # Build Nav2 goal
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

        # Publish initial feedback
        feedback = GoToZoneAction.Feedback()
        feedback.progress = 0.0
        feedback.status = 'Sending goal to Nav2'
        goal_handle.publish_feedback(feedback)

        # Send Nav2 goal
        nav_goal_future = self.nav_client.send_goal_async(goal_msg)
        nav_goal_handle = await nav_goal_future

        if not nav_goal_handle.accepted:
            result.success = False
            result.message = 'Nav2 rejected goal'
            goal_handle.abort()
            return result

        self.go_to_next_ros2ws_goal = nav_goal_handle
        nav_result = await nav_goal_handle.get_result_async()

        # Check cancel first
        if goal_handle.is_cancel_requested or nav_result.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            result.success = False
            result.message = 'GoToZone canceled'
        elif nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            feedback.progress = 1.0
            feedback.status = 'Arrived'
            goal_handle.publish_feedback(feedback)
            result.success = True
            result.message = f'Reached zone "{zone_name}"'
            goal_handle.succeed()
        else:
            goal_handle.abort()
            result.success = False
            result.message = f'Nav2 failed with status {nav_result.status}'

        self.go_to_next_ros2ws_goal = None
        return result

    def follow_path_goal_callback(self, goal_request):
        """Validate incoming FollowPath action goals"""
        if self.estop_active:
            self.get_logger().warn('Rejecting follow_path goal: E-STOP active')
            return GoalResponse.REJECT

        if len(goal_request.waypoints) < 1:
            self.get_logger().warn('Rejecting follow_path goal: no waypoints provided')
            return GoalResponse.REJECT

        if self.go_to_next_ros2ws_goal is not None:
            self.get_logger().warn('Rejecting follow_path goal: go_to_zone is in progress')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def follow_path_cancel_callback(self, goal_handle):
        if self.follow_path_nav_goal is not None:
            try:
                self.follow_path_nav_goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f'Failed to cancel Nav2 goal for FollowPath: {e}')
        return CancelResponse.ACCEPT

    async def execute_follow_path(self, goal_handle):
        waypoints = goal_handle.request.waypoints
        total = len(waypoints)
        result = FollowPathAction.Result()

        if total == 0:
            result.success = False
            result.message = 'No waypoints provided'
            goal_handle.abort()
            return result

        # Wait for Nav2 action server once
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = 'Nav2 action server not available'
            goal_handle.abort()
            return result

        feedback = FollowPathAction.Feedback()

        for idx, waypoint in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                if self.follow_path_nav_goal is not None:
                    try:
                        self.follow_path_nav_goal.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f'Error canceling Nav2 waypoint goal: {e}')
                goal_handle.canceled()
                result.success = False
                result.message = 'FollowPath canceled'
                result.completed = idx
                self.follow_path_nav_goal = None
                return result

            feedback.current_index = idx
            feedback.total = total
            feedback.status = f'Navigating to waypoint {idx + 1}/{total}'
            goal_handle.publish_feedback(feedback)

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = waypoint
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

            nav_goal_handle = await self.nav_client.send_goal_async(nav_goal)

            if not nav_goal_handle.accepted:
                result.success = False
                result.message = f'Nav2 rejected waypoint {idx + 1}'
                result.completed = idx
                goal_handle.abort()
                return result

            self.follow_path_nav_goal = nav_goal_handle
            nav_result = await nav_goal_handle.get_result_async()

            if goal_handle.is_cancel_requested or nav_result.status == GoalStatus.STATUS_CANCELED:
                try:
                    nav_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                goal_handle.canceled()
                result.success = False
                result.message = 'FollowPath canceled'
                result.completed = idx
                self.follow_path_nav_goal = None
                return result

            if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
                goal_handle.abort()
                result.success = False
                result.message = f'Waypoint {idx + 1} failed with status {nav_result.status}'
                result.completed = idx
                self.follow_path_nav_goal = None
                return result

            result.completed = idx + 1

        self.follow_path_nav_goal = None
        result.success = True
        result.message = f'Completed {total} waypoints'
        goal_handle.succeed()
        return result
    
    def delete_zone_callback(self, request, response):
        """Delete a zone"""
        zone_name = request.name
        self.load_zones()
        zones = self.zones.get("zones", {})
        
        if zone_name not in zones:
            response.ok = False
            response.message = f'Zone "{zone_name}" not found'
            self.get_logger().warn(response.message)
            return response
        
        del zones[zone_name]
        self.save_zones()
        response.ok = True
        response.message = f'Zone "{zone_name}" deleted'
        self.get_logger().info(response.message)
        return response
    
    def update_zone_params_callback(self, request, response):
        """Update zone parameters (type, speed, action, charge_duration)"""
        zone_name = request.name
        self.load_zones()
        zones = self.zones.get("zones", {})
        
        if zone_name not in zones:
            response.ok = False
            response.message = f'Zone "{zone_name}" not found'
            return response
        
        # Update metadata
        zones[zone_name]['type'] = request.type
        zones[zone_name]['speed'] = request.speed
        
        if request.type == 'action' and request.action:
            zones[zone_name]['action'] = request.action
        elif request.type == 'charge':
            zones[zone_name]['charge_duration'] = request.charge_duration
        
        # Clean up old parameters
        if request.type != 'action':
            zones[zone_name].pop('action', None)
        if request.type != 'charge':
            zones[zone_name].pop('charge_duration', None)
        
        self.save_zones()
        response.ok = True
        response.message = f'Zone "{zone_name}" parameters updated'
        self.get_logger().info(response.message)
        return response
    
    def reorder_zones_callback(self, request, response):
        """Reorder zones by name sequence"""
        self.load_zones()
        zones = self.zones.get("zones", {})
        
        # Create new ordered dict
        new_zones = {}
        for zone_name in request.zone_names:
            if zone_name in zones:
                new_zones[zone_name] = zones[zone_name]
        
        self.zones["zones"] = new_zones
        self.save_zones()
        response.ok = True
        response.message = 'Zones reordered'
        self.get_logger().info(response.message)
        return response
    
    def save_layout_callback(self, request, response):
        """Save current configuration as a layout"""
        try:
            layouts = self._load_layouts()
            zones = self.zones.get("zones", {})
            paths = self._load_paths()
            
            layouts[request.name] = {
                'description': request.description,
                'zones': zones,
                'paths': paths
            }
            
            with open(self.layouts_file, 'w') as f:
                yaml.dump({'layouts': layouts}, f, default_flow_style=False)
            
            response.ok = True
            response.message = f'Layout "{request.name}" saved'
            self.get_logger().info(response.message)
        except Exception as e:
            response.ok = False
            response.message = f'Failed to save layout: {str(e)}'
            self.get_logger().error(response.message)
        return response
    
    def load_layout_callback(self, request, response):
        """Load a layout configuration"""
        try:
            layouts = self._load_layouts()
            if request.name not in layouts:
                response.ok = False
                response.message = f'Layout "{request.name}" not found'
                return response
            
            layout = layouts[request.name]
            self.zones = {"zones": layout.get("zones", {})}
            self.save_zones()
            
            response.ok = True
            response.message = f'Layout "{request.name}" loaded'
            self.get_logger().info(response.message)
        except Exception as e:
            response.ok = False
            response.message = f'Failed to load layout: {str(e)}'
        return response
    
    def delete_layout_callback(self, request, response):
        """Delete a layout"""
        try:
            layouts = self._load_layouts()
            if request.name not in layouts:
                response.ok = False
                response.message = f'Layout "{request.name}" not found'
                return response
            
            del layouts[request.name]
            with open(self.layouts_file, 'w') as f:
                yaml.dump({'layouts': layouts}, f, default_flow_style=False)
            
            response.ok = True
            response.message = f'Layout "{request.name}" deleted'
            self.get_logger().info(response.message)
        except Exception as e:
            response.ok = False
            response.message = f'Failed to delete layout: {str(e)}'
        return response
    
    def set_max_speed_callback(self, request, response):
        """Set maximum speed for manual control"""
        if request.speed <= 0 or request.speed > 2.0:
            response.ok = False
            response.message = 'Speed must be between 0.01 and 2.0 m/s'
            return response
        
        self.max_speed = request.speed
        response.ok = True
        response.message = f'Max speed set to {request.speed} m/s'
        self.get_logger().info(response.message)
        return response
    
    def set_control_mode_callback(self, request, response):
        """Set control mode (manual, sequence, autonomous)"""
        valid_modes = ['manual', 'sequence', 'autonomous']
        if request.mode not in valid_modes:
            response.ok = False
            response.message = f'Invalid mode. Use: {", ".join(valid_modes)}'
            return response
        
        self.control_mode = request.mode
        response.ok = True
        response.message = f'Control mode set to {request.mode}'
        self.get_logger().info(response.message)
        return response
    
    def set_safety_override_callback(self, request, response):
        """Enable/disable safety override"""
        self.safety_override = request.override_active
        response.ok = True
        status = 'enabled' if request.override_active else 'disabled'
        response.message = f'Safety override {status}'
        self.get_logger().info(response.message)
        return response
    
    def set_estop_callback(self, request, response):
        """Activate/deactivate emergency stop"""
        self.estop_active = request.activate
        response.ok = True
        status = 'activated' if request.activate else 'deactivated'
        response.message = f'E-STOP {status}'
        self.get_logger().info(response.message)
        return response
    
    def _load_layouts(self):
        """Load layouts from file"""
        if os.path.exists(self.layouts_file):
            try:
                with open(self.layouts_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                return data.get('layouts', {})
            except Exception as e:
                self.get_logger().error(f'Failed to load layouts: {e}')
                return {}
        return {}
    
    def _load_paths(self):
        """Load paths from file"""
        if os.path.exists(self.paths_file):
            try:
                with open(self.paths_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                return data.get('paths', {})
            except Exception as e:
                self.get_logger().error(f'Failed to load paths: {e}')
                return {}
        return {}
    
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
