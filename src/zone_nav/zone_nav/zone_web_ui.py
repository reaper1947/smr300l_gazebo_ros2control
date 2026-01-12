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
        
        # Publisher for goal poses
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Publisher for emergency stop - use joystick topic (highest priority in twist_mux)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        
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
        
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % sample_rate == 0 and msg.range_min < r < msg.range_max:
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
        
        # Call service
        request = SaveZone.Request()
        request.name = name
        future = self.save_zone_client.call_async(request)
        
        # Wait for result
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            return {'ok': response.ok, 'message': response.message}
        else:
            return {'ok': False, 'message': 'Service call timeout'}
    
    def go_to_zone(self, name):
        """Call go_to_zone service"""
        if not self.go_to_zone_client.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'message': 'Go to zone service not available'}
        
        # Clear safety stop if confidence is good
        if self.localization_confidence > self.confidence_resume_threshold:
            self.safety_stop_active = False
            self.safety_override_active = False
        
        # Check if safety stop is active (unless overridden)
        if self.safety_stop_active and not self.safety_override_active:
            return {'ok': False, 'message': f'Cannot navigate: Localization confidence too low ({self.localization_confidence:.1f}%). Improve robot localization first or use "Force Resume" if obstacles are cleared.'}
        
        request = GoToZone.Request()
        request.name = name
        # Send the request asynchronously without blocking
        future = self.go_to_zone_client.call_async(request)
        
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
    
    if not name or x is None or y is None:
        return jsonify({'error': 'Missing required fields'}), 400
    
    result = ros_node.save_zone(name, x, y, theta)
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


@app.route('/api/safety/force_resume', methods=['POST'])
def force_resume():
    """Force resume navigation by overriding safety stop"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    result = ros_node.force_resume_navigation()
    return jsonify(result)


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
