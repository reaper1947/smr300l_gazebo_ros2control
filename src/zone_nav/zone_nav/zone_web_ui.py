#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from zone_nav_interfaces.srv import SaveZone, GoToZone
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
        
        # Subscribe to robot position - match AMCL's QoS profile
        self.robot_pose = None
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
        self.get_logger().info(f'Robot pose updated: x={self.robot_pose["x"]:.2f}, y={self.robot_pose["y"]:.2f}, theta={self.robot_pose["theta"]:.2f}', throttle_duration_sec=2.0)
    
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
        
        request = GoToZone.Request()
        request.name = name
        future = self.go_to_zone_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            return {'ok': response.ok, 'message': response.message}
        else:
            return {'ok': False, 'message': 'Service call timeout'}
    
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
    
    return jsonify({'pose': ros_node.robot_pose})


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
