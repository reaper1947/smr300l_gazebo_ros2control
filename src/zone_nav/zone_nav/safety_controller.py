#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, SetBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math


class SafetyController(Node):
    """
    Safety controller that monitors robot state and publishes emergency stops
    to the highest priority twist_mux topic (cmd_vel_safety).
    
    Monitors:
    - Localization confidence (AMCL)
    - Obstacle proximity (LIDAR)
    - Emergency stop requests (service)
    """
    
    def __init__(self):
        super().__init__('safety_controller')
        
        # Publisher for safety commands (highest priority in twist_mux)
        self.safety_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        
        # Safety state
        self.estop_active = False
        self.low_confidence_stop = False
        self.obstacle_stop = False
        self.manual_override = False
        
        # Localization tracking
        self.localization_confidence = 100.0
        self.confidence_threshold = 30.0
        self.confidence_resume_threshold = 35.0
        
        # Obstacle detection
        self.safety_radius = 0.35  # meters - minimum clearance
        self.min_obstacle_distance = float('inf')
        
        # Subscribe to AMCL pose with correct QoS
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
        
        # Subscribe to lidar
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Services
        self.estop_srv = self.create_service(SetBool, 'safety/emergency_stop', self.estop_callback)
        self.override_srv = self.create_service(SetBool, 'safety/override', self.override_callback)
        self.status_srv = self.create_service(Trigger, 'safety/status', self.status_callback)
        
        # Timer to continuously publish stops when safety conditions are active
        self.safety_timer = self.create_timer(0.1, self.safety_check_callback)
        
        self.get_logger().info('🛡️  Safety Controller active - monitoring robot safety')
        self.get_logger().info('Services: safety/emergency_stop, safety/override, safety/status')
    
    def pose_callback(self, msg):
        """Monitor localization confidence from AMCL"""
        # Calculate confidence from covariance
        cov = msg.pose.covariance
        pos_variance = cov[0] + cov[7]  # x + y variance
        
        if pos_variance < 0.01:
            self.localization_confidence = 100.0
        elif pos_variance < 0.1:
            self.localization_confidence = 90.0
        elif pos_variance < 0.5:
            self.localization_confidence = 70.0
        elif pos_variance < 1.0:
            self.localization_confidence = 50.0
        else:
            self.localization_confidence = max(0.0, 50.0 - (pos_variance - 1.0) * 10.0)
        
        # Check if confidence dropped too low
        if self.localization_confidence < self.confidence_threshold:
            if not self.low_confidence_stop and not self.manual_override:
                self.low_confidence_stop = True
                self.get_logger().warn(f'⚠️  SAFETY: Low localization confidence ({self.localization_confidence:.1f}%) - stopping robot')
        elif self.localization_confidence > self.confidence_resume_threshold:
            if self.low_confidence_stop:
                self.low_confidence_stop = False
                self.get_logger().info(f'✓ Confidence restored ({self.localization_confidence:.1f}%) - safety clear')
    
    def scan_callback(self, msg):
        """Monitor obstacles from LIDAR"""
        min_dist = float('inf')
        
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                min_dist = min(min_dist, distance)
        
        self.min_obstacle_distance = min_dist
        
        # Check if obstacle too close
        if min_dist < self.safety_radius:
            if not self.obstacle_stop:
                self.obstacle_stop = True
                self.get_logger().warn(f'⚠️  SAFETY: Obstacle detected at {min_dist:.2f}m (< {self.safety_radius}m) - stopping robot')
        elif min_dist > self.safety_radius + 0.1:  # Hysteresis
            if self.obstacle_stop:
                self.obstacle_stop = False
                self.get_logger().info(f'✓ Obstacle cleared ({min_dist:.2f}m) - safety clear')
    
    def safety_check_callback(self):
        """Continuously check safety and publish stop commands if needed"""
        should_stop = False
        reasons = []
        
        if self.estop_active:
            should_stop = True
            reasons.append('E-STOP')
        
        if self.low_confidence_stop and not self.manual_override:
            should_stop = True
            reasons.append(f'LOW_CONFIDENCE({self.localization_confidence:.1f}%)')
        
        if self.obstacle_stop:
            should_stop = True
            reasons.append(f'OBSTACLE({self.min_obstacle_distance:.2f}m)')
        
        # Publish stop command if any safety condition is active
        if should_stop:
            twist = Twist()  # All zeros = stop
            self.safety_pub.publish(twist)
            
            # Log periodically (every 2 seconds)
            if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
                self.get_logger().debug(f'🛑 Safety stop active: {", ".join(reasons)}')
    
    def estop_callback(self, request, response):
        """Service to activate/deactivate emergency stop"""
        self.estop_active = request.data
        
        if self.estop_active:
            self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED')
            response.success = True
            response.message = 'Emergency stop activated - robot stopped'
        else:
            self.get_logger().info('✓ Emergency stop deactivated')
            response.success = True
            response.message = 'Emergency stop deactivated - robot can move'
        
        return response
    
    def override_callback(self, request, response):
        """Service to override safety checks (use with caution!)"""
        self.manual_override = request.data
        
        if self.manual_override:
            self.get_logger().warn('⚠️  SAFETY OVERRIDE ENABLED - confidence checks disabled')
            self.low_confidence_stop = False
            response.success = True
            response.message = 'Safety override enabled - confidence checks disabled'
        else:
            self.get_logger().info('✓ Safety override disabled - normal safety checks active')
            response.success = True
            response.message = 'Safety override disabled'
        
        return response
    
    def status_callback(self, request, response):
        """Service to get current safety status"""
        status_lines = [
            f"Safety Controller Status:",
            f"  E-Stop: {'ACTIVE' if self.estop_active else 'inactive'}",
            f"  Low Confidence Stop: {'ACTIVE' if self.low_confidence_stop else 'inactive'}",
            f"  Obstacle Stop: {'ACTIVE' if self.obstacle_stop else 'inactive'}",
            f"  Manual Override: {'ENABLED' if self.manual_override else 'disabled'}",
            f"",
            f"Sensors:",
            f"  Localization: {self.localization_confidence:.1f}% (threshold: {self.confidence_threshold}%)",
            f"  Nearest Obstacle: {self.min_obstacle_distance:.2f}m (safety: {self.safety_radius}m)"
        ]
        
        response.success = True
        response.message = '\n'.join(status_lines)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
