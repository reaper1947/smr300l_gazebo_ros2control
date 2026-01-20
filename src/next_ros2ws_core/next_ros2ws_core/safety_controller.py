#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
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
        self.safety_pub = self.create_publisher(Twist, 'cmd_vel_safety', 10)
        
        # Safety state
        self.estop_active = False
        self.low_confidence_stop = False
        self.obstacle_stop = False
        self.manual_override = False
        self.control_mode = 'unknown'
        self.lenient_mode = False
        self.lenient_in_manual = bool(self.declare_parameter('lenient_in_manual', True).value)
        self.lenient_scale = float(self.declare_parameter('slam_lenient_scale', 0.1).value)
        
        # Localization tracking
        self.localization_confidence = 100.0
        self.confidence_threshold = 30.0
        self.confidence_resume_threshold = 35.0
        
        # Obstacle detection - using research-based distance thresholds
        # Paper: "Detection and Classification of Obstacles Using a 2D LiDAR Sensor"
        self.base_critical_zone = 0.35   # < 0.35m = CRITICAL - immediate stop
        self.base_warning_zone = 0.60    # 0.35-0.60m = WARNING - slow down
        self.base_safe_zone = 1.0        # 0.60-1.0m = SAFE - monitor
        self.critical_zone = self.base_critical_zone
        self.warning_zone = self.base_warning_zone
        self.safe_zone = self.base_safe_zone
        
        self.min_obstacle_distance_front = float('inf')
        self.min_obstacle_distance_rear = float('inf')
        self.critical_obstacles_front = 0
        self.critical_obstacles_rear = 0
        self.warning_obstacles_front = 0
        self.warning_obstacles_rear = 0
        
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
        
        # Subscribe to BOTH front and rear lidars
        self.scan_front_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_front_callback,
            10
        )
        
        self.scan_rear_sub = self.create_subscription(
            LaserScan,
            '/scan2',
            self.scan_rear_callback,
            10
        )

        # Track control mode to optionally relax safety during manual/SLAM mapping
        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10
        )
        
        self.get_logger().info('📡 Monitoring FRONT lidar: /scan')
        self.get_logger().info('📡 Monitoring REAR lidar: /scan2')
        
        # Services
        self.estop_srv = self.create_service(SetBool, 'safety/emergency_stop', self.estop_callback)
        self.override_srv = self.create_service(SetBool, 'safety/override', self.override_callback)
        self.status_srv = self.create_service(Trigger, 'safety/status', self.status_callback)
        
        # Timer to continuously publish stops when safety conditions are active
        self.safety_timer = self.create_timer(0.1, self.safety_check_callback)
        
        self.get_logger().info('🛡️  Safety Controller active - monitoring robot safety')
        self.get_logger().info('Services: safety/emergency_stop, safety/override, safety/status')

    def _apply_lenient_zones(self):
        scale = float(self.lenient_scale)
        if scale <= 0.0:
            scale = 0.1
        elif scale > 1.0:
            scale = 1.0

        if self.lenient_mode:
            self.critical_zone = self.base_critical_zone * scale
            self.warning_zone = self.base_warning_zone * scale
            self.safe_zone = self.base_safe_zone * scale
        else:
            self.critical_zone = self.base_critical_zone
            self.warning_zone = self.base_warning_zone
            self.safe_zone = self.base_safe_zone

    def _set_lenient_mode(self, enabled, reason=''):
        enabled = bool(enabled)
        if enabled == self.lenient_mode:
            return
        self.lenient_mode = enabled
        self._apply_lenient_zones()
        suffix = f' ({reason})' if reason else ''
        if enabled:
            self.get_logger().warn(
                f'Safety leniency enabled{suffix}: '
                f'critical<{self.critical_zone:.2f}m, warning<{self.warning_zone:.2f}m'
            )
        else:
            self.get_logger().info(
                f'Safety leniency disabled{suffix}: '
                f'critical<{self.critical_zone:.2f}m, warning<{self.warning_zone:.2f}m'
            )

    def control_mode_callback(self, msg):
        mode = (msg.data or '').strip().lower()
        if not mode:
            return
        self.control_mode = mode
        if self.lenient_in_manual:
            self._set_lenient_mode(mode == 'manual', reason=f'control_mode={mode}')
    
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
    
    def scan_front_callback(self, msg):
        """Monitor obstacles from FRONT LIDAR (/scan) using distance classification"""
        min_dist = float('inf')
        critical_count = 0
        warning_count = 0
        
        # Analyze each distance reading
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                min_dist = min(min_dist, distance)
                
                # Classify obstacle by distance (research-based zones)
                if distance < self.critical_zone:
                    critical_count += 1
                elif distance < self.warning_zone:
                    warning_count += 1
        
        self.min_obstacle_distance_front = min_dist
        self.critical_obstacles_front = critical_count
        self.warning_obstacles_front = warning_count
        
        # CRITICAL ZONE: Immediate stop if multiple critical obstacles detected
        # Using threshold: > 3 points in critical zone = real obstacle (not noise)
        if critical_count > 3:
            if not self.obstacle_stop:
                self.obstacle_stop = True
                self.get_logger().warn(
                    f'🚨 SAFETY STOP: FRONT obstacle CRITICAL at {min_dist:.2f}m '
                    f'({critical_count} points < {self.critical_zone}m)'
                )
        # Clear stop only if well outside critical zone (hysteresis)
        elif min_dist > self.critical_zone + 0.15:
            if self.obstacle_stop and self.critical_obstacles_rear <= 3:
                self.obstacle_stop = False
                self.get_logger().info(f'✓ FRONT obstacle cleared ({min_dist:.2f}m)')
        
        # Log warnings for warning zone
        if warning_count > 5 and critical_count <= 3:
            if int(self.get_clock().now().nanoseconds / 1e9) % 3 == 0:
                self.get_logger().debug(
                    f'⚠️  WARNING: FRONT obstacles at {min_dist:.2f}m '
                    f'({warning_count} points in warning zone)'
                )
    
    def scan_rear_callback(self, msg):
        """Monitor obstacles from REAR LIDAR (/scan2) using distance classification"""
        min_dist = float('inf')
        critical_count = 0
        warning_count = 0
        
        # Analyze each distance reading
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                min_dist = min(min_dist, distance)
                
                # Classify obstacle by distance (research-based zones)
                if distance < self.critical_zone:
                    critical_count += 1
                elif distance < self.warning_zone:
                    warning_count += 1
        
        self.min_obstacle_distance_rear = min_dist
        self.critical_obstacles_rear = critical_count
        self.warning_obstacles_rear = warning_count
        
        # CRITICAL ZONE: Immediate stop if multiple critical obstacles detected
        # Using threshold: > 3 points in critical zone = real obstacle (not noise)
        if critical_count > 3:
            if not self.obstacle_stop:
                self.obstacle_stop = True
                self.get_logger().warn(
                    f'🚨 SAFETY STOP: REAR obstacle CRITICAL at {min_dist:.2f}m '
                    f'({critical_count} points < {self.critical_zone}m)'
                )
        # Clear stop only if well outside critical zone (hysteresis)
        elif min_dist > self.critical_zone + 0.15:
            if self.obstacle_stop and self.critical_obstacles_front <= 3:
                self.obstacle_stop = False
                self.get_logger().info(f'✓ REAR obstacle cleared ({min_dist:.2f}m)')
        
        # Log warnings for warning zone
        if warning_count > 5 and critical_count <= 3:
            if int(self.get_clock().now().nanoseconds / 1e9) % 3 == 0:
                self.get_logger().debug(
                    f'⚠️  WARNING: REAR obstacles at {min_dist:.2f}m '
                    f'({warning_count} points in warning zone)'
                )
    
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
            front_status = f'F:{self.critical_obstacles_front}crit' if self.critical_obstacles_front > 3 else ''
            rear_status = f'R:{self.critical_obstacles_rear}crit' if self.critical_obstacles_rear > 3 else ''
            obstacle_details = ', '.join(filter(None, [front_status, rear_status]))
            reasons.append(f'OBSTACLE({obstacle_details})')
        
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
            f"  Control Mode: {self.control_mode}",
            f"  Lenient Mode: {'ENABLED' if self.lenient_mode else 'disabled'} (scale: {self.lenient_scale:.2f})",
            f"",
            f"Sensors:",
            f"  Localization: {self.localization_confidence:.1f}% (threshold: {self.confidence_threshold}%)",
            f"",
            f"Obstacle Detection (Research-Based Zones):",
            f"  FRONT Lidar (/scan):",
            f"    Nearest: {self.min_obstacle_distance_front:.2f}m",
            f"    Critical (<{self.critical_zone}m): {self.critical_obstacles_front} points",
            f"    Warning ({self.critical_zone}-{self.warning_zone}m): {self.warning_obstacles_front} points",
            f"  REAR Lidar (/scan2):",
            f"    Nearest: {self.min_obstacle_distance_rear:.2f}m",
            f"    Critical (<{self.critical_zone}m): {self.critical_obstacles_rear} points",
            f"    Warning ({self.critical_zone}-{self.warning_zone}m): {self.warning_obstacles_rear} points"
        ]
        
        response.success = True
        response.message = '\n'.join(status_lines)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
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
