#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')
        
        # Publishers to lock/unlock twist_mux topics
        self.lock_pubs = {
            'navigation': self.create_publisher(Bool, '/twist_mux/locks/navigation', 10),
            'tracker': self.create_publisher(Bool, '/twist_mux/locks/tracker', 10),
            'joystick': self.create_publisher(Bool, '/twist_mux/locks/joystick', 10)
        }
        
        # Subscriber to mode commands from web UI or other sources
        self.mode_sub = self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        
        # Current active mode
        self.current_mode = 'manual'
        
        # Start in manual mode
        self.activate_mode('zones')
        
        self.get_logger().info('Mode Switcher started - Default: ZONES')
        self.get_logger().info('Available modes: manual, zones, sequence, path')
        self.get_logger().info('Publish to /control_mode: ros2 topic pub -1 /control_mode std_msgs/msg/String "{data: manual}"')
    
    def activate_mode(self, mode: str):
        """Activate a mode by unlocking it and locking all others"""
        mode_map = {
            'manual': 'joystick',      # Joystick control
            'zones': 'navigation',     # Saved zones navigation
            'sequence': 'tracker',     # Sequence following
            'path': 'navigation'       # Path following
        }
        
        if mode not in mode_map:
            self.get_logger().warn(f'Unknown mode: {mode}')
            return
        
        active_topic = mode_map[mode]
        
        # Lock all topics except the active one (safety is never locked)
        for topic_name, pub in self.lock_pubs.items():
            msg = Bool()
            if topic_name == active_topic:
                msg.data = False  # Unlock active mode
            else:
                msg.data = True   # Lock inactive modes
            pub.publish(msg)
        
        mode_icons = {
            'manual': 'MANUAL',
            'zones': 'ZONES',
            'sequence': 'SEQUENCE',
            'path': 'PATH'
        }
        
        self.get_logger().info(f'{mode_icons.get(mode, "🤖")} MODE ACTIVE - {active_topic} unlocked, others locked')
        self.current_mode = mode
    
    def mode_callback(self, msg):
        """Switch mode when receiving command"""
        mode = msg.data.lower()
        if mode != self.current_mode:
            self.activate_mode(mode)

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
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
