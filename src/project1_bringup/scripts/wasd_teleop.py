#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        
        # Publisher
        self.publisher = self.create_publisher(
            TwistStamped,
            '/bicycle_steering_controller/reference',
            10
        )
        
        # Parameters
        self.linear_speed = 1.5
        self.angular_speed = 0.67 # Limited to ±0.5
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_twist)
        
        self.get_logger().info('WASD Teleop Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W - Forward')
        self.get_logger().info('  S - Backward')
        self.get_logger().info('  A - Turn Left')
        self.get_logger().info('  D - Turn Right')
        self.get_logger().info('  Space - Stop')
        self.get_logger().info('  Q - Quit')
        self.get_logger().info(f'Max linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Max angular speed: ±{self.angular_speed} rad/s')
        
    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.current_linear
        msg.twist.angular.z = self.current_angular
        self.publisher.publish(msg)
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w' or key == 'W':
                    self.current_linear = self.linear_speed
                    self.get_logger().info(f'Forward: {self.current_linear} m/s')
                    
                elif key == 's' or key == 'S':
                    self.current_linear = -self.linear_speed
                    self.get_logger().info(f'Backward: {self.current_linear} m/s')
                    
                elif key == 'a' or key == 'A':
                    self.current_angular = self.angular_speed
                    self.get_logger().info(f'Turn Left: {self.current_angular} rad/s')
                    
                elif key == 'd' or key == 'D':
                    self.current_angular = -self.angular_speed
                    self.get_logger().info(f'Turn Right: {self.current_angular} rad/s')
                    
                elif key == ' ':
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                    self.get_logger().info('Stop')
                    
                elif key == 'q' or key == 'Q':
                    self.get_logger().info('Quitting...')
                    break
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.publish_twist()

def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()