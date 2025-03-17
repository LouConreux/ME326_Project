#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/locobot/diffdrive_controller/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish every 0.1 seconds
        self.get_logger().info('CmdVelPublisher node has started')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  # Set linear velocity (forward)
        msg.angular.z = 0.5  # Set angular velocity (turn)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
