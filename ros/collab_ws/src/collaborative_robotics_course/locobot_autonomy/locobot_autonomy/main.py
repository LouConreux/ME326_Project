#!/usr/bin/env python3
"""
Main Coordinator Node
-------------------
This node coordinates the robot's behavior based on speech commands.
It manages speech recording and monitors task completion.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

class MainCoordinator(Node):
    """ROS node that handles speech recording and monitors task status."""
    
    def __init__(self):
        super().__init__('main_coordinator')
        self.get_logger().info('Starting main coordinator node...')
        
        # Publishers
        self.record_pub = self.create_publisher(
            Bool, 
            '/speech/start_recording', 
            10
        )
        
        # Subscribers
        self.task_status_sub = self.create_subscription(
            String, 
            '/task/status', 
            self.task_status_callback, 
            10
        )
        
        self.task_complete_sub = self.create_subscription(
            Bool, 
            '/task/complete', 
            self.task_complete_callback, 
            10
        )
        
        # Timer to start listening for commands initially
        self.timer = self.create_timer(2.0, self.start_listening)
        
        self.get_logger().info('Main coordinator initialized and ready')
    
    def start_listening(self):
        """Trigger speech recognition to begin listening."""
        # Cancel the timer so this only runs once on startup
        self.timer.cancel()
        
        # Publish message to start recording
        msg = Bool()
        msg.data = True
        self.record_pub.publish(msg)
        self.get_logger().info('Requested start of speech recording')
    
    def task_status_callback(self, msg):
        """
        Handle status updates from task execution.
        
        Args:
            msg: String message with status information
        """
        self.get_logger().info(f"Task status: {msg.data}")
    
    def task_complete_callback(self, msg):
        """
        Handle task completion notification.
        
        Args:
            msg: Boolean message indicating if task completed successfully
        """
        if msg.data:
            self.get_logger().info("Task completed successfully")
        else:
            self.get_logger().warn("Task failed or was cancelled")
        
        # Wait a moment, then start listening again
        time.sleep(2.0)
        record_msg = Bool()
        record_msg.data = True
        self.record_pub.publish(record_msg)
        self.get_logger().info("Listening for next command")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MainCoordinator()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main coordinator: {str(e)}')
        import traceback
        print(traceback.format_exc())
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()