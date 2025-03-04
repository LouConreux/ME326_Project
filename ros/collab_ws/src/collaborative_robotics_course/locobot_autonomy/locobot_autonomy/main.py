#!/usr/bin/env python3
"""
Main Coordinator Node
-------------------
This node coordinates the robot's behavior based on speech commands.
It manages the interactions between speech processing, perception, and task execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import json
import time

class MainCoordinator(Node):
    """ROS node that coordinates tasks and actions based on speech commands."""
    
    def __init__(self):
        super().__init__('main_coordinator')
        self.get_logger().info('Starting main coordinator node...')
        
        # Task state tracking
        self.current_task = None
        self.target_object = None
        self.target_color = None
        self.destination = None
        self.task_in_progress = False
        
        # Publishers
        self.record_pub = self.create_publisher(
            Bool, 
            '/speech/start_recording', 
            10
        )
        
        self.perception_pub = self.create_publisher(
            String, 
            '/perception/activate', 
            10
        )
        
        self.task_pub = self.create_publisher(
            String, 
            '/task/execute', 
            10
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, 
            '/speech/parsed_command', 
            self.command_callback, 
            10
        )
        
        self.target_loc_sub = self.create_subscription(
            PoseStamped, 
            '/perception/target_location', 
            self.target_location_callback, 
            10
        )
        
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
    
    def command_callback(self, msg):
        """
        Process the parsed command from speech recognition.
        
        Args:
            msg: String message containing JSON-formatted command details
        """
        try:
            # Parse the JSON command
            command = json.loads(msg.data)
            self.get_logger().info(f'Received command: {command}')
            
            # Store the current task parameters
            self.current_task = command.get('task_type')
            self.target_object = command.get('object')
            self.target_color = command.get('color')
            self.destination = command.get('destination')
            
            if not self.current_task:
                self.get_logger().warn('Received command without task type')
                return
            
            # Don't start a new task if one is already in progress
            if self.task_in_progress:
                self.get_logger().info('Task already in progress, ignoring new command')
                return
            
            self.task_in_progress = True
            
            # For object-related tasks, activate perception
            if self.target_object:
                perceive_msg = String()
                search_target = self.target_object
                if self.target_color:
                    search_target = f"{self.target_color} {self.target_object}"
                
                perceive_msg.data = search_target
                self.perception_pub.publish(perceive_msg)
                self.get_logger().info(f"Requested perception to find: {search_target}")
            
            # For color sorting tasks, no need to find a specific object
            elif self.current_task == 'sort':
                task_msg = String()
                task_msg.data = json.dumps({
                    "task_type": "sort",
                    "parameters": {}
                })
                self.task_pub.publish(task_msg)
                self.get_logger().info("Requested color sorting task")
            
            # If something went wrong, reset task state
            else:
                self.get_logger().warn("Couldn't determine what to do with the command")
                self.task_in_progress = False
        
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            self.task_in_progress = False
    
    def target_location_callback(self, msg):
        """
        Handle the detected target location from perception.
        
        Args:
            msg: PoseStamped message with the position of the detected object
        """
        if not self.task_in_progress or not self.current_task:
            return
        
        self.get_logger().info(f"Received target location: {msg.pose.position}")
        
        # Create task execution message
        task_msg = String()
        task_params = {
            "task_type": self.current_task,
            "object": self.target_object,
            "color": self.target_color,
            "destination": self.destination,
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z
            }
        }
        
        task_msg.data = json.dumps(task_params)
        self.task_pub.publish(task_msg)
        self.get_logger().info(f"Sent task execution command for {self.current_task}")
    
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
        
        # Reset task state
        self.task_in_progress = False
        self.current_task = None
        self.target_object = None
        self.target_color = None
        self.destination = None
        
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