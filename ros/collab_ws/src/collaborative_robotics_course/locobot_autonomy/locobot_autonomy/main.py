#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import time

class MainCoordinator(Node):
    def __init__(self):
        super().__init__('main_coordinator')
        
        # State tracking
        self.current_task = None
        self.task_complete = False
        
        # Publishers
        self.listen_pub = self.create_publisher(Bool, '/speech/start_listening', 10)
        self.perception_pub = self.create_publisher(String, '/perception/activate', 10)
        self.task_pub = self.create_publisher(String, '/task/execute', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(String, '/speech/parsed_command', 
                                                   self.command_callback, 10)
        self.target_sub = self.create_subscription(String, '/perception/target_location', 
                                                  self.target_callback, 10)
        self.task_status_sub = self.create_subscription(String, '/task/status', 
                                                       self.task_status_callback, 10)
        self.task_complete_sub = self.create_subscription(Bool, '/task/complete', 
                                                         self.task_complete_callback, 10)
        
        # Start the system by listening for commands
        self.timer = self.create_timer(2.0, self.start_listening)
        
    def start_listening(self):
        # One-time initialization trigger
        self.timer.cancel()
        
        msg = Bool()
        msg.data = True
        self.listen_pub.publish(msg)
        self.get_logger().info('Started listening for commands')
        
    def command_callback(self, msg):
        # Parse the JSON command
        command = json.loads(msg.data)
        self.get_logger().info(f'Received command: {command}')
        
        self.current_task = command
        self.task_complete = False
        
        # Send request to perception system
        if command['object']:
            perceive_msg = String()
            perceive_msg.data = command['object']
            if command['color']:
                perceive_msg.data = f"{command['color']} {command['object']}"
            
            self.perception_pub.publish(perceive_msg)
            self.get_logger().info(f"Requested perception to find: {perceive_msg.data}")
        elif command['task_type'] == 'sort':
            # For sorting tasks, we might have a different workflow
            task_msg = String()
            task_msg.data = json.dumps({"task_type": "sort", "parameters": {}})
            self.task_pub.publish(task_msg)
    
    def target_callback(self, msg):
        # After perception returns target location, execute the task
        if self.current_task and not self.task_complete:
            task_msg = String()
            task_msg.data = json.dumps({
                "task_type": self.current_task['task_type'],
                "object": self.current_task['object'],
                "color": self.current_task['color'],
                "destination": self.current_task['destination'],
                "target_location": msg.data  # This would be the pose from perception
            })
            self.task_pub.publish(task_msg)
            self.get_logger().info(f"Sent task execution command: {self.current_task['task_type']}")
    
    def task_status_callback(self, msg):
        self.get_logger().info(f"Task status: {msg.data}")
    
    def task_complete_callback(self, msg):
        if msg.data:
            self.task_complete = True
            self.get_logger().info("Task completed successfully")
            # Wait a moment, then start listening again
            time.sleep(2.0)
            listen_msg = Bool()
            listen_msg.data = True
            self.listen_pub.publish(listen_msg)

def main():
    rclpy.init()
    node = MainCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()