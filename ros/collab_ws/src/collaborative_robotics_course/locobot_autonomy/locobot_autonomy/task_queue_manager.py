#!/usr/bin/env python3
"""
Task Queue Manager
-----------------
This node manages a queue of sequential tasks derived from speech commands.
It coordinates the execution of multiple actions to complete complex tasks.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose
import json
import time
import threading

class TaskState:
    """Object to track the current state of the robot and task execution"""
    def __init__(self):
        self.holding_object = False
        self.current_object = None
        self.current_color = None
        self.robot_position = None
        self.target_position = None
        self.destination = None
        self.destination_position = None
        self.task_in_progress = False

class TaskQueueManager(Node):
    """ROS node that manages sequential tasks for the locobot"""
    
    def __init__(self):
        super().__init__('task_queue_manager')
        self.get_logger().info('Starting task queue manager...')
        
        # Task queue and state
        self.task_queue = []
        self.state = TaskState()
        self.lock = threading.Lock()  # Lock for thread-safe queue operations
        
        # Publishers
        self.perception_pub = self.create_publisher(
            String, 
            '/perception/activate', 
            10
        )
        
        self.goal_pose_pub = self.create_publisher(
            Pose,
            '/goal_pose',
            10
        )
        
        self.task_status_pub = self.create_publisher(
            String,
            '/task/status',
            10
        )
        
        self.task_complete_pub = self.create_publisher(
            Bool,
            '/task/complete',
            10
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, 
            '/speech/parsed_command', 
            self.command_callback, 
            10
        )
        
        self.object_pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self.object_pose_callback,
            10
        )
        
        self.robot_at_goal_sub = self.create_subscription(
            Bool,
            '/robot_at_goal',
            self.robot_at_goal_callback,
            10
        )
        
        self.task_execution_timer = self.create_timer(0.5, self.execute_next_task)
        
        self.get_logger().info('Task queue manager initialized')
    
    def command_callback(self, msg):
        """
        Process incoming speech commands and break them into sequential tasks
        
        Args:
            msg: String message containing JSON-formatted command
        """
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f'Received command: {command}')
            
            task_type = command.get('task_type')
            obj = command.get('object')
            color = command.get('color')
            destination = command.get('destination')
            
            if not task_type:
                self.get_logger().warn('Received command without task type')
                return
            
            with self.lock:
                # Clear any existing tasks
                self.task_queue.clear()
                
                # Reset task state
                self.state = TaskState()
                self.state.current_object = obj
                self.state.current_color = color
                self.state.destination = destination
                
                # Parse different kinds of tasks
                if task_type == 'retrieve':
                    # For retrieval: find -> navigate -> pick -> return
                    self.task_queue.append({
                        'action': 'find',
                        'object': obj,
                        'color': color
                    })
                    
                    self.task_queue.append({
                        'action': 'navigate',
                        'target': 'object'
                    })
                    
                    self.task_queue.append({
                        'action': 'pick',
                        'object': obj
                    })
                    
                    self.task_queue.append({
                        'action': 'navigate',
                        'target': 'home'
                    })
                
                elif task_type == 'sequential':
                    # For sequential: find -> navigate -> pick -> find destination -> navigate -> place
                    self.task_queue.append({
                        'action': 'find',
                        'object': obj,
                        'color': color
                    })
                    
                    self.task_queue.append({
                        'action': 'navigate',
                        'target': 'object'
                    })
                    
                    self.task_queue.append({
                        'action': 'pick',
                        'object': obj
                    })
                    
                    if destination:
                        self.task_queue.append({
                            'action': 'find',
                            'object': destination
                        })
                        
                        self.task_queue.append({
                            'action': 'navigate',
                            'target': 'destination'
                        })
                    
                    self.task_queue.append({
                        'action': 'place'
                    })
                
                elif task_type == 'sort':
                    # For sorting: perform color detection and grouping logic
                    self.task_queue.append({
                        'action': 'sort_objects',
                    })
                
                self.get_logger().info(f'Task queue created with {len(self.task_queue)} actions')
                self.publish_status(f'Created task queue with {len(self.task_queue)} actions')
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
    
    def object_pose_callback(self, msg):
        """
        Handle detected object pose from perception
        
        Args:
            msg: PoseStamped with object position
        """
        if not self.state.task_in_progress:
            return
            
        self.get_logger().info(f'Received object position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        
        # Check which object we're currently looking for
        current_task = None
        with self.lock:
            if self.task_queue:
                current_task = self.task_queue[0]
        
        if current_task and current_task['action'] == 'find':
            # If we're looking for the primary object
            if current_task.get('object') == self.state.current_object:
                self.state.target_position = msg.pose
            # If we're looking for the destination object
            elif current_task.get('object') == self.state.destination:
                self.state.destination_position = msg.pose
            
            # Mark the find task as completed
            with self.lock:
                if self.task_queue and self.task_queue[0]['action'] == 'find':
                    self.get_logger().info(f"Object found, moving to next task")
                    self.task_queue.pop(0)  # Remove the find task
    
    def robot_at_goal_callback(self, msg):
        """
        Handle navigation completion notification
        
        Args:
            msg: Boolean indicating if robot reached goal
        """
        if not msg.data or not self.state.task_in_progress:
            return
            
        self.get_logger().info('Robot has reached the goal position')
        
        # If the current task is 'navigate', mark it as completed
        with self.lock:
            if self.task_queue and self.task_queue[0]['action'] == 'navigate':
                self.get_logger().info(f"Navigation complete, moving to next task")
                self.task_queue.pop(0)  # Remove the navigate task
    
    def execute_next_task(self):
        """
        Timer callback to execute the next task in the queue
        """
        with self.lock:
            if not self.task_queue:
                if self.state.task_in_progress:
                    # All tasks completed
                    self.state.task_in_progress = False
                    self.publish_status("All tasks completed")
                    
                    complete_msg = Bool()
                    complete_msg.data = True
                    self.task_complete_pub.publish(complete_msg)
                return
            
            # Set task in progress flag
            self.state.task_in_progress = True
            
            # Get the next task
            current_task = self.task_queue[0]
            action = current_task['action']
            
            # Execute the appropriate action
            if action == 'find':
                self.execute_find_task(current_task)
            elif action == 'navigate':
                self.execute_navigate_task(current_task)
            elif action == 'pick':
                self.execute_pick_task(current_task)
            elif action == 'place':
                self.execute_place_task(current_task)
            elif action == 'sort_objects':
                self.execute_sort_task(current_task)
    
    def execute_find_task(self, task):
        """
        Execute a find task to locate an object
        
        Args:
            task: Dictionary with task parameters
        """
        object_name = task.get('object')
        color = task.get('color')
        
        # Determine if we're looking for the primary object or destination
        is_destination = (object_name == self.state.destination)
        has_position = (self.state.destination_position is not None) if is_destination else (self.state.target_position is not None)
        
        # Only send the perception request if we haven't received a position yet
        if not has_position:
            search_target = object_name
            if color and not is_destination:
                search_target = f"{color} {object_name}"
            
            self.publish_status(f"Searching for {search_target}")
            
            perceive_msg = String()
            perceive_msg.data = search_target
            self.perception_pub.publish(perceive_msg)
            self.get_logger().info(f"Requested perception to find: {search_target}")
    
    def execute_navigate_task(self, task):
        """
        Execute a navigation task to move the robot
        
        Args:
            task: Dictionary with task parameters
        """
        target_type = task.get('target')
        
        if target_type == 'object' and self.state.target_position:
            self.publish_status(f"Navigating to {self.state.current_object}")
            
            # Create a goal pose for the navigation
            goal_pose = Pose()
            goal_pose.position.x = self.state.target_position.position.x
            goal_pose.position.y = self.state.target_position.position.y
            goal_pose.position.z = 0.0  # Keep z at ground level for navigation
            
            # Set orientation (robot facing the object)
            goal_pose.orientation.w = 1.0
            
            self.goal_pose_pub.publish(goal_pose)
            self.get_logger().info(f"Published goal pose to navigate to object")
            
        elif target_type == 'destination' and self.state.destination_position:
            self.publish_status(f"Navigating to {self.state.destination}")
            
            # Use the detected destination position
            goal_pose = Pose()
            goal_pose.position.x = self.state.destination_position.position.x
            goal_pose.position.y = self.state.destination_position.position.y
            goal_pose.position.z = 0.0
            goal_pose.orientation.w = 1.0
            
            self.goal_pose_pub.publish(goal_pose)
            self.get_logger().info(f"Published goal pose to navigate to destination")
            
        elif target_type == 'home':
            self.publish_status("Returning home")
            
            # Navigate back to home position
            goal_pose = Pose()
            goal_pose.position.x = 0.0
            goal_pose.position.y = 0.0
            goal_pose.position.z = 0.0
            goal_pose.orientation.w = 1.0
            
            self.goal_pose_pub.publish(goal_pose)
            self.get_logger().info("Published goal pose to return home")
    
    def execute_pick_task(self, task):
        """
        Execute a pick task to grasp an object
        
        Args:
            task: Dictionary with task parameters
        """
        self.publish_status(f"Picking up {self.state.current_object}")
        
        # In a full implementation, you would send commands to the manipulation node
        # For now, we'll simulate success after a delay
        time.sleep(2.0)
        
        # Update state
        self.state.holding_object = True
        
        # Task completed
        with self.lock:
            self.task_queue.pop(0)
        
        self.get_logger().info(f"Picked up {self.state.current_object}")
    
    def execute_place_task(self, task):
        """
        Execute a place task to release an object
        
        Args:
            task: Dictionary with task parameters
        """
        if self.state.holding_object:
            self.publish_status("Placing object")
            
            # In a full implementation, you would send commands to the manipulation node
            # For now, we'll simulate success after a delay
            time.sleep(2.0)
            
            # Update state
            self.state.holding_object = False
            
            # Task completed
            with self.lock:
                self.task_queue.pop(0)
            
            self.get_logger().info("Placed object")
        else:
            self.get_logger().warn("Tried to place object, but not holding anything")
            # Skip this task
            with self.lock:
                self.task_queue.pop(0)
    
    def execute_sort_task(self, task):
        """
        Execute a sorting task to group objects by color
        
        Args:
            task: Dictionary with task parameters
        """
        self.publish_status("Sorting objects by color")
        
        # In a real implementation, this would involve a more complex sequence
        # of perception, navigation, and manipulation
        # For this demo, we'll just complete the task after a delay
        time.sleep(2.0)
        
        # Task completed
        with self.lock:
            self.task_queue.pop(0)
        
        self.get_logger().info("Completed color sorting task")
    
    def publish_status(self, status_text):
        """
        Publish status updates about the current task
        
        Args:
            status_text: Status message to publish
        """
        status_msg = String()
        status_msg.data = status_text
        self.task_status_pub.publish(status_msg)
        self.get_logger().info(status_text)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TaskQueueManager()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in task queue manager: {str(e)}')
        import traceback
        print(traceback.format_exc())
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()