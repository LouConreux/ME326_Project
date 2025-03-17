#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import NavigateToPosition
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class MoveBaseActionClient(Node):
    def __init__(self):
        super().__init__('move_base_action_client')
        self._client = ActionClient(self, NavigateToPosition, '/locobot/mobile_base/navigate_to_position')
        
    def send_goal(self):
        goal = NavigateToPosition.Goal()
        goal.goal_pose.pose.position = Point(x=1.0, y=0.0, z=0.0)  # 0.1 meters forward
        goal.goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # No rotation
        goal.achieve_goal_heading = False  # Don't rotate, just move forward

        self.get_logger().info('Sending goal...')
        self._client.wait_for_server()
        # future = self._client.send_goal_async(goal)
        # future.add_done_callback(self.goal_response_callback)

    # def goal_response_callback(self, future):
    #     result = future.result()
    #     if result:
    #         self.get_logger().info('Goal succeeded!')
    #     else:
    #         self.get_logger().warn('Goal failed.')
        
    #     # After the goal finishes, shutdown ROS
    #     self.destroy_node()
    #     rclpy.shutdown()

def main():
    rclpy.init()
    action_client = MoveBaseActionClient()
    action_client.send_goal()

    # Spin the node so the action client can execute the goal
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
