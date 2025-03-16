# Add the following method to your Manipulation class in manipulation.py

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS
from scipy.spatial.transform import Rotation as R
import sys
sys.path.append("/home/locobot/Desktop/ME326_Project/ros/collab_ws/src/locobot_wrapper/scripts")
from arm_control_wrapper import ArmWrapperNode  # Import your arm control wrapper
import time


class Manipulation(Node):
    def __init__(self):
        super().__init__('manipulation')

        # Initialize the ArmWrapperNode as part of this script
        self.arm_wrapper_node = ArmWrapperNode()

        self.base_goal_reach = False

        # Subscribe to the detected object pose topic
        self.object_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',  # Topic from perception node
            self.object_pose_callback,
            10
        )

        self.goal_reached_subscriber = self.create_subscription(
            Bool, 
            "/robot_at_goal", 
            self.base_goal_reach_callback,
            10)

    def base_goal_reach_callback(self, msg):
        self.base_goal_reach = msg.data
        self.get_logger().info(f'{msg.data}')


    def object_pose_callback(self, msg):
        # self.base_goal_reach = True
        if self.base_goal_reach:
            self.get_logger().info(f"Detected Object Pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")
            # Call the ArmWrapperNode to move the arm to the object position and pick it
            self.pick_object(msg)
        else:
            self.get_logger().info(f"Did not reach to the goal yet ...")


    def pick_object(self, msg):
######## TASK 3 #########
        self.arm_wrapper_node.locobot.gripper.release()
        self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x - 0.05, 
                                                                 y = msg.pose.position.y + 0.02, 
                                                                 z = msg.pose.position.z + 0.1,
                                                                 roll = math.pi/2.5, 
                                                                 pitch = math.pi/2.3
                                                                 )
        self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x - 0.05, 
                                                                 y = msg.pose.position.y + 0.02, 
                                                                 z = msg.pose.position.z + 0.02,
                                                                 roll = math.pi/2.5, 
                                                                 pitch = math.pi/2.3
                                                                 )

        self.arm_wrapper_node.locobot.gripper.grasp()

        self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x - 0.1, 
                                                                 y = msg.pose.position.y + 0.02, 
                                                                 z = msg.pose.position.z + 0.2,
                                                                 roll = math.pi/2.5, 
                                                                 pitch = math.pi/2.5
                                                                 )
        
        self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x - 0.1, 
                                                                 y = msg.pose.position.y - 0.2, 
                                                                 z = msg.pose.position.z + 0.2,
                                                                 roll = math.pi/2.5, 
                                                                 pitch = math.pi/2.5
                                                                 )
 
        self.arm_wrapper_node.locobot.gripper.release()
        self.arm_wrapper_node.locobot.arm.go_to_sleep_pose()
        self.arm_wrapper_node.locobot.shutdown()

# ######## TASK 1 #########
#         self.arm_wrapper_node.locobot.gripper.release()
#         self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x + 0.02, 
#                                                                  y = msg.pose.position.y + 0.02, 
#                                                                  z = msg.pose.position.z + 0.1,
#                                                                  roll = math.pi/2.5, 
#                                                                  pitch = math.pi/2.5
#                                                                  )
#         self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x + 0.02, 
#                                                                  y = msg.pose.position.y + 0.02, 
#                                                                  z = msg.pose.position.z + 0.02,
#                                                                  roll = math.pi/2.5, 
#                                                                  pitch = math.pi/2.5
#                                                                  )

#         self.arm_wrapper_node.locobot.gripper.grasp()
#         self.arm_wrapper_node.locobot.arm.go_to_sleep_pose()
#         self.arm_wrapper_node.locobot.shutdown()
# #########
        


def main():
    rclpy.init()
    node = Manipulation()
    
    try:
        rclpy.spin(node)  # Keep spinning to receive object pose and move the arm
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()