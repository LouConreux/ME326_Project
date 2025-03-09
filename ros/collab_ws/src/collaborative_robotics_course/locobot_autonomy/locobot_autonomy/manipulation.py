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
######## TASK 2 #########
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
        
        # self.arm_wrapper_node.locobot.arm.set_single_joint_position('waist', math.pi/4.0)

        self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x - 0.1, 
                                                                 y = msg.pose.position.y - 0.2, 
                                                                 z = msg.pose.position.z + 0.2,
                                                                 roll = math.pi/2.5, 
                                                                 pitch = math.pi/2.5
                                                                 )
        # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x - 0.1, 
        #                                                          y = msg.pose.position.y - 0.2, 
        #                                                          z = msg.pose.position.z + 0.1,
        #                                                          roll = math.pi/2.5, 
        #                                                          pitch = math.pi/2.5
        #                                                          )
        self.arm_wrapper_node.locobot.gripper.release()
        self.arm_wrapper_node.locobot.arm.go_to_sleep_pose()
        self.arm_wrapper_node.locobot.shutdown()
#########
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
        # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = 0.3, z = 0.2)

        # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = 0.3, z = 0.2, roll = 2.0)
        # self.arm_wrapper_node.locobot.gripper.release()
        # self.arm_wrapper_node.locobot.gripper.grasp()
        # self.arm_wrapper_node.locobot.gripper.release()

        # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x + 0.02, 
        #                                                          y = msg.pose.position.y + 0.02, 
        #                                                          z = msg.pose.position.z + 0.1,
        #                                                          roll = math.pi/2.5, 
        #                                                          pitch = math.pi/2.5
        #                                                          )
        
        # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = msg.pose.position.x + 0.02, 
        #                                                          y = msg.pose.position.y + 0.02, 
        #                                                          z = msg.pose.position.z + 0.02,
        #                                                          roll = math.pi/2.5, 
        #                                                          pitch = math.pi/2.5
        #                                                          )

        # self.arm_wrapper_node.locobot.gripper.grasp()


        # # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x = 0.3, z = 0.2, yaw = 2.0)
        # # self.arm_wrapper_node.locobot.gripper.release()
        # # self.arm_wrapper_node.locobot.gripper.grasp()

        # self.arm_wrapper_node.locobot.arm.go_to_sleep_pose()

        # self.arm_wrapper_node.locobot.shutdown()


        # self.arm_wrapper_node.locobot.arm.set_ee_pose_components(x=0.3, z=0.2)
        # self.arm_wrapper_node.locobot.arm.set_single_joint_position('waist', math.pi/4.0)
        # self.arm_wrapper_node.locobot.gripper.release()

        # self.arm_wrapper_node.locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
        # self.arm_wrapper_node.locobot.gripper.grasp()
        # self.arm_wrapper_node.locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
        # self.arm_wrapper_node.locobot.arm.set_single_joint_position('waist', -math.pi/4.0)
        # self.arm_wrapper_node.locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)
        # self.arm_wrapper_node.locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
        # self.arm_wrapper_node.locobot.arm.set_single_joint_position('waist', math.pi/4.0)
        # self.arm_wrapper_node.locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
        # self.arm_wrapper_node.locobot.gripper.release()
        # self.arm_wrapper_node.locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
        # self.arm_wrapper_node.locobot.arm.go_to_home_pose()
        # self.arm_wrapper_node.locobot.arm.go_to_sleep_pose()
        # Open gripper
        # self.arm_wrapper_node.gripper_callback(Bool(data=True))  # Assuming closing gripper means False

        # # Move arm to object pose
        # self.arm_wrapper_node.pose_callback(msg)  # This will handle both simulation or hardware motion
        # self.get_logger().info(f"Moving arm to object pose at: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

        # # Close the gripper to pick up the object
        # self.get_logger().info("Closing gripper to pick the object.")
        # self.arm_wrapper_node.gripper_callback(Bool(data=False))  # Assuming closing gripper means False

        # # Lift the arm slightly to avoid ground obstacles
        # self.get_logger().info(f"Lifting arm to object pose at: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z + 0.4}")
        # # self.lift_arm()

        # # Optionally, rotate the arm if necessary (e.g., to adjust orientation)
        # self.get_logger().info(f"Moving arm to object pose at: x={msg.pose.position.x}, y={msg.pose.position.y + 0.2}, z={msg.pose.position.z + 0.4}")
        # # self.rotate_arm()

        # # # Lower the arm to drop the object
        # self.get_logger().info(f"Lowering arm to a position at: x={msg.pose.position.x}, y={msg.pose.position.y + 0.2}, z={msg.pose.position.z}")
        # # self.lower_arm()

        # # Open the gripper to release the object
        # # self.release_object()
        # self.arm_wrapper_node.gripper_callback(Bool(data=True))  # Assuming closing gripper means False


    def lift_arm(self):
        # Move the arm upwards to avoid obstacles (e.g., lifting 10cm)
        self.get_logger().info("Lifting the arm to avoid obstacles.")
        # Update the pose by adding a small amount to the Z-axis (adjust this value as needed)
        lifted_pose = PoseStamped()
        lifted_pose.pose = self.arm_wrapper_node.get_current_pose()
        lifted_pose.pose.position.z += 0.1  # Lift 10 cm
        self.arm_wrapper_node.pose_callback(lifted_pose)

    def rotate_arm(self):
        # Rotate the arm if needed (e.g., rotating 90 degrees around the Z-axis)
        self.get_logger().info("Rotating the arm.")
        current_pose = self.arm_wrapper_node.get_current_pose()
        rotation = R.from_euler('z', 90, degrees=True)  # Rotate 90 degrees
        new_orientation = rotation.apply([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z])
        current_pose.pose.orientation.x = new_orientation[0]
        current_pose.pose.orientation.y = new_orientation[1]
        current_pose.pose.orientation.z = new_orientation[2]
        self.arm_wrapper_node.pose_callback(current_pose)

    def lower_arm(self):
        # Lower the arm to drop the object
        self.get_logger().info("Lowering the arm to drop the object.")
        lowered_pose = PoseStamped()
        lowered_pose.pose = self.arm_wrapper_node.get_current_pose()
        lowered_pose.pose.position.z -= 0.1  # Lower 10 cm (adjust as needed)
        self.arm_wrapper_node.pose_callback(lowered_pose)

    def release_object(self):
        # Open the gripper to release the object
        self.get_logger().info("Releasing the object by opening the gripper.")
        self.arm_wrapper_node.gripper_callback(Bool(data=True))  # Assuming opening gripper means True
    
    # def pick_and_place(self, object_pose):
    #     """
    #     Execute a complete pick and place operation
        
    #     Args:
    #         object_pose: PoseStamped containing object position
    #     """
    #     # First move to the pre-grasp position
    #     self.get_logger().info(f"Moving to pre-grasp position")
    #     # Implementation depends on your robot's arm control
        
    #     # Then grasp the object
    #     self.get_logger().info(f"Closing gripper")
    #     # Implementation to close gripper
        
    #     # Lift the object
    #     self.get_logger().info(f"Lifting object")
    #     # Implementation to lift arm
        
    #     # Move to destination (this would come from the task manager)
    #     self.get_logger().info(f"Moving to destination")
    #     # Implementation to move arm to destination
        
    #     # Place the object
    #     self.get_logger().info(f"Releasing object")
    #     # Implementation to open gripper
        
    #     # Return to home position
    #     self.get_logger().info(f"Returning to home position")
    #     # Implementation to move arm to home


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