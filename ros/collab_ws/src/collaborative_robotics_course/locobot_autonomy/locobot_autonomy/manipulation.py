# import math
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Bool
# from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS
# from scipy.spatial.transform import Rotation as R
# from arm_control_wrapper import ArmWrapperNode  # Import your arm control wrapper
# import time


# class Manipulation(Node):
#     def __init__(self):
#         super().__init__('maniplation')

#         locobot = InterbotixLocobotXS(
#             robot_model='locobot_wx250s',
#             robot_name='locobot',
#             arm_model='mobile_wx250s'
#         )

#         # Initialize the ArmWrapperNode as part of this script
#         self.arm_wrapper_node = ArmWrapperNode()

#         # Subscribe to the detected object pose topic
#         self.object_pose_subscriber = self.create_subscription(
#             PoseStamped,
#             '/detected_object_pose',  # Topic from perception node
#             self.object_pose_callback,
#             10
#         )

#     def object_pose_callback(self, msg):
#         self.get_logger().info(f"Detected Object Pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

#         # Call the ArmWrapperNode to move the arm to the object position
#         self.move_arm_to_object(msg)

#     def move_arm_to_object(self, msg):
#         # Use ArmWrapperNode to move the arm to the detected object's pose
#         # We pass the position and orientation of the object
#         self.arm_wrapper_node.pose_callback(msg)  # This will handle both simulation or hardware motion

#         # Log that we're moving the arm
#         self.get_logger().info(f"Moving arm to object pose at: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

# def main():
#     rclpy.init()
#     node = Manipulation()
    
#     try:
#         rclpy.spin(node)  # Keep spinning to receive object pose and move the arm
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()



import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS
from scipy.spatial.transform import Rotation as R
from arm_control_wrapper import ArmWrapperNode  # Import your arm control wrapper
import time


class Manipulation(Node):
    def __init__(self):
        super().__init__('maniplation')

        self.locobot = InterbotixLocobotXS(
            robot_model='locobot_wx250s',
            robot_name='locobot',
            arm_model='mobile_wx250s'
        )

        # Initialize the ArmWrapperNode as part of this script
        self.arm_wrapper_node = ArmWrapperNode()

        # Subscribe to the detected object pose topic
        self.object_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',  # Topic from perception node
            self.object_pose_callback,
            10
        )

    def object_pose_callback(self, msg):
        self.get_logger().info(f"Detected Object Pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

        # Call the ArmWrapperNode to move the arm to the object position and pick it
        self.pick_object(msg)

    def pick_object(self, msg):
        # Move arm to object pose
        self.arm_wrapper_node.pose_callback(msg)  # This will handle both simulation or hardware motion
        self.get_logger().info(f"Moving arm to object pose at: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

        # Close the gripper to pick up the object
        self.get_logger().info("Closing gripper to pick the object.")
        self.arm_wrapper_node.gripper_callback(Bool(data=False))  # Assuming closing gripper means False

        # Lift the arm slightly to avoid ground obstacles
        self.lift_arm()

        # Optionally, rotate the arm if necessary (e.g., to adjust orientation)
        self.rotate_arm()

        # Lower the arm to drop the object
        self.lower_arm()

        # Open the gripper to release the object
        self.release_object()

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




