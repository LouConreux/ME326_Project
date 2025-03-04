import math

from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS

"""
This script makes the end-effector perform pick, pour, and place tasks.

To get started, open a terminal and run the command:

    ros2 launch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s

Then change to this directory and run the command:

    python3 bartender.py

...or run the command:

    ros2 run interbotix_xslocobot_control bartender.py
"""
class Manipulation(Node):
    def __init__(self):
        super().__init__('manipulation')


        self.publisher = self.create_publisher(
            PoseStamped, 
            '/arm_pose', 
            10
        )

        # Subscribe to goal reached topic
        self.subscription = self.create_subscription(
            Bool,
            '/robot_at_goal',
            self.goal_reached_callback,
            10
        )

def main():
    locobot = InterbotixLocobotXS(
        robot_model='locobot_wx250s',
        robot_name='locobot',
        arm_model='mobile_wx250s'
        )

    locobot.arm.set_ee_pose_components(x=0.3, z=0.2)
    #locobot.arm.set_single_joint_position('waist', math.pi/4.0)
    locobot.gripper.release()
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.28)
    locobot.gripper.grasp()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    # locobot.arm.set_single_joint_position('waist', -math.pi/4.0)
    # locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    # locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # locobot.arm.set_single_joint_position('waist', math.pi/4.0)
    # locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    # locobot.gripper.release()
    # locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    # locobot.arm.go_to_home_pose()
    # locobot.arm.go_to_sleep_pose()

    locobot.shutdown()


if __name__ == '__main__':
    main()





 import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.publisher = self.create_publisher(PoseStamped, '/arm_pose', 10)
        time.sleep(1)  # Give ROS time to establish connections

    def move_arm(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Change if needed
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent Pose: {msg.pose}")

def main():
    rclpy.init()
    node = ArmMover()
    node.move_arm(0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 1.0)  # Example pose
    time.sleep(2)  # Ensure message is sent before shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
