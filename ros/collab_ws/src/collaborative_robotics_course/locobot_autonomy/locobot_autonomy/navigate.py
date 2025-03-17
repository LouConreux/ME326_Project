#!/usr/bin/env python3
'''

This script controls the robot to go from the origin (0, 0, 0) to the published target point.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data 

import numpy as np

class Navigation(Node):
    """Class for example operations for locobot control
    Point A: the origin (x,y,z) = (0,0,0)
    Point B: the point (x,y,z) that is published
    """
    def __init__(self, target_pose=None):
        """
        Input: Target_pose - ROS geometry_msgs.msg.Pose type
        """
        super().__init__('navigate')
        

        #set the initial target pose to no value, and have the boolean demonstrate that the target position is not reached
        self.target_pose_reached_bool = False
        self.target_pose = None
        self.max_linear_velocity = 0.2 # m/s
        self.max_angular_velocity = 0.2 # rad/s
        
        # Define the publishers, including "Twist" for moving the base, and "Bool" for determining if the robot is at the goal pose
        self.mobile_base_vel_publisher = self.create_publisher(
            Twist, 
            "/locobot/diffdrive_controller/cmd_vel_unstamped",
            1)
        
        self.goal_reached_publisher = self.create_publisher(
            Bool, 
            "/robot_at_goal", 
            10)

        # Define the subscribers, including "Pose" to get the goal pose, and "Odometry" to get the instantaneous pose of the robot
        self.pose_sub = self.create_subscription(
            Pose,
            '/goal_pose',  # topic
            self.position_callback,
            qos_profile=qos_profile_sensor_data  # QoS depth
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/locobot/sim_ground_truth_pose",   #topic
            self.odom_mobile_base_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.L = 0.1 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        #set targets for when a goal is reached: 
        self.goal_reached_error = 0.05
        self.integrated_error = np.matrix([[0],[0]]) #this is the integrated error for Proportional, Integral (PI) control
        self.integrated_error_list = []
        self.length_of_integrated_error_list = 20
        self.t_init = self.get_clock().now()
        self.prev_err = np.array([[0],[0]])


    def position_callback(self, msg):
        # get the target pose based on the Pose data
        self.target_pose = msg
        self.get_logger().info(f'Position -> x: {msg.position.x}, y: {msg.position.y}, z: {msg.position.z}')
        self.get_logger().info(f'Orientation -> x: {msg.orientation.x}, y: {msg.orientation.y}, z: {msg.orientation.z}, w: {msg.orientation.w}')

        self.target_pose.position.x = msg.position.x - 0.2
        self.target_pose.position.y = msg.position.y
        self.target_pose.position.z = msg.position.z


    def odom_mobile_base_callback(self, data):

        if self.target_pose is None:
            self.get_logger().warn("No target set yet. Waiting for target...")
            return

        if self.target_pose_reached_bool:
            return
            
        # Extract position and orientation from the Odometry data
        x_data = data.pose.pose.position.x
        y_data = data.pose.pose.position.y
        z_data = data.pose.pose.position.z
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z

        # Calculate the rotation matrix
        R11 = qw**2 + qx**2 - qy**2 - qz**2
        R12 = 2 * qx * qz + 2 * qw * qz
        R21 = 2 * qx * qz - 2 * qw * qz
        R22 = qw**2 - qx**2 + qy**2 - qz**2

        # Calculate the position of point P
        point_P = Pose()
        point_P.position.x = x_data + 0.1 * R11
        point_P.position.y = y_data + 0.1 * R21
        point_P.position.z = 0.1  # slightly above the ground

        # Calculate error between the target pose and current pose
        err_x = self.target_pose.position.x - point_P.position.x
        err_y = self.target_pose.position.y - point_P.position.y
        self.get_logger().info(f'err -> x: {err_x}, y: {err_y}')

        error_vect = np.matrix([[err_x], [err_y]])

        Kp_mat = np.array([[1.2, 0], [0, 0.2]])
        Ki_mat = 0.2 * np.eye(2)
        Kd_mat = 0.5 * np.eye(2)

        current_time = self.get_clock().now()
        dt = (current_time-self.t_init).nanoseconds * 1e-9

        if dt>0:
            error_deriv = (error_vect - self.prev_err)/dt
        else:
            error_deriv = np.array([[0],[0]])

        self.prev_err = error_vect
        self.t_init = current_time

        self.integrated_error_list.append(error_vect)
        if len(self.integrated_error_list) > self.length_of_integrated_error_list:
            self.integrated_error_list.pop(0) #remove last element
        #now sum them
        self.integrated_error = np.matrix([[0],[0]])
        for err in self.integrated_error_list:
            self.integrated_error = self.integrated_error + err

        # Control inputs (velocity) based on proportional control
        point_p_error_signal = Kp_mat * error_vect
        # point_p_error_signal = Kp_mat * error_vect + Kd_mat * error_deriv+ Ki_mat*self.integrated_error

        #control_input = point_p_error_signal
        Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
        R_det = np.linalg.det(Rotation_mat)
        R_rounded = np.round(Rotation_mat,decimals=3)
        current_angle = np.arctan2(Rotation_mat[0,1],Rotation_mat[1,1]) #this is also the angle about the z-axis of the base

        # The following relates the desired motion of the point P and the commanded forward and angular velocity of the mobile base [v,w]
        non_holonomic_mat = np.matrix([[np.cos(current_angle), -self.L*np.sin(current_angle)],[np.sin(current_angle),self.L*np.cos(current_angle)]])
        #Now perform inversion to find the forward velocity and angular velcoity of the mobile base.
        control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal #note: this matrix can always be inverted because the angle is L

        # Control message to command velocities
        control_msg = Twist()
        control_msg.linear.x = float(min(control_input.item(0), self.max_linear_velocity))
        control_msg.angular.z = float(min(control_input.item(1), self.max_angular_velocity))

        self.mobile_base_vel_publisher.publish(control_msg)

        # Check if target is reached
        err_magnitude = np.linalg.norm(error_vect)
        if err_magnitude < self.goal_reached_error:
            self.get_logger().info(f"Target reached at: {point_P.position.x}, {point_P.position.y}, {point_P.position.z}")
            # Once target is reached, stop the robot
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0
            self.mobile_base_vel_publisher.publish(control_msg)
            self.target_pose_reached_bool = True

            # Notify manipulation node
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_reached_publisher.publish(goal_msg)
            self.get_logger().info("Published goal reached signal.")
        
def main(args=None):
    rclpy.init(args=args)
    cls_obj = Navigation()  # Instantiate object of the class
    rclpy.spin(cls_obj)
    cls_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
