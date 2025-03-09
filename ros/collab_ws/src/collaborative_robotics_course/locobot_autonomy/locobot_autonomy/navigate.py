#!/usr/bin/env python3
'''

This script controls the robot to go from the origin (0, 0, 0) to the published target point.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros
from tf2_geometry_msgs import do_transform_pose, do_transform_pose_stamped

import geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

# odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

import time
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set the target pose to (10, 10, 10) - we don't switch between points now
        
        # target_pose = Pose()
        # target_pose.pose.position.x = 3.0
        # target_pose.pose.position.y = 0.0
        # target_pose.pose.position.z = 0.0
        # self.target_pose = target_pose

        # Initialize locobot
        # self.bot = InterbotixLocobotXS(
        # robot_model='locobot_wx250s',
        # robot_name='locobot',
        # arm_model='mobile_wx250s'
        # )
        
        # Define the publishers, including "Twist" for moving the base, and "Bool" for determining if the robot is at the goal pose
        self.mobile_base_vel_publisher = self.create_publisher(
            Twist, 
            "/locobot/mobile_base/cmd_vel",
            10)
        
        self.goal_reached_publisher = self.create_publisher(
            Bool, 
            "/robot_at_goal", 
            10)


        # **************ros2 topic pub /goal_pose geometry_msgs/msg/Pose '{position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
        #self.target_pose_visual = self.create_publisher(Marker, "/locobot/mobile_base/target_pose_visual", 1)

        # Define the subscribers, including "Pose" to get the goal pose, and "Odometry" to get the instantaneous pose of the robot
        self.object_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',  # Topic from perception node
            self.position_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/locobot/mobile_base/odom",   #topic
            self.odom_mobile_base_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.L = 0.1 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        #set targets for when a goal is reached: 
        self.goal_reached_error = 0.05
        self.integrated_error = np.matrix([[0],[0]]) #this is the integrated error for Proportional, Integral (PI) control
        # self.integrated_error_factor = 1.0 #multiply this by accumulated error, this is the Ki (integrated error) gain
        self.integrated_error_list = []
        self.length_of_integrated_error_list = 20
        self.t_init = self.get_clock().now()
        self.prev_err = np.array([[0],[0]])
        


    def position_callback(self, msg):
        # get the target pose based on the Pose data
        self.target_pose = msg
        self.get_logger().info(f'Position -> x: {msg.pose.position.x}, y: {msg.pose.position.y}, z: {msg.pose.position.z}')
        self.get_logger().info(f'Orientation -> x: {msg.pose.orientation.x}, y: {msg.pose.orientation.y}, z: {msg.pose.orientation.z}, w: {msg.pose.orientation.w}')

        self.target_pose.pose.position.x = msg.pose.position.x - 0.2
        self.target_pose.pose.position.y = msg.pose.position.y
        self.target_pose.pose.position.z = msg.pose.position.z



    def odom_mobile_base_callback(self, data):


        if self.target_pose is None:
            #self.get_logger().warn("No target set yet. Waiting for target...")
            return

        pose_msg = PoseStamped()  
        pose_msg.header.frame_id = 'odom'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        try:
            transform = self.tf_buffer.lookup_transform(
                'locobot/arm_base_link',
                pose_msg.header.frame_id,
                rclpy.time.Time()
            )
            
            pose_base = do_transform_pose_stamped(pose_msg, transform)
            self.get_logger().info(f'Odom Pose: {pose_base.pose.position}')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF2 error: {str(e)}')

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
        err_x = self.target_pose.pose.position.x - point_P.position.x
        err_y = self.target_pose.pose.position.y - point_P.position.y
        self.get_logger().info(f'err -> x: {err_x}, y: {err_y}')

        error_vect = np.matrix([[err_x], [err_y]])

        Kp_mat = 1.2 * np.eye(2)
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
        self.get_logger().info(f'Control Input:{control_input}')
        control_msg.linear.x = float(control_input.item(0))
        control_msg.angular.z = float(control_input.item(1))
        self.get_logger().info(f"Move in X-axis: {control_msg.linear.x}, Rotate in Z-axis: {control_msg.angular.z}")
        
        self.mobile_base_vel_publisher.publish(control_msg)

        # Check if target is reached
        err_magnitude = np.linalg.norm(error_vect)
        if err_magnitude < self.goal_reached_error:
            self.get_logger().info(f"Target reached at: {point_P.position.x}, {point_P.position.y}, {point_P.position.z}")
            # Once target is reached, stop the robot
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0
            self.mobile_base_vel_publisher.publish(control_msg)

            # Notify manipulation node
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_reached_publisher.publish(goal_msg)
            self.get_logger().info("Published goal reached signal.")
            time.sleep(1000)
        
def main(args=None):
    rclpy.init(args=args)
    cls_obj = Navigation()  # Instantiate object of the class
    rclpy.spin(cls_obj)
    cls_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
