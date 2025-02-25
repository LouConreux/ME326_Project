#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import os
from tf2_geometry_msgs import do_transform_pose
from camera_utils import align_depth


# Import your existing components
from pipeline_perception import PipelinePerception
from pnp import get_object_pose

# Path to JSON key file
JSON_KEY_PATH = '/home/ubuntu/Desktop/collaborative/loulou_key.json'

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.jason_key_path = JSON_KEY_PATH
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.jason_key_path
        
        # Initialize perception pipeline
        self.pipeline = PipelinePerception()
        self.bridge = CvBridge()
        
        # Initialize camera info
        self.rgb_camera_matrix = None
        self.rgb_dist_coeffs = None
        self.depth_camera_matrix = None
        self.depth_dist_coeffs = None

        self.latest_rgb = None
        self.latest_depth = None

        self.cam2cam_transform = np.eye(4) ### to change
        
        # TF2 Buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/locobot/camera_frame_sensor/image_raw',
            self.rgb_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/locobot/camera_frame_sensor/image_raw',
            self.depth_callback,
            10)
        
        
        self.prompt_sub = self.create_subscription(
            String,
            '/user_prompt',
            self.prompt_callback,
            10)
            
        # Create publisher for object pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/detected_object_pose',
            10)
            
        # Store the latest prompt
        self.current_prompt = "Yellow Cube"

        # Store the latest audio
        self.current_audio = None

        # Initialize the perception pipeline        
        self.get_logger().info('Integrated perception node initialized')
        
    def rgb_info_callback(self, msg):
        """Process RGB camera calibration data"""
        if self.rgb_camera_matrix is None:
            self.rgb_camera_matrix = np.array(msg.k).reshape(3, 3)
            self.rgb_dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received RGB camera calibration')
    
    def depth_info_callback(self, msg):
        """Process depth camera calibration data"""
        if self.depth_camera_matrix is None:
            self.depth_camera_matrix = np.array(msg.k).reshape(3, 3)
            self.depth_dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received depth camera calibration')
        
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info('Received RGB image')
        self.process_images()
        self.get_logger().info('Processed RGB image')

    def depth_callback(self, msg):
        """Store latest depth image"""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.process_images()
        
    def prompt_callback(self, msg):
        if msg.data == 'None':
            self.current_prompt = "Yellow Cube"
            return
        else:
            self.current_prompt = msg.data
            self.get_logger().info(f'Received prompt: {self.current_prompt}')
            return

    def audio_callback(self, msg):
        # TODO: Find audio topic in LocoBot and set up class attribute audio
        self.current_audio = msg.data
        return 
        
    def process_images(self):
        """Process RGB and depth images when both are available"""
        if (self.latest_rgb is None or self.latest_depth is None or 
            self.current_prompt is None or self.rgb_camera_matrix is None or 
            self.depth_camera_matrix is None):
            return
        
        try:
            # Align depth to RGB
            rgb_intrinsics = (
                self.rgb_camera_matrix[0,0],  # fx
                self.rgb_camera_matrix[1,1],  # fy
                self.rgb_camera_matrix[0,2],  # cx
                self.rgb_camera_matrix[1,2]   # cy
            )
            
            depth_intrinsics = (
                self.depth_camera_matrix[0,0],  # fx
                self.depth_camera_matrix[1,1],  # fy
                self.depth_camera_matrix[0,2],  # cx
                self.depth_camera_matrix[1,2]   # cy
            )
            
            aligned_depth = align_depth(
                self.latest_depth,
                depth_intrinsics,
                self.latest_rgb,
                rgb_intrinsics,
                self.cam2cam_transform
            )
            
            # Convert RGB image to bytes for vision detector
            success, img_encoded = cv2.imencode('.jpg', self.latest_rgb)
            if not success:
                self.get_logger().error('Failed to encode image')
                return
                
            image_bytes = img_encoded.tobytes()
            
            # Detect object in RGB image
            center_coords, detected_object = self.pipeline.detector.find_center(
                command=self.current_prompt,
                image=image_bytes,
            )
            
            if center_coords is None:
                self.get_logger().warn(f'Object {self.current_prompt} not found in image')
                return
            else:
                self.get_logger().info(f'Found object {detected_object} at {center_coords} pixel coordinates')
            # Get depth at object location
            x, y = center_coords
            object_depth = aligned_depth[y, x]
            
            # Define 3D model points using actual depth
            object_size = 0.1  # 10cm
            object_points_3d = np.array([
                [0, 0, object_depth],
                [object_size, 0, object_depth],
                [object_size, object_size, object_depth],
                [0, object_size, object_depth]
            ])
            
            # Create 2D points array from center coordinates
            box_size = 50  # pixels
            pixel_coords = np.array([
                [x - box_size/2, y - box_size/2],
                [x + box_size/2, y - box_size/2],
                [x + box_size/2, y + box_size/2],
                [x - box_size/2, y + box_size/2]
            ], dtype=np.float32)
            
            # Get object pose using PnP
            rotation_matrix, translation_vector = get_object_pose(
                pixel_coords,
                object_points_3d,
                self.rgb_camera_matrix,
                self.rgb_dist_coeffs
            )
            
            # Create and publish pose message
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "rgb_camera_frame"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            pose_msg.pose.position.x = float(translation_vector[0])
            pose_msg.pose.position.y = float(translation_vector[1])
            pose_msg.pose.position.z = float(translation_vector[2])
            
            # Convert rotation matrix to quaternion
            from scipy.spatial.transform import Rotation
            r = Rotation.from_matrix(rotation_matrix)
            quat = r.as_quat()
            
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            
            # Transform to robot base frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    "rgb_camera_frame",
                    rclpy.time.Time()
                )
                
                pose_base = do_transform_pose(pose_msg, transform)
                self.pose_pub.publish(pose_base)
                self.get_logger().info(f'Published pose for {detected_object}')
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF2 error: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')
        
        return

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()