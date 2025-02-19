#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from camera_utils import align_depth

# Import your existing components
from pipeline_perception import PipelinePerception
import os

JSON_KEY_PATH = "C:\Users\louis\Desktop\ME326\powerful-hall-449222-h3-6beba59045ba.json" #this is Louis' key, update with your path

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
            '/locobot/camera/color/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.depth_callback,
            10)
            
        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            'rgb_camera/camera_info',
            self.rgb_info_callback,
            10)
        
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            'depth_camera/camera_info',
            self.depth_info_callback,
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
        self.current_prompt = "Shoe"

        # Store the latest audio
        self.current_audio = None

        # Initialize the perception pipeline
        self.perceptor = PipelinePerception()
        
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
        self.process_images()
        
    def depth_callback(self, msg):
        """Store latest depth image"""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.process_images()
        
    def prompt_callback(self, msg):
        """Handle new object detection requests"""
        self.current_prompt = msg.data
        self.get_logger().info(f'Looking for object: {self.current_prompt}')
        
    def process_images(self):
        """Process RGB and depth images when both are available"""
        if (self.latest_rgb is None or self.latest_depth is None or 
            self.current_prompt is None or self.rgb_camera_matrix is None or 
            self.depth_camera_matrix is None):
            return
            
        try:
            cv_ColorImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            _, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            image_bytes = encoded_image.tobytes()
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')

        if self.current_prompt is None and self.current_audio is None:
            return

        elif self.current_prompt is not None:
            center_coordinates, object_name = self.perceptor.command_pipeline(self.current_prompt, image_bytes)

        elif self.current_audio is not None:
            center_coordinates, object_name = self.perceptor.audio_pipeline(self.current_prompt, image_bytes)
        return

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()