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
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera info
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF2 Buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/locobot/camera/color/image_raw',
            self.image_callback,
            10)
            
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
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
        self.perceptor = PipelinePerception()
        
        self.get_logger().info('Integrated perception node initialized')
        
    def camera_info_callback(self, msg):
        """Process camera calibration data"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received camera calibration data')
        
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
        
    def image_callback(self, msg):
        """Process incoming images and detect objects"""
        if self.current_prompt is None or self.camera_matrix is None:
            return
            
        try:
            cv_ColorImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            _, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            image_bytes = encoded_image.tobytes()
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

        if self.current_prompt is None and self.current_audio is None:
            return

        elif self.current_prompt is not None:
            center_coordinates, object_name = self.perceptor.command_pipeline(self.current_prompt, image_bytes)
            self.logger.info(f'Object {object_name} found at pixel coordinates: {center_coordinates}')

        elif self.current_audio is not None:
            center_coordinates, object_name = self.perceptor.audio_pipeline(self.current_prompt, image_bytes)
            self.logger.info(f'Object {object_name} found at pixel coordinates: {center_coordinates}')
        return

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()