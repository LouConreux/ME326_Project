#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pipeline_perception import PipelinePerception
import os

JSON_KEY_PATH = "C:\Users\louis\Desktop\ME326\powerful-hall-449222-h3-6beba59045ba.json" #this is Louis' key, update with your path

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.jason_key_path = JSON_KEY_PATH
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.jason_key_path
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/locobot/camera/color/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.prompt_sub = self.create_subscription(
            String,
            '/user_prompt',
            self.prompt_callback,
            10)
        
        self.audio_sub = self.create_subscription(
            String,
            '/audio',
            self.audio_callback,
            10)
            
        # Store the latest prompt
        self.current_prompt = "Yellow Cube"

        # Store the latest audio
        self.current_audio = None

        # Initialize the perception pipeline
        self.perceptor = PipelinePerception()
        
    def prompt_callback(self, msg):
        self.current_prompt = msg.data
        self.get_logger().info(f'Received prompt: {self.current_prompt}')

    def audio_callback(self, msg):
        # TODO: Find audio topic in LocoBot and set up class attribute audio
        self.current_audio = msg.data
        
    def image_callback(self, msg):
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
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()