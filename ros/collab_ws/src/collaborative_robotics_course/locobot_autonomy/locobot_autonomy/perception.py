#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pipeline_perception import PipelinePerception

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
        
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
        self.current_prompt = None

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
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

        if self.current_prompt is None and self.current_audio is None:
            return

        elif self.current_prompt is not None:
            center_coordinates, object_name = self.perceptor.command_pipeline(self.current_prompt, cv_image)

        elif self.current_audio is not None:
            center_coordinates, object_name = self.perceptor.audio_pipeline(self.current_prompt, cv_image)
        return

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()