#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

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
            
        # Store the latest prompt
        self.current_prompt = None
        
    def prompt_callback(self, msg):
        self.current_prompt = msg.data
        self.get_logger().info(f'Received prompt: {self.current_prompt}')
        
    def image_callback(self, msg):
        if self.current_prompt is None:
            return
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # TODO: Add your perception logic here based on self.current_prompt
            # Examples:
            # - Object detection
            # - Pose estimation
            # - Feature extraction
            # - Segmentation
            
            # Process results and publish as needed
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()