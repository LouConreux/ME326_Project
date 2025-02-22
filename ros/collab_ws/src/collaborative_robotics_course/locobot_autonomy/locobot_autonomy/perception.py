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
JSON_KEY_PATH = '/home/ubuntu/Desktop/collaborative/keys/tomtom_key.json'

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info('Starting perception node initialization...')
        
        self.jason_key_path = JSON_KEY_PATH
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.jason_key_path
        self.get_logger().info(f'Set Google credentials path to: {self.jason_key_path}')
        
        # Initialize perception pipeline
        self.get_logger().info('Initializing perception pipeline...')
        self.pipeline = PipelinePerception()
        self.bridge = CvBridge()
        self.get_logger().info('Perception pipeline initialized')
        
        # Initialize camera info
        self.rgb_camera_matrix = None
        self.rgb_dist_coeffs = None
        self.depth_camera_matrix = None
        self.depth_dist_coeffs = None

        self.latest_rgb = None
        self.latest_depth = None

        self.cam2cam_transform = np.eye(4) ### to change
        
        # TF2 Buffer for coordinate transformations
        self.get_logger().info('Setting up TF2 buffer...')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF2 buffer setup complete')
        
        # Create subscribers
        self.get_logger().info('Creating subscribers...')
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
            
        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            '/locobot/camera_frame_sensor/camera_info',
            self.rgb_info_callback,
            10)
        
        
        self.prompt_sub = self.create_subscription(
            String,
            '/user_prompt',
            self.prompt_callback,
            10)
        self.get_logger().info('All subscribers created')
            
        # Create publisher for object pose
        self.get_logger().info('Creating publishers...')
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/detected_object_pose',
            10)
        self.get_logger().info('Publishers created')
            
        # Store the latest prompt
        self.current_prompt = "Blue Cube"
        self.current_audio = None

        # Initialize the perception pipeline
        self.perceptor = PipelinePerception()
        
        self.get_logger().info('Perception node initialization complete')
        
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
        self.get_logger().debug('Received RGB image')
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite('latest_rgb.jpg', self.latest_rgb)
            self.get_logger().info('Saved latest RGB image to latest_rgb.jpg')

            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {str(e)}')
        
    def depth_callback(self, msg):
        """Store latest depth image"""
        self.get_logger().debug('Received depth image')
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {str(e)}')
        
    def prompt_callback(self, msg):
        self.get_logger().info(f'Received prompt message: {msg.data}')
        if msg.data == 'None':
            self.current_prompt = "Blue Cube"
            return
        else:
            self.current_prompt = msg.data
            self.get_logger().info(f'Set current prompt to: {self.current_prompt}')
            return
        
    def process_images(self):
        """Process RGB and depth images when both are available"""
        self.get_logger().debug('Starting image processing')
        
        # Check if we have all required data
        if self.latest_rgb is None:
            self.get_logger().debug('RGB image not available')
            return
        if self.latest_depth is None:
            self.get_logger().debug('Depth image not available')
            return
        if self.current_prompt is None:
            self.get_logger().debug('Prompt not available')
            return
        if self.rgb_camera_matrix is None:
            self.get_logger().debug('RGB camera matrix not available')
            return
        if self.depth_camera_matrix is None:
            self.get_logger().debug('Depth camera matrix not available')
            return
        
        try:
            self.get_logger().debug('Aligning depth to RGB...')
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
            self.get_logger().debug('Depth alignment complete')
            
            # Convert RGB image to bytes for vision detector
            self.get_logger().debug('Converting RGB image to bytes...')
            success, img_encoded = cv2.imencode('.jpg', self.latest_rgb)
            if not success:
                self.get_logger().error('Failed to encode image')
                return
                
            image_bytes = img_encoded.tobytes()
            self.get_logger().debug('Image conversion complete')
            
            # Detect object in RGB image
            self.get_logger().debug(f'Detecting object: {self.current_prompt}')
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
            self.get_logger().debug(f'Object depth: {object_depth}')
            
            # Rest of the processing...
            self.get_logger().debug('Computing object pose...')
            
            # Get object pose using PnP
            rotation_matrix, translation_vector = get_object_pose(
                pixel_coords,
                object_points_3d,
                self.rgb_camera_matrix,
                self.rgb_dist_coeffs
            )
            self.get_logger().debug('Object pose computed')
            
            # Create and publish pose message
            self.get_logger().debug('Publishing pose...')
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
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        return

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
        node.get_logger().info('Starting node spin...')
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
        import traceback
        print(traceback.format_exc())
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()