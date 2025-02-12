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
from pnp import get_object_pose

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
            10)
        
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
        self.current_prompt = "Yellow Cube"

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
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert OpenCV image to bytes for vision detector
            success, img_encoded = cv2.imencode('.jpg', cv_image)
            if not success:
                self.get_logger().error('Failed to encode image')
                return
                
            image_bytes = img_encoded.tobytes()
            
            # Use the pipeline to detect object (note: we skip audio part)
            center_coords, detected_object = self.pipeline.detector.find_center(image_bytes, self.current_prompt)
            
            if center_coords is None:
                self.get_logger().warn(f'Object {self.current_prompt} not found in image')
                return
                
            # Get annotated image with detections (useful for debugging)
            annotated_image = self.pipeline.detector.annotate_image(image_bytes)
                
            # Define 3D model points for PnP
            # This is an example - adjust based on your known object dimensions
            object_size = 0.1  # 10cm
            object_points_3d = np.array([
                [0, 0, 0],
                [object_size, 0, 0],
                [object_size, object_size, 0],
                [0, object_size, 0]
            ])
            
            # Create 2D points array from center coordinates
            # This expands the center point to create a bounding box for PnP
            x, y = center_coords
            box_size = 50  # pixels
            pixel_coords = np.array([
                [x - box_size/2, y - box_size/2],
                [x + box_size/2, y - box_size/2],
                [x + box_size/2, y + box_size/2],
                [x - box_size/2, y + box_size/2]
            ], dtype=np.float32)
            
            # Get object pose using PnP
            try:
                rotation_matrix, translation_vector = get_object_pose(
                    pixel_coords,
                    object_points_3d,
                    self.camera_matrix,
                    self.dist_coeffs
                )
                
                # Create PoseStamped message
                pose_camera = PoseStamped()
                pose_camera.header.frame_id = msg.header.frame_id
                pose_camera.header.stamp = self.get_clock().now().to_msg()
                
                # Set position from translation vector
                pose_camera.pose.position.x = float(translation_vector[0])
                pose_camera.pose.position.y = float(translation_vector[1])
                pose_camera.pose.position.z = float(translation_vector[2])
                
                # Convert rotation matrix to quaternion
                from scipy.spatial.transform import Rotation
                r = Rotation.from_matrix(rotation_matrix)
                quat = r.as_quat()
                
                pose_camera.pose.orientation.x = float(quat[0])
                pose_camera.pose.orientation.y = float(quat[1])
                pose_camera.pose.orientation.z = float(quat[2])
                pose_camera.pose.orientation.w = float(quat[3])
                
                # Transform to robot frame
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',  # target frame
                        msg.header.frame_id,  # source frame
                        rclpy.time.Time())
                    
                    pose_base = do_transform_pose(pose_camera, transform)
                    self.pose_pub.publish(pose_base)
                    self.get_logger().info(f'Published pose for {detected_object}')
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    self.get_logger().error(f'TF2 error: {str(e)}')
                
            except RuntimeError as e:
                self.get_logger().error(f'PnP error: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')

        if self.current_prompt is None and self.current_audio is None:
            return

        elif self.current_prompt is not None:
            center_coordinates, object_name = self.perceptor.command_pipeline(self.current_prompt, cv_image)

        elif self.current_audio is not None:
            center_coordinates, object_name = self.perceptor.audio_pipeline(self.current_prompt, cv_image)
        return

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()