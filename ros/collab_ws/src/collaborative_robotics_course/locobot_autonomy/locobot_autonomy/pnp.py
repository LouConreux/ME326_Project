import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from typing import Optional, Tuple, List

@dataclass
class CameraParams:
    """Camera intrinsic parameters"""
    fx: float = 915.1482543945312
    fy: float = 915.0738525390625
    cx: float = 633.98828125
    cy: float = 363.17529296875
    dist_coeffs: np.ndarray = np.array([0.1, -0.1, 0.001, -0.001, 0])

    @property
    def camera_matrix(self) -> np.ndarray:
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

@dataclass
class ObjectModel:
    """3D model points of the object"""
    points_3d: np.ndarray  # Nx3 array of 3D points
    name: str = "default"

    @classmethod
    def create_rectangular_prism(cls, width: float, height: float, depth: float) -> 'ObjectModel':
        """Create a rectangular prism model with given dimensions (in meters)"""
        points = np.array([
            [0, 0, 0],          # Origin
            [width, 0, 0],      # +X
            [width, height, 0], # +X +Y
            [0, height, 0],     # +Y
            [0, 0, depth],      # +Z
            [width, 0, depth],  # +X +Z
            [width, height, depth], # +X +Y +Z
            [0, height, depth]  # +Y +Z
        ])
        return cls(points)

def estimate_pose(image_points: np.ndarray, 
                 object_model: ObjectModel,
                 camera_params: CameraParams) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Estimate 6-DoF pose using PnP.
    
    Args:
        image_points: Nx2 array of pixel coordinates
        object_model: 3D model points
        camera_params: Camera calibration parameters
    
    Returns:
        rotation_matrix: 3x3 rotation matrix
        translation_vector: 3x1 translation vector
        None, None if estimation fails
    """
    if len(image_points) < 4:
        print("Need at least 4 points for PnP")
        return None, None

    try:
        success, rotation_vec, translation_vec = cv2.solvePnP(
            object_model.points_3d,
            image_points,
            camera_params.camera_matrix,
            camera_params.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            print("PnP solving failed")
            return None, None

        rotation_mat, _ = cv2.Rodrigues(rotation_vec)
        return rotation_mat, translation_vec

    except cv2.error as e:
        print(f"OpenCV error during PnP: {e}")
        return None, None

def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to quaternion [x, y, z, w]"""
    r = Rotation.from_matrix(rotation_matrix)
    return r.as_quat()

def transform_pose(pose_rotation: np.ndarray, 
                  pose_translation: np.ndarray,
                  transform_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Transform pose from one frame to another using 4x4 transform matrix
    """
    # Create 4x4 pose matrix
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = pose_rotation
    pose_matrix[:3, 3] = pose_translation.ravel()

    # Apply transform
    transformed_pose = transform_matrix @ pose_matrix

    # Extract new rotation and translation
    new_rotation = transformed_pose[:3, :3]
    new_translation = transformed_pose[:3, 3].reshape(3, 1)

    return new_rotation, new_translation

def validate_pose(rotation_mat: np.ndarray, 
                 translation_vec: np.ndarray, 
                 max_distance: float = 2.0) -> bool:
    """
    Basic validation of estimated pose
    """
    # Check if rotation matrix is valid
    if not np.allclose(np.linalg.det(rotation_mat), 1.0, atol=1e-6):
        return False

    # Check if object is too far (probably erroneous)
    if np.linalg.norm(translation_vec) > max_distance:
        return False

    return True

# Usage example:
if __name__ == "__main__":
    # Create camera parameters
    camera = CameraParams()
    
    # Create object model (10cm x 8cm x 5cm box)
    object_model = ObjectModel.create_rectangular_prism(0.10, 0.08, 0.05)
    
    # Example pixel coordinates (replace with actual detections)
    image_points = np.array([
        [200, 300],  # Point 1
        [400, 300],  # Point 2
        [400, 400],  # Point 3
        [200, 400],  # Point 4
        [200, 250],  # Point 5
        [400, 250],  # Point 6
        [400, 350],  # Point 7
        [200, 350]   # Point 8
    ], dtype=np.float32)
    
    # Estimate pose
    rotation, translation = estimate_pose(image_points, object_model, camera)
    
    if rotation is not None:
        # Convert to quaternion if needed
        quaternion = rotation_matrix_to_quaternion(rotation)
        print("Estimated pose:")
        print(f"Translation: {translation.ravel()}")
        print(f"Rotation (quaternion): {quaternion}")
        
        # Example transform to robot base frame (replace with actual transform)
        transform = np.eye(4)  # Identity transform for example
        transform[:3, 3] = [0.1, 0, 0.2]  # 10cm X, 20cm Z offset
        
        # Transform pose to robot frame
        robot_rotation, robot_translation = transform_pose(
            rotation, translation, transform)