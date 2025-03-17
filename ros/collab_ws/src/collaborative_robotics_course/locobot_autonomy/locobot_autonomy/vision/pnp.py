import numpy as np
import cv2
from scipy.spatial.transform import Rotation

def get_object_pose(pixel_coords,
                   object_points_3d,
                   camera_matrix,
                   dist_coeffs):
    """
    Get object pose in camera frame using PnP
    
    Args:
        pixel_coords: Nx2 array of pixel coordinates
        object_points_3d: Nx3 array of corresponding 3D model points
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: distortion coefficients
        
    Returns:
        rotation_matrix: 3x3 rotation matrix
        translation_vector: 3x1 translation vector
    """
    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        object_points_3d,
        pixel_coords,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success:
        raise RuntimeError("Failed to solve PnP")
        
    # Convert rotation vector to matrix
    rmat, _ = cv2.Rodrigues(rvec)
    
    return rmat, tvec


if __name__ == "__main__":
    camera_matrix = np.array([
        [915.1482543945312, 0, 633.98828125],
        [0, 915.0738525390625, 363.17529296875],
        [0, 0, 1]
    ])
    
    dist_coeffs = np.array([0.1, -0.1, 0.001, -0.001, 0])
    
    # Example 3D model points (10cm x 8cm rectangle)
    object_points = np.array([
        [0.0, 0.0, 0.0],      # Origin
        [0.10, 0.0, 0.0],     # Point on +X axis
        [0.10, 0.08, 0.0],    # Point on +Y axis
        [0.0, 0.08, 0.0]      # Fourth corner
    ])
    
    # Example detected pixel coordinates
    pixel_coords = np.array([
        [200, 300],
        [400, 300],
        [400, 400],
        [200, 400]
    ], dtype=np.float32)
    
    # Get pose
    rotation, translation = get_object_pose(
        pixel_coords, 
        object_points, 
        camera_matrix, 
        dist_coeffs
    )
    
    print("Object pose in camera frame:")
    print(f"Translation: {translation.ravel()}")
    r = Rotation.from_matrix(rotation)
    print(f"Rotation (euler xyz): {r.as_euler('xyz', degrees=True)}")