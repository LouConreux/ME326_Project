#!/usr/bin/env python3
import numpy as np
import cv2

def convert_intrinsics(img, old_intrinsics=(360.01, 360.01, 243.87, 137.92), 
                      new_intrinsics=(1297.67, 1298.63, 620.91, 238.28), 
                      new_size=(1280, 720)):
    """
    Convert a set of images to a different set of camera intrinsics.
    
    Parameters:
    - img: Input image
    - old_intrinsics: Tuple (fx, fy, cx, cy) of the old camera intrinsics
    - new_intrinsics: Tuple (fx, fy, cx, cy) of the new camera intrinsics
    - new_size: Tuple (width, height) defining the size of the output images
    
    Returns:
    - Converted image with new camera intrinsics
    """
    old_fx, old_fy, old_cx, old_cy = old_intrinsics
    new_fx, new_fy, new_cx, new_cy = new_intrinsics
    width, height = new_size
    
    # Constructing the old and new intrinsics matrices
    K_old = np.array([[old_fx, 0, old_cx], [0, old_fy, old_cy], [0, 0, 1]])
    K_new = np.array([[new_fx, 0, new_cx], [0, new_fy, new_cy], [0, 0, 1]])
    K_new_inv = np.linalg.inv(K_new)
    
    # Construct a grid of points representing the new image coordinates
    x, y = np.meshgrid(np.arange(width), np.arange(height))
    homogenous_coords = np.stack([x.ravel(), y.ravel(), np.ones_like(x).ravel()], axis=-1).T
    
    # Convert to the old image coordinates
    old_coords = K_old @ K_new_inv @ homogenous_coords
    old_coords /= old_coords[2, :]  # Normalize to make homogeneous
    
    # Reshape for remapping
    map_x = old_coords[0, :].reshape(height, width).astype(np.float32)
    map_y = old_coords[1, :].reshape(height, width).astype(np.float32)
    
    # Remap the image to the new intrinsics
    return cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR)

def compute_homography(K, R, t):
    """
    Compute the homography matrix given intrinsic matrix K, rotation matrix R, and translation vector t.
    """
    K_inv = np.linalg.inv(K)
    H = np.dot(K, np.dot(R - np.dot(t.reshape(-1, 1), K_inv[-1, :].reshape(1, -1)), K_inv))
    return H

def warp_image(image, K, R, t):
    """
    Warp an image from the perspective of camera 1 to camera 2.

    Parameters:
    - image: Input image from camera 1
    - K: Intrinsic matrix of both cameras
    - R: Rotation matrix from camera 1 to camera 2
    - t: Translation vector from camera 1 to camera 2
    
    Returns:
    - Warped image as seen from camera 2
    """
    H = compute_homography(K, R, t)
    height, width = image.shape[:2]
    return cv2.warpPerspective(image, H, (width, height))

def align_depth(depth: np.ndarray, depth_K: tuple, rgb: np.ndarray, 
                rgb_K: tuple, cam2cam_transform: np.ndarray) -> np.ndarray:
    """
    Align depth image to the RGB image.

    Parameters:
    - depth: Depth image
    - depth_K: Intrinsics of the depth camera (fx, fy, cx, cy)
    - rgb: RGB image
    - rgb_K: Intrinsics of the RGB camera (fx, fy, cx, cy)
    - cam2cam_transform: 4x4 transformation matrix from depth to RGB camera
    
    Returns:
    - Aligned depth image
    """
    # Convert the intrinsics of the depth camera to match the RGB camera
    depth = convert_intrinsics(
        depth, 
        new_size=(rgb.shape[1], rgb.shape[0]),
        new_intrinsics=rgb_K,
        old_intrinsics=depth_K
    )

    # Warp the depth image to the RGB image perspective
    return warp_image(
        depth,
        np.array([[rgb_K[0], 0, rgb_K[2]], [0, rgb_K[1], rgb_K[3]], [0, 0, 1]]),
        cam2cam_transform[:3, :3],
        cam2cam_transform[:3, 3]
    )