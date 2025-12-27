#!/usr/bin/env python3
"""
Projection utilities for GodView V2 Demo
=========================================
Proper 3D-to-2D projection using camera intrinsics/extrinsics.
This fixes V1's broken approximate projection that caused boxes to drift.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, List
import math


@dataclass
class CameraTransform:
    """Camera position and rotation in world coordinates."""
    x: float
    y: float
    z: float
    pitch: float  # degrees
    yaw: float    # degrees
    roll: float   # degrees


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters."""
    width: int
    height: int
    fov: float  # degrees
    fx: float   # focal length x
    fy: float   # focal length y
    cx: float   # principal point x
    cy: float   # principal point y


def build_intrinsics(width: int, height: int, fov: float) -> CameraIntrinsics:
    """
    Build camera intrinsics from resolution and field of view.
    
    Args:
        width: Image width in pixels
        height: Image height in pixels
        fov: Horizontal field of view in degrees
    
    Returns:
        CameraIntrinsics with computed focal lengths and principal point
    """
    # Focal length from FOV: f = w / (2 * tan(fov/2))
    fov_rad = np.radians(fov)
    fx = width / (2.0 * np.tan(fov_rad / 2.0))
    fy = fx  # Square pixels assumed
    
    # Principal point at image center
    cx = width / 2.0
    cy = height / 2.0
    
    return CameraIntrinsics(
        width=width,
        height=height,
        fov=fov,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy
    )


def build_K_matrix(intrinsics: CameraIntrinsics) -> np.ndarray:
    """
    Build 3x3 intrinsic matrix K.
    
    Returns:
        np.ndarray: 3x3 camera intrinsic matrix
    """
    return np.array([
        [intrinsics.fx, 0, intrinsics.cx],
        [0, intrinsics.fy, intrinsics.cy],
        [0, 0, 1]
    ], dtype=np.float64)


def rotation_matrix(pitch: float, yaw: float, roll: float) -> np.ndarray:
    """
    Build 3x3 rotation matrix from Euler angles (CARLA convention).
    
    CARLA uses:
    - Pitch: rotation around Y-axis (positive = nose up)
    - Yaw: rotation around Z-axis (positive = turn left)
    - Roll: rotation around X-axis (positive = tilt right)
    
    Args:
        pitch, yaw, roll: Angles in degrees
    
    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    # Convert to radians
    p = np.radians(pitch)
    y = np.radians(yaw)
    r = np.radians(roll)
    
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)]
    ])
    
    Ry = np.array([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])
    
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y), np.cos(y), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: R = Rz * Ry * Rx (ZYX order)
    return Rz @ Ry @ Rx


def world_to_camera(world_point: np.ndarray, cam_transform: CameraTransform) -> np.ndarray:
    """
    Transform a point from world coordinates to camera coordinates.
    
    Args:
        world_point: 3D point in world coordinates [x, y, z]
        cam_transform: Camera position and rotation
    
    Returns:
        np.ndarray: 3D point in camera coordinates
    """
    # Camera position
    cam_pos = np.array([cam_transform.x, cam_transform.y, cam_transform.z])
    
    # Translation: move point to camera origin
    translated = world_point - cam_pos
    
    # Rotation: rotate to camera frame
    R = rotation_matrix(cam_transform.pitch, cam_transform.yaw, cam_transform.roll)
    
    # Camera looks along +X in CARLA, but standard camera convention is -Z
    # Apply rotation and axis swap
    rotated = R.T @ translated  # Inverse rotation to go world -> camera
    
    # CARLA to standard camera convention:
    # CARLA: X=forward, Y=right, Z=up
    # Camera: X=right, Y=down, Z=forward
    camera_point = np.array([
        rotated[1],   # Y -> X (right)
        -rotated[2],  # -Z -> Y (down)
        rotated[0]    # X -> Z (forward/depth)
    ])
    
    return camera_point


def project_to_screen(camera_point: np.ndarray, K: np.ndarray) -> Optional[Tuple[int, int]]:
    """
    Project a 3D camera-space point to 2D screen coordinates.
    
    Args:
        camera_point: 3D point in camera coordinates [x, y, z]
        K: 3x3 intrinsic matrix
    
    Returns:
        (u, v) screen coordinates, or None if behind camera
    """
    # Point is behind camera
    if camera_point[2] <= 0:
        return None
    
    # Project: [u, v, w] = K * [x, y, z]
    projected = K @ camera_point
    
    # Normalize by depth
    u = int(projected[0] / projected[2])
    v = int(projected[1] / projected[2])
    
    return (u, v)


def get_3d_bbox_corners(
    pose: dict,
    extent: dict
) -> List[np.ndarray]:
    """
    Get the 8 corners of a 3D bounding box in world coordinates.
    
    Args:
        pose: Object pose with x, y, z, yaw (optional)
        extent: Bounding box half-extents with x, y, z
    
    Returns:
        List of 8 corner points in world coordinates
    """
    # Center position
    cx = pose['x']
    cy = pose['y']
    cz = pose['z']
    yaw = pose.get('yaw', 0)
    
    # Half-extents
    ex = extent['x']
    ey = extent['y']
    ez = extent['z']
    
    # 8 corners in local frame (centered at origin)
    corners_local = [
        np.array([-ex, -ey, -ez]),
        np.array([-ex, -ey, +ez]),
        np.array([-ex, +ey, -ez]),
        np.array([-ex, +ey, +ez]),
        np.array([+ex, -ey, -ez]),
        np.array([+ex, -ey, +ez]),
        np.array([+ex, +ey, -ez]),
        np.array([+ex, +ey, +ez]),
    ]
    
    # Rotation by yaw
    yaw_rad = np.radians(yaw)
    cos_yaw = np.cos(yaw_rad)
    sin_yaw = np.sin(yaw_rad)
    
    R_yaw = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    # Transform corners to world
    center = np.array([cx, cy, cz])
    corners_world = [R_yaw @ c + center for c in corners_local]
    
    return corners_world


def project_3d_bbox(
    pose: dict,
    extent: dict,
    cam_transform: CameraTransform,
    K: np.ndarray
) -> Optional[Tuple[int, int, int, int]]:
    """
    Project a 3D bounding box to 2D screen rectangle.
    
    This is the CORRECT way to render boxes that stick to vehicles.
    Projects all 8 corners and returns min/max bounds.
    
    Args:
        pose: Object pose with x, y, z, yaw
        extent: Bounding box half-extents with x, y, z
        cam_transform: Camera position and rotation
        K: 3x3 intrinsic matrix
    
    Returns:
        (x_min, y_min, x_max, y_max) screen rectangle, or None if not visible
    """
    # Get 8 corners in world coordinates
    corners_world = get_3d_bbox_corners(pose, extent)
    
    # Project each corner
    corners_2d = []
    for corner in corners_world:
        cam_point = world_to_camera(corner, cam_transform)
        screen_point = project_to_screen(cam_point, K)
        if screen_point is not None:
            corners_2d.append(screen_point)
    
    # Need at least 4 visible corners for a valid box
    if len(corners_2d) < 4:
        return None
    
    # Compute 2D bounding rectangle
    xs = [p[0] for p in corners_2d]
    ys = [p[1] for p in corners_2d]
    
    return (min(xs), min(ys), max(xs), max(ys))


def clamp_bbox_to_screen(
    bbox: Tuple[int, int, int, int],
    width: int,
    height: int
) -> Optional[Tuple[int, int, int, int]]:
    """
    Clamp bounding box to screen bounds.
    
    Returns None if completely outside screen.
    """
    x_min, y_min, x_max, y_max = bbox
    
    # Check if completely outside
    if x_max < 0 or y_max < 0 or x_min >= width or y_min >= height:
        return None
    
    # Clamp to screen
    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(width - 1, x_max)
    y_max = min(height - 1, y_max)
    
    # Check minimum size
    if x_max - x_min < 5 or y_max - y_min < 5:
        return None
    
    return (x_min, y_min, x_max, y_max)


# Convenience function
def project_actor_to_screen(
    pose: dict,
    extent: dict,
    cam_transform: CameraTransform,
    intrinsics: CameraIntrinsics
) -> Optional[Tuple[int, int, int, int]]:
    """
    Full pipeline: project actor 3D bbox to clamped 2D screen rectangle.
    """
    K = build_K_matrix(intrinsics)
    bbox = project_3d_bbox(pose, extent, cam_transform, K)
    
    if bbox is None:
        return None
    
    return clamp_bbox_to_screen(bbox, intrinsics.width, intrinsics.height)
