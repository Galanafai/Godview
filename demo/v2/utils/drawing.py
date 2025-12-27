#!/usr/bin/env python3
"""
OpenCV Drawing Utilities for GodView V2 Demo
=============================================
Helper functions for rendering HUD overlays, bounding boxes,
animations, and split-screen layouts.
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass


# ============================================================================
# COLOR CONSTANTS (BGR for OpenCV)
# ============================================================================

class Colors:
    """Color palette for the demo."""
    RED = (50, 50, 255)         # Raw/chaos
    GREEN = (50, 255, 50)       # Fused/stable
    MAGENTA = (255, 0, 255)     # Sybil/malicious
    YELLOW = (0, 255, 255)      # Scanline/activation
    WHITE = (255, 255, 255)     # Text
    BLACK = (0, 0, 0)           # Background
    GREY = (100, 100, 100)      # Grid lines
    DARK_GREY = (40, 40, 40)    # Panels
    LIGHT_BLUE = (255, 200, 150)  # LIDAR rings
    ORANGE = (0, 165, 255)      # Warnings


# ============================================================================
# FONTS
# ============================================================================

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_MONO = cv2.FONT_HERSHEY_DUPLEX


# ============================================================================
# BOUNDING BOX DRAWING
# ============================================================================

def draw_bbox(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    color: Tuple[int, int, int] = Colors.GREEN,
    thickness: int = 2,
    dashed: bool = False,
    label: Optional[str] = None,
    alpha: float = 1.0
) -> None:
    """
    Draw a 2D bounding box on the frame.
    
    Args:
        frame: Image to draw on
        bbox: (x_min, y_min, x_max, y_max) rectangle
        color: BGR color tuple
        thickness: Line thickness
        dashed: If True, draw dashed lines
        label: Optional text label
        alpha: Opacity (1.0 = fully opaque)
    """
    x1, y1, x2, y2 = bbox
    
    if alpha < 1.0:
        overlay = frame.copy()
        if dashed:
            draw_dashed_rect(overlay, (x1, y1), (x2, y2), color, thickness)
        else:
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thickness)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
    else:
        if dashed:
            draw_dashed_rect(frame, (x1, y1), (x2, y2), color, thickness)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
    
    if label:
        # Draw label above box
        label_size, _ = cv2.getTextSize(label, FONT, 0.5, 1)
        label_y = max(y1 - 5, label_size[1] + 5)
        cv2.putText(frame, label, (x1, label_y), FONT, 0.5, color, 1, cv2.LINE_AA)


def draw_dashed_rect(
    frame: np.ndarray,
    pt1: Tuple[int, int],
    pt2: Tuple[int, int],
    color: Tuple[int, int, int],
    thickness: int = 2,
    dash_length: int = 10
) -> None:
    """Draw a dashed rectangle."""
    x1, y1 = pt1
    x2, y2 = pt2
    
    # Draw each side with dashes
    draw_dashed_line(frame, (x1, y1), (x2, y1), color, thickness, dash_length)
    draw_dashed_line(frame, (x2, y1), (x2, y2), color, thickness, dash_length)
    draw_dashed_line(frame, (x2, y2), (x1, y2), color, thickness, dash_length)
    draw_dashed_line(frame, (x1, y2), (x1, y1), color, thickness, dash_length)


def draw_dashed_line(
    frame: np.ndarray,
    pt1: Tuple[int, int],
    pt2: Tuple[int, int],
    color: Tuple[int, int, int],
    thickness: int = 2,
    dash_length: int = 10
) -> None:
    """Draw a dashed line."""
    dist = np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    num_dashes = int(dist / (dash_length * 2))
    
    if num_dashes < 1:
        cv2.line(frame, pt1, pt2, color, thickness)
        return
    
    for i in range(num_dashes):
        start_ratio = (i * 2) / (num_dashes * 2)
        end_ratio = (i * 2 + 1) / (num_dashes * 2)
        
        start_x = int(pt1[0] + start_ratio * (pt2[0] - pt1[0]))
        start_y = int(pt1[1] + start_ratio * (pt2[1] - pt1[1]))
        end_x = int(pt1[0] + end_ratio * (pt2[0] - pt1[0]))
        end_y = int(pt1[1] + end_ratio * (pt2[1] - pt1[1]))
        
        cv2.line(frame, (start_x, start_y), (end_x, end_y), color, thickness)


# ============================================================================
# DRONE ALTITUDE STEM
# ============================================================================

def draw_altitude_stem(
    frame: np.ndarray,
    drone_screen_pos: Tuple[int, int],
    ground_screen_pos: Tuple[int, int],
    color: Tuple[int, int, int] = Colors.GREEN,
    altitude_m: float = 0.0
) -> None:
    """
    Draw a vertical stem from drone to ground showing altitude.
    
    Args:
        frame: Image to draw on
        drone_screen_pos: (x, y) of drone in screen coords
        ground_screen_pos: (x, y) of ground point in screen coords
        color: Line color
        altitude_m: Altitude in meters (for label)
    """
    # Draw stem line
    cv2.line(frame, drone_screen_pos, ground_screen_pos, color, 2)
    
    # Draw ground marker
    gx, gy = ground_screen_pos
    cv2.circle(frame, (gx, gy), 4, color, -1)
    
    # Draw altitude label
    if altitude_m > 0:
        label = f"Z: {altitude_m:.0f}m"
        mid_x = (drone_screen_pos[0] + ground_screen_pos[0]) // 2 + 5
        mid_y = (drone_screen_pos[1] + ground_screen_pos[1]) // 2
        cv2.putText(frame, label, (mid_x, mid_y), FONT, 0.4, color, 1, cv2.LINE_AA)


# ============================================================================
# TRUST BADGES
# ============================================================================

def draw_trust_badge(
    frame: np.ndarray,
    pos: Tuple[int, int],
    trusted: bool,
    size: int = 20
) -> None:
    """
    Draw a trust verification badge (checkmark or X).
    
    Args:
        frame: Image to draw on
        pos: (x, y) center position
        trusted: True for green ✓, False for red ✗
        size: Badge size in pixels
    """
    x, y = pos
    
    if trusted:
        # Green checkmark
        color = Colors.GREEN
        # Draw circle background
        cv2.circle(frame, (x, y), size, color, 2)
        # Draw checkmark
        scale = size / 20
        pts = np.array([
            [x - int(8*scale), y + int(2*scale)],
            [x - int(2*scale), y + int(8*scale)],
            [x + int(10*scale), y - int(6*scale)]
        ], np.int32)
        cv2.polylines(frame, [pts], False, color, 2)
    else:
        # Red X
        color = Colors.RED
        cv2.circle(frame, (x, y), size, color, 2)
        offset = int(size * 0.5)
        cv2.line(frame, (x - offset, y - offset), (x + offset, y + offset), color, 2)
        cv2.line(frame, (x + offset, y - offset), (x - offset, y + offset), color, 2)


# ============================================================================
# SCANLINE EFFECT
# ============================================================================

def draw_scanline(
    frame: np.ndarray,
    progress: float,
    color: Tuple[int, int, int] = Colors.YELLOW,
    glow_width: int = 5
) -> None:
    """
    Draw a horizontal scanline sweeping down the screen.
    
    Args:
        frame: Image to draw on
        progress: 0.0 (top) to 1.0 (bottom)
        color: Scanline color
        glow_width: Width of glow effect
    """
    h, w = frame.shape[:2]
    y = int(progress * h)
    
    if y < 0 or y >= h:
        return
    
    # Draw glow (multiple fading lines)
    for offset in range(glow_width):
        alpha = 1.0 - (offset / glow_width)
        glow_color = tuple(int(c * alpha) for c in color)
        
        y_pos = y - offset
        if 0 <= y_pos < h:
            cv2.line(frame, (0, y_pos), (w, y_pos), glow_color, 2)
    
    # Main bright line
    cv2.line(frame, (0, y), (w, y), color, 3)


# ============================================================================
# LIDAR RANGE RINGS
# ============================================================================

def draw_lidar_rings(
    frame: np.ndarray,
    center: Tuple[int, int],
    radii_px: List[int],
    color: Tuple[int, int, int] = Colors.LIGHT_BLUE
) -> None:
    """
    Draw concentric LIDAR range rings.
    
    Args:
        frame: Image to draw on
        center: (x, y) screen center
        radii_px: List of radii in pixels
        color: Ring color
    """
    for radius in radii_px:
        cv2.circle(frame, center, radius, color, 1, cv2.LINE_AA)
        
        # Add range label
        label_x = center[0] + radius + 5
        label_y = center[1]
        # Only add label if within frame
        h, w = frame.shape[:2]
        if label_x < w - 30:
            cv2.putText(frame, f"{radius}px", (label_x, label_y), 
                       FONT, 0.3, color, 1, cv2.LINE_AA)


# ============================================================================
# H3 HEXAGON GRID
# ============================================================================

def draw_hexagon(
    frame: np.ndarray,
    vertices: List[Tuple[int, int]],
    color: Tuple[int, int, int] = Colors.GREY,
    filled: bool = False,
    thickness: int = 1,
    alpha: float = 1.0
) -> None:
    """
    Draw a hexagon from screen-projected vertices.
    
    Args:
        frame: Image to draw on
        vertices: List of 6 (x, y) screen coordinates
        color: Hex color
        filled: If True, fill the hexagon
        thickness: Line thickness (for outline)
        alpha: Opacity
    """
    if len(vertices) < 6:
        return
    
    pts = np.array(vertices, dtype=np.int32)
    
    if alpha < 1.0:
        overlay = frame.copy()
        if filled:
            cv2.fillPoly(overlay, [pts], color)
        else:
            cv2.polylines(overlay, [pts], True, color, thickness)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
    else:
        if filled:
            cv2.fillPoly(frame, [pts], color)
        else:
            cv2.polylines(frame, [pts], True, color, thickness)


# ============================================================================
# SPLIT-SCREEN LAYOUT
# ============================================================================

def create_split_screen(
    left_frame: np.ndarray,
    right_frame: np.ndarray,
    divider_color: Tuple[int, int, int] = Colors.WHITE,
    labels: Tuple[str, str] = ("RAW SENSOR", "GODVIEW CONSENSUS")
) -> np.ndarray:
    """
    Combine two frames into a split-screen view.
    
    Args:
        left_frame: Left half (RAW view)
        right_frame: Right half (GODVIEW view)
        divider_color: Color of vertical divider
        labels: (left_label, right_label)
    
    Returns:
        Combined frame
    """
    h, w = left_frame.shape[:2]
    half_w = w // 2
    
    # Create output frame
    combined = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Resize and place left frame
    left_resized = cv2.resize(left_frame, (half_w, h))
    combined[:, :half_w] = left_resized
    
    # Resize and place right frame
    right_resized = cv2.resize(right_frame, (half_w, h))
    combined[:, half_w:] = right_resized
    
    # Draw vertical divider
    cv2.line(combined, (half_w, 0), (half_w, h), divider_color, 3)
    
    # Draw labels
    left_label, right_label = labels
    cv2.putText(combined, left_label, (20, 40), FONT, 0.7, Colors.RED, 2, cv2.LINE_AA)
    cv2.putText(combined, right_label, (half_w + 20, 40), FONT, 0.7, Colors.GREEN, 2, cv2.LINE_AA)
    
    return combined


# ============================================================================
# HUD PANELS
# ============================================================================

def draw_top_bar(
    frame: np.ndarray,
    phase_name: str,
    status: str,
    phase_color: Tuple[int, int, int] = Colors.WHITE
) -> None:
    """Draw the top HUD bar with phase name and status."""
    h, w = frame.shape[:2]
    
    # Semi-transparent background
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 60), Colors.DARK_GREY, -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    # Phase name
    cv2.putText(frame, phase_name, (20, 40), FONT, 1.0, phase_color, 2, cv2.LINE_AA)
    
    # Status (right-aligned)
    status_size = cv2.getTextSize(status, FONT, 0.7, 1)[0]
    cv2.putText(frame, status, (w - status_size[0] - 20, 40), FONT, 0.7, Colors.WHITE, 1, cv2.LINE_AA)


def draw_bottom_bar(
    frame: np.ndarray,
    metrics: dict,
    time_sec: float,
    total_sec: float
) -> None:
    """Draw the bottom HUD bar with metrics and timer."""
    h, w = frame.shape[:2]
    
    # Semi-transparent background
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h - 60), (w, h), Colors.DARK_GREY, -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    # Timer
    timer_str = f"{int(time_sec)}s / {int(total_sec)}s"
    cv2.putText(frame, timer_str, (20, h - 20), FONT, 0.7, Colors.WHITE, 1, cv2.LINE_AA)
    
    # Metrics
    x_offset = 200
    for key, value in metrics.items():
        metric_str = f"{key}: {value}"
        cv2.putText(frame, metric_str, (x_offset, h - 20), FONT, 0.5, Colors.WHITE, 1, cv2.LINE_AA)
        x_offset += 180


def draw_caption(
    frame: np.ndarray,
    text: str,
    y_pos: int = 100
) -> None:
    """Draw a large centered caption."""
    h, w = frame.shape[:2]
    
    # Calculate text size for centering
    text_size = cv2.getTextSize(text, FONT, 1.2, 2)[0]
    x = (w - text_size[0]) // 2
    
    # Draw with shadow
    cv2.putText(frame, text, (x + 2, y_pos + 2), FONT, 1.2, Colors.BLACK, 3, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y_pos), FONT, 1.2, Colors.WHITE, 2, cv2.LINE_AA)


# ============================================================================
# ANIMATIONS
# ============================================================================

def ease_in_out(t: float) -> float:
    """Smooth ease-in-out interpolation."""
    return t * t * (3 - 2 * t)


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation."""
    return a + (b - a) * t


def draw_ghost_merge_animation(
    frame: np.ndarray,
    ghost_bbox: Tuple[int, int, int, int],
    canonical_bbox: Tuple[int, int, int, int],
    progress: float
) -> None:
    """
    Animate a ghost box merging into the canonical box.
    
    Args:
        frame: Image to draw on
        ghost_bbox: Starting ghost position
        canonical_bbox: Target canonical position
        progress: 0.0 to 1.0
    """
    t = ease_in_out(progress)
    
    # Interpolate position
    x1 = int(lerp(ghost_bbox[0], canonical_bbox[0], t))
    y1 = int(lerp(ghost_bbox[1], canonical_bbox[1], t))
    x2 = int(lerp(ghost_bbox[2], canonical_bbox[2], t))
    y2 = int(lerp(ghost_bbox[3], canonical_bbox[3], t))
    
    # Fade alpha
    alpha = 1.0 - (progress * 0.7)
    
    # Draw fading ghost
    draw_bbox(frame, (x1, y1, x2, y2), Colors.RED, dashed=True, alpha=alpha)
    
    # Pulse effect at end
    if progress >= 0.9:
        pulse = int((progress - 0.9) * 50)
        x1_p, y1_p = canonical_bbox[0] - pulse, canonical_bbox[1] - pulse
        x2_p, y2_p = canonical_bbox[2] + pulse, canonical_bbox[3] + pulse
        draw_bbox(frame, (x1_p, y1_p, x2_p, y2_p), Colors.GREEN, alpha=0.3)


def draw_rejection_stamp(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    text: str = "SIGNATURE INVALID"
) -> None:
    """Draw a rejection stamp over a malicious object."""
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    
    # Red X through the box
    cv2.line(frame, (x1, y1), (x2, y2), Colors.RED, 3)
    cv2.line(frame, (x2, y1), (x1, y2), Colors.RED, 3)
    
    # Stamp text
    text_size = cv2.getTextSize(text, FONT, 0.6, 2)[0]
    tx = cx - text_size[0] // 2
    ty = cy + text_size[1] // 2
    
    # Background for text
    cv2.rectangle(frame, (tx - 5, ty - text_size[1] - 5), 
                 (tx + text_size[0] + 5, ty + 5), Colors.RED, -1)
    cv2.putText(frame, text, (tx, ty), FONT, 0.6, Colors.WHITE, 2, cv2.LINE_AA)
