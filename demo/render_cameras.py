#!/usr/bin/env python3
"""
GodView Demo - Camera Renderer (The Cinematographer)
Replays CARLA recording and renders high-res frames from multiple camera angles.

Based on: carla.md Section 6.2
"""

import carla
import numpy as np
import cv2
import os
import time
import queue
import argparse

# Configuration
RECORDING_PATH = "/workspace/godview_demo/logs/godview_demo.log"
FRAMES_BASE_PATH = "/workspace/godview_demo/frames"

# Camera settings
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FOV = 90.0

# Render passes configuration
RENDER_PASSES = {
    "pass1_ego": {
        "description": "Ego-Chase (behind hero vehicle)",
        "type": "follow",
        "offset": carla.Location(x=-8.0, y=0.0, z=4.0),
        "rotation": carla.Rotation(pitch=-15.0, yaw=0.0, roll=0.0)
    },
    "pass2_drone": {
        "description": "Drone-View (overhead looking down)",
        "type": "fixed",
        "location": carla.Location(x=0.0, y=0.0, z=60.0),
        "rotation": carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
    },
    "pass3_cinematic": {
        "description": "Cinematic (sweeping intersection view)",
        "type": "animated",
        "keyframes": [
            {"frame": 0, "location": carla.Location(x=-50, y=-50, z=30), "rotation": carla.Rotation(pitch=-30, yaw=45, roll=0)},
            {"frame": 300, "location": carla.Location(x=50, y=-50, z=40), "rotation": carla.Rotation(pitch=-35, yaw=130, roll=0)},
            {"frame": 600, "location": carla.Location(x=50, y=50, z=25), "rotation": carla.Rotation(pitch=-25, yaw=220, roll=0)}
        ]
    }
}


def lerp(a, b, t):
    """Linear interpolation."""
    return a + (b - a) * t


def interpolate_transform(keyframes, frame):
    """Interpolate camera transform from keyframes."""
    # Find surrounding keyframes
    prev_kf = keyframes[0]
    next_kf = keyframes[-1]
    
    for i, kf in enumerate(keyframes):
        if kf["frame"] <= frame:
            prev_kf = kf
        if kf["frame"] >= frame and i < len(keyframes) - 1:
            next_kf = keyframes[i + 1]
            break
    
    if prev_kf["frame"] == next_kf["frame"]:
        t = 0.0
    else:
        t = (frame - prev_kf["frame"]) / (next_kf["frame"] - prev_kf["frame"])
    t = max(0.0, min(1.0, t))
    
    # Interpolate location
    loc = carla.Location(
        x=lerp(prev_kf["location"].x, next_kf["location"].x, t),
        y=lerp(prev_kf["location"].y, next_kf["location"].y, t),
        z=lerp(prev_kf["location"].z, next_kf["location"].z, t)
    )
    
    # Interpolate rotation
    rot = carla.Rotation(
        pitch=lerp(prev_kf["rotation"].pitch, next_kf["rotation"].pitch, t),
        yaw=lerp(prev_kf["rotation"].yaw, next_kf["rotation"].yaw, t),
        roll=lerp(prev_kf["rotation"].roll, next_kf["rotation"].roll, t)
    )
    
    return carla.Transform(loc, rot)


def create_camera(world, transform):
    """Create RGB camera sensor."""
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(CAMERA_WIDTH))
    bp.set_attribute("image_size_y", str(CAMERA_HEIGHT))
    bp.set_attribute("fov", str(CAMERA_FOV))
    bp.set_attribute("sensor_tick", "0.05")  # 20 FPS
    
    camera = world.spawn_actor(bp, transform)
    return camera


def save_frame(image, output_path):
    """Convert CARLA image to numpy array and save."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 4))
    array = array[:, :, :3]  # Remove alpha
    array = array[:, :, ::-1]  # BGRA to RGB
    cv2.imwrite(output_path, array)


def render_pass(client, world, pass_name, pass_config, output_dir, total_frames):
    """Render a single camera pass."""
    print(f"\n  Rendering {pass_name}: {pass_config['description']}")
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Setup image queue
    image_queue = queue.Queue()
    
    # Create initial camera
    if pass_config["type"] == "fixed":
        transform = carla.Transform(pass_config["location"], pass_config["rotation"])
    elif pass_config["type"] == "animated":
        transform = interpolate_transform(pass_config["keyframes"], 0)
    else:
        # Follow mode - start at origin, will be updated
        transform = carla.Transform(carla.Location(x=0, y=0, z=10), carla.Rotation(pitch=-15))
    
    camera = create_camera(world, transform)
    camera.listen(image_queue.put)
    
    # Start replay
    print(f"    Replaying {RECORDING_PATH}...")
    client.replay_file(RECORDING_PATH, 0.0, 0.0, 0, False)
    time.sleep(1.0)  # Wait for replay to initialize
    
    # Get hero vehicle for follow mode
    hero_vehicle = None
    if pass_config["type"] == "follow":
        for actor in world.get_actors().filter("vehicle.*"):
            hero_vehicle = actor
            break
    
    start_time = time.time()
    frame_count = 0
    
    try:
        while frame_count < total_frames:
            world.tick()
            
            # Update camera position
            if pass_config["type"] == "animated":
                new_transform = interpolate_transform(pass_config["keyframes"], frame_count)
                camera.set_transform(new_transform)
            elif pass_config["type"] == "follow" and hero_vehicle:
                hero_transform = hero_vehicle.get_transform()
                cam_location = hero_transform.location + hero_transform.transform(pass_config["offset"])
                camera.set_transform(carla.Transform(cam_location, pass_config["rotation"]))
            
            # Capture frame
            try:
                image = image_queue.get(timeout=2.0)
                frame_path = os.path.join(output_dir, f"{pass_name}_{frame_count:04d}.png")
                save_frame(image, frame_path)
                frame_count += 1
                
                if frame_count % 50 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    print(f"    Frame {frame_count}/{total_frames} ({fps:.1f} FPS)")
            
            except queue.Empty:
                print(f"    Warning: Queue empty at frame {frame_count}")
                frame_count += 1
    
    finally:
        camera.stop()
        camera.destroy()
    
    print(f"    Saved {frame_count} frames to {output_dir}")
    return frame_count


def main():
    parser = argparse.ArgumentParser(description="GodView Demo - Camera Renderer")
    parser.add_argument("--pass", dest="render_pass", default="all",
                        help="Which pass to render: pass1_ego, pass2_drone, pass3_cinematic, or all")
    parser.add_argument("--frames", type=int, default=600,
                        help="Number of frames to render")
    parser.add_argument("--quality", default="Epic",
                        help="Rendering quality: Low, Medium, High, Epic")
    args = parser.parse_args()
    
    separator = "=" * 60
    print(separator)
    print("GodView Demo - Camera Renderer")
    print(separator)
    
    # Connect to CARLA
    print("\n[1/3] Connecting to CARLA server...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    
    world = client.get_world()
    
    # Configure rendering quality
    print(f"[2/3] Setting quality to {args.quality}...")
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    
    # Determine which passes to render
    if args.render_pass == "all":
        passes_to_render = list(RENDER_PASSES.keys())
    else:
        passes_to_render = [args.render_pass]
    
    # Render each pass
    print(f"\n[3/3] Rendering {len(passes_to_render)} camera pass(es)...")
    
    total_rendered = 0
    for pass_name in passes_to_render:
        if pass_name not in RENDER_PASSES:
            print(f"  Unknown pass: {pass_name}, skipping")
            continue
        
        output_dir = os.path.join(FRAMES_BASE_PATH, pass_name)
        frames = render_pass(
            client, world, pass_name, RENDER_PASSES[pass_name],
            output_dir, args.frames
        )
        total_rendered += frames
    
    print(f"\n{separator}")
    print(f"COMPLETE! Rendered {total_rendered} total frames")
    print(f"  Output: {FRAMES_BASE_PATH}/")
    print(separator)


if __name__ == "__main__":
    main()
