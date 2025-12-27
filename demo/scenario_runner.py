#!/usr/bin/env python3
"""
GodView Demo - Scenario Runner (The Director)
Spawns vehicles, pedestrians, and simulated drones in CARLA Town10HD.
Records everything to a binary log for deterministic replay.

Based on: carla.md Section 6.1
"""

import carla
import random
import time
import json
import os

# Configuration
NUM_VEHICLES = 15
NUM_PEDESTRIANS = 10
NUM_DRONES = 3  # Simulated as walkers at altitude
DRONE_HEIGHTS = [15.0, 25.0, 35.0]  # Meters
SIMULATION_FRAMES = 600  # 30 seconds at 20 FPS
FIXED_DELTA = 0.05  # 20 FPS
VILLAIN_VEHICLE_INDEX = 3  # Which vehicle gets erratic behavior

# Output paths
RECORDING_PATH = "/workspace/godview_demo/logs/godview_demo.log"
GROUND_TRUTH_PATH = "/workspace/godview_demo/logs/ground_truth.ndjson"


def spawn_vehicles(world, blueprint_library, spawn_points, num_vehicles):
    """Spawn vehicles with autopilot enabled."""
    vehicles = []
    vehicle_bps = blueprint_library.filter("vehicle.*")
    
    for i in range(min(num_vehicles, len(spawn_points))):
        bp = random.choice(vehicle_bps)
        
        # Make villain vehicle bright red
        if i == VILLAIN_VEHICLE_INDEX and bp.has_attribute("color"):
            bp.set_attribute("color", "255,0,0")
        
        # Disable autopilot variation for consistent demo
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "true")
            
        spawn_point = spawn_points[i]
        vehicle = world.try_spawn_actor(bp, spawn_point)
        
        if vehicle is not None:
            vehicle.set_autopilot(True)
            vehicles.append({
                "actor": vehicle,
                "id": vehicle.id,
                "type": "vehicle",
                "is_villain": (i == VILLAIN_VEHICLE_INDEX),
                "model": bp.id
            })
            print(f"  Spawned vehicle {i}: {bp.id} (ID: {vehicle.id})")
    
    return vehicles


def spawn_pedestrians(world, blueprint_library, num_pedestrians):
    """Spawn pedestrians at random locations."""
    pedestrians = []
    walker_bps = blueprint_library.filter("walker.pedestrian.*")
    
    # Get random spawn locations
    spawn_points = []
    for _ in range(num_pedestrians * 2):  # Extra attempts
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            spawn_points.append(carla.Transform(loc))
    
    for i in range(min(num_pedestrians, len(spawn_points))):
        bp = random.choice(walker_bps)
        spawn_point = spawn_points[i]
        
        walker = world.try_spawn_actor(bp, spawn_point)
        if walker is not None:
            pedestrians.append({
                "actor": walker,
                "id": walker.id,
                "type": "pedestrian",
                "model": bp.id
            })
            print(f"  Spawned pedestrian {i}: {bp.id} (ID: {walker.id})")
    
    return pedestrians


def spawn_drones(world, blueprint_library, spawn_points, num_drones):
    """
    Simulate drones by spawning static meshes and teleporting them each tick.
    CARLA doesn't have native drone blueprints, so we use this workaround.
    """
    drones = []
    
    # Use a simple prop as drone placeholder
    drone_bp = blueprint_library.find("static.prop.streetbarrier")
    
    for i in range(min(num_drones, len(spawn_points))):
        # Start at ground level, will be teleported up
        base_transform = spawn_points[i % len(spawn_points)]
        drone = world.try_spawn_actor(drone_bp, base_transform)
        
        if drone is not None:
            height = DRONE_HEIGHTS[i % len(DRONE_HEIGHTS)]
            drones.append({
                "actor": drone,
                "id": drone.id,
                "type": "drone",
                "target_height": height,
                "base_x": base_transform.location.x,
                "base_y": base_transform.location.y
            })
            print(f"  Spawned drone {i} at height {height}m (ID: {drone.id})")
    
    return drones


def update_drones(drones, frame, world):
    """
    Move drones in circular patterns at their designated heights.
    This creates dynamic movement for the visualization.
    """
    for drone_data in drones:
        actor = drone_data["actor"]
        height = drone_data["target_height"]
        base_x = drone_data["base_x"]
        base_y = drone_data["base_y"]
        
        # Circular motion pattern
        radius = 15.0
        angular_speed = 0.02
        angle = frame * angular_speed
        
        new_x = base_x + radius * (1 - abs(((frame % 200) - 100) / 100))
        new_y = base_y + radius * 0.5 * (1 if (frame % 400) < 200 else -1)
        
        new_transform = carla.Transform(
            carla.Location(x=new_x, y=new_y, z=height),
            carla.Rotation(pitch=0, yaw=frame * 0.5, roll=0)
        )
        actor.set_transform(new_transform)


def apply_villain_jitter(vehicles, frame):
    """Apply erratic steering/throttle to the villain vehicle."""
    for v in vehicles:
        if v["is_villain"]:
            actor = v["actor"]
            # Random control noise every 10 frames
            if frame % 10 == 0:
                control = carla.VehicleControl(
                    throttle=random.uniform(0.3, 0.9),
                    steer=random.uniform(-0.3, 0.3),
                    brake=random.uniform(0.0, 0.2)
                )
                actor.apply_control(control)


def save_ground_truth(all_actors, frame, timestamp, gt_file):
    """Save ground truth positions to NDJSON for later processing."""
    for actor_data in all_actors:
        actor = actor_data["actor"]
        if not actor.is_alive:
            continue
            
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        
        packet = {
            "frame": frame,
            "timestamp": timestamp,
            "actor_id": actor_data["id"],
            "actor_type": actor_data["type"],
            "position": {
                "x": transform.location.x,
                "y": transform.location.y,
                "z": transform.location.z
            },
            "rotation": {
                "pitch": transform.rotation.pitch,
                "yaw": transform.rotation.yaw,
                "roll": transform.rotation.roll
            },
            "velocity": {
                "x": velocity.x,
                "y": velocity.y,
                "z": velocity.z
            },
            "is_villain": actor_data.get("is_villain", False),
            "is_drone": actor_data["type"] == "drone"
        }
        gt_file.write(json.dumps(packet) + "\n")


def main():
    separator = "=" * 60
    print(separator)
    print("GodView Demo - Scenario Runner")
    print(separator)
    
    # Connect to CARLA
    print("\n[1/6] Connecting to CARLA server...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    
    # Load Town10HD (high-fidelity urban map)
    print("[2/6] Loading Town10HD...")
    world = client.load_world("Town10HD")
    time.sleep(2)  # Wait for world to fully load
    
    # Get blueprints and spawn points
    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    
    # Enable synchronous mode for deterministic recording
    print("[3/6] Enabling synchronous mode...")
    settings = world.get_settings()
    original_settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA
    settings.no_rendering_mode = False  # We want visuals
    world.apply_settings(settings)
    
    # Spawn actors
    print("[4/6] Spawning actors...")
    print(f"  Target: {NUM_VEHICLES} vehicles, {NUM_PEDESTRIANS} pedestrians, {NUM_DRONES} drones")
    
    vehicles = spawn_vehicles(world, blueprint_library, spawn_points, NUM_VEHICLES)
    pedestrians = spawn_pedestrians(world, blueprint_library, NUM_PEDESTRIANS)
    drones = spawn_drones(world, blueprint_library, spawn_points[NUM_VEHICLES:], NUM_DRONES)
    
    all_actors = vehicles + pedestrians + drones
    print(f"  Total spawned: {len(all_actors)} actors")
    
    # Start CARLA recorder
    print("[5/6] Starting CARLA recorder...")
    os.makedirs(os.path.dirname(RECORDING_PATH), exist_ok=True)
    client.start_recorder(RECORDING_PATH, additional_data=True)
    print(f"  Recording to: {RECORDING_PATH}")
    
    # Open ground truth file
    gt_file = open(GROUND_TRUTH_PATH, "w")
    print(f"  Ground truth: {GROUND_TRUTH_PATH}")
    
    # Main simulation loop
    print(f"\n[6/6] Running simulation ({SIMULATION_FRAMES} frames = {SIMULATION_FRAMES * FIXED_DELTA:.1f}s)...")
    start_time = time.time()
    
    try:
        for frame in range(SIMULATION_FRAMES):
            # Tick the simulation
            world.tick()
            sim_time = frame * FIXED_DELTA
            
            # Update drone positions (manual teleportation)
            update_drones(drones, frame, world)
            
            # Apply villain behavior
            apply_villain_jitter(vehicles, frame)
            
            # Save ground truth
            save_ground_truth(all_actors, frame, sim_time, gt_file)
            
            # Progress update
            if frame % 100 == 0:
                elapsed = time.time() - start_time
                fps = frame / elapsed if elapsed > 0 else 0
                print(f"  Frame {frame}/{SIMULATION_FRAMES} ({fps:.1f} FPS)")
    
    except KeyboardInterrupt:
        print("\n  Interrupted by user!")
    
    finally:
        # Cleanup
        print("\n[CLEANUP] Stopping recorder and destroying actors...")
        client.stop_recorder()
        gt_file.close()
        
        for actor_data in all_actors:
            try:
                actor_data["actor"].destroy()
            except:
                pass
        
        # Restore original settings
        world.apply_settings(original_settings)
        
        elapsed = time.time() - start_time
        print(f"\n{separator}")
        print(f"COMPLETE! Recorded {SIMULATION_FRAMES} frames in {elapsed:.1f}s")
        print(f"  Recording: {RECORDING_PATH}")
        print(f"  Ground Truth: {GROUND_TRUTH_PATH}")
        print(separator)


if __name__ == "__main__":
    main()
