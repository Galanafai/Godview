#!/usr/bin/env python3
"""
GodView LinkedIn Demo - Visual Storytelling
============================================
This script creates a compelling "Before vs After" visualization directly in CARLA,
showing GodView's ability to solve real perception problems.

NARRATIVE BEATS (per hmm.md):
1. THE CHAOS (0:00-0:30): Red boxes showing ghosts, pancake drones, jittering
2. GODVIEW ACTIVATION (0:30-0:35): Visual transition pulse
3. THE SOLUTION (0:35-1:00): Green boxes, stable tracking, rejected attacks

Run this while CARLA is visible on screen and screen record!
"""

import carla
import random
import time
import math
import argparse
from collections import defaultdict

# ============================================================================
# CONFIGURATION
# ============================================================================

# Simulation settings
NUM_VEHICLES = 12
NUM_PEDESTRIANS = 6
NUM_DRONES = 3
SIMULATION_DURATION = 60  # seconds
FPS = 20

# Visualization colors (RGBA)
COLOR_RAW = carla.Color(255, 50, 50, 255)      # RED - chaos/broken
COLOR_GODVIEW = carla.Color(50, 255, 50, 255)  # GREEN - consensus/fixed
COLOR_GHOST = carla.Color(255, 100, 100, 150)  # Transparent red for ghosts
COLOR_SYBIL = carla.Color(255, 0, 255, 255)    # Magenta - malicious
COLOR_DRONE_STEM = carla.Color(100, 200, 255, 255)  # Cyan for altitude line
COLOR_TEXT = carla.Color(255, 255, 255, 255)   # White for labels

# Fault injection rates
OOSM_RATE = 0.3        # 30% of vehicles get latency jitter
GHOST_RATE = 0.15      # 15% chance of ghost per frame
PANCAKE_DRONES = True  # Flatten drones in "before" view


class GodViewVisualDemo:
    """
    Creates a live visualization of GodView's perception fusion capabilities.
    Uses CARLA's debug drawing to show Before/After states simultaneously.
    """
    
    def __init__(self, client, world):
        self.client = client
        self.world = world
        self.debug = world.debug
        self.vehicles = []
        self.pedestrians = []
        self.drones = []
        self.hero_vehicle = None
        
        # Tracking state
        self.actor_positions = {}  # True positions
        self.raw_positions = {}    # "Before" - with faults injected
        self.godview_positions = {}  # "After" - corrected
        
        # Fault state
        self.oosm_actors = set()   # Actors with latency
        self.ghost_actors = {}     # {actor_id: ghost_offset}
        self.sybil_injection = None
        
        # Demo phase
        self.phase = "CHAOS"  # CHAOS -> TRANSITION -> CONSENSUS
        self.frame = 0
        self.transition_frame = None
        
        # Stats for HUD
        self.stats = {
            "ghosts_active": 0,
            "oosm_packets": 0,
            "pancake_errors": 0,
            "sybil_rejected": 0,
            "latency_corrected": 0,
        }
    
    def setup_world(self):
        """Configure world settings for the demo."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / FPS
        self.world.apply_settings(settings)
        
        # Set nice weather for cinematic look
        weather = carla.WeatherParameters(
            cloudiness=40.0,
            precipitation=0.0,
            sun_altitude_angle=45.0,
            fog_density=5.0,
            wetness=20.0
        )
        self.world.set_weather(weather)
        print("[SETUP] World configured for demo")
    
    def spawn_actors(self):
        """Spawn vehicles, pedestrians, and simulated drones."""
        bp_lib = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
        # Spawn vehicles
        vehicle_bps = bp_lib.filter('vehicle.*')
        for i, sp in enumerate(spawn_points[:NUM_VEHICLES]):
            bp = random.choice(vehicle_bps)
            if bp.has_attribute('color'):
                # Make hero vehicle distinctive (bright blue)
                if i == 0:
                    bp.set_attribute('color', '0,100,255')
                else:
                    bp.set_attribute('color', random.choice(['255,255,255', '50,50,50', '200,0,0']))
            
            vehicle = self.world.try_spawn_actor(bp, sp)
            if vehicle:
                vehicle.set_autopilot(True)
                self.vehicles.append(vehicle)
                
                # Select some for OOSM (latency injection)
                if random.random() < OOSM_RATE:
                    self.oosm_actors.add(vehicle.id)
                
                if i == 0:
                    self.hero_vehicle = vehicle
        
        print(f"[SPAWN] {len(self.vehicles)} vehicles ({len(self.oosm_actors)} with OOSM)")
        
        # Spawn "drones" (walkers teleported to altitude)
        walker_bp = bp_lib.find('walker.pedestrian.0001')
        drone_heights = [15.0, 25.0, 35.0]
        
        for i, height in enumerate(drone_heights[:NUM_DRONES]):
            sp = random.choice(spawn_points)
            sp.location.z = height
            drone = self.world.try_spawn_actor(walker_bp, sp)
            if drone:
                self.drones.append((drone, height))
        
        print(f"[SPAWN] {len(self.drones)} drones at various altitudes")
        
        # Create a sybil attack injection (fake obstacle)
        self.sybil_injection = {
            "position": carla.Location(
                x=spawn_points[0].location.x + 5,
                y=spawn_points[0].location.y,
                z=0.5
            ),
            "label": "SYBIL ATTACK!",
            "active": True
        }
        print("[SPAWN] Sybil injection prepared")
    
    def get_raw_position(self, actor, true_location, is_drone=False, drone_height=0):
        """
        Generate "raw sensor" position with injected faults.
        This represents what a broken perception system would see.
        """
        raw = carla.Location(true_location.x, true_location.y, true_location.z)
        
        # FAULT 1: OOSM Jitter (latency causes position lag/jump)
        if actor.id in self.oosm_actors:
            jitter = random.gauss(0, 0.8)  # +/- 0.8m jitter
            raw.x += jitter
            raw.y += jitter * random.choice([-1, 1])
            self.stats["oosm_packets"] += 1
        
        # FAULT 2: Pancake World (drones flattened to ground)
        if is_drone and PANCAKE_DRONES:
            raw.z = 0.5  # Force to ground level
            self.stats["pancake_errors"] += 1
        
        # FAULT 3: Ghost injection (duplicate bounding box)
        if random.random() < GHOST_RATE and not is_drone:
            if actor.id not in self.ghost_actors:
                self.ghost_actors[actor.id] = carla.Location(
                    x=random.uniform(-3, 3),
                    y=random.uniform(-3, 3),
                    z=0
                )
            self.stats["ghosts_active"] = len(self.ghost_actors)
        
        return raw
    
    def get_godview_position(self, actor, true_location, is_drone=False, drone_height=0):
        """
        Generate "GodView corrected" position.
        This is what GodView's fusion produces - smooth, accurate, verified.
        """
        corrected = carla.Location(true_location.x, true_location.y, true_location.z)
        
        # GodView corrects OOSM via AugmentedStateFilter resequencing
        # (positions are smooth, no jitter)
        if actor.id in self.oosm_actors:
            self.stats["latency_corrected"] += 1
        
        # GodView corrects Pancake via H3 voxel grid
        # (drones show at correct altitude)
        if is_drone:
            corrected.z = drone_height
        
        # Ghosts are merged via Highlander consensus
        # (no duplicate boxes drawn in "After" view)
        
        return corrected
    
    def draw_bounding_box(self, location, color, size=(2.0, 4.5, 1.8), label=None, is_drone=False, draw_stem=False, stem_height=0):
        """Draw a 3D bounding box at the given location."""
        # Create box corners
        half_w = size[0] / 2
        half_l = size[1] / 2
        half_h = size[2] / 2
        
        box = carla.BoundingBox(location, carla.Vector3D(half_w, half_l, half_h))
        rotation = carla.Rotation(0, 0, 0)
        
        self.debug.draw_box(box, rotation, thickness=0.1, color=color, life_time=0.08)
        
        # Draw altitude stem for drones
        if draw_stem and stem_height > 0:
            ground_point = carla.Location(location.x, location.y, 0.1)
            self.debug.draw_line(ground_point, location, thickness=0.05, 
                               color=COLOR_DRONE_STEM, life_time=0.08)
            self.debug.draw_string(
                carla.Location(location.x, location.y, stem_height / 2),
                f"Z: {stem_height:.0f}m",
                color=COLOR_DRONE_STEM,
                life_time=0.08
            )
        
        # Draw label
        if label:
            label_loc = carla.Location(location.x, location.y, location.z + half_h + 0.5)
            self.debug.draw_string(label_loc, label, color=color, life_time=0.08)
    
    def draw_hud(self, spectator_location):
        """Draw status information in the world."""
        # Draw phase indicator
        if self.phase == "CHAOS":
            phase_text = "RAW SENSOR FEED - NO FUSION"
            status_text = "STATUS: CRITICAL"
            phase_color = COLOR_RAW
        elif self.phase == "TRANSITION":
            phase_text = "ACTIVATING GODVIEW CORE..."
            status_text = "INITIALIZING"
            phase_color = carla.Color(255, 255, 0, 255)
        else:
            phase_text = "GODVIEW CONSENSUS ACTIVE"
            status_text = "STATUS: STABLE"
            phase_color = COLOR_GODVIEW
        
        # Draw phase info above scene
        hud_location = carla.Location(
            spectator_location.x,
            spectator_location.y - 30,
            spectator_location.z + 10
        )
        
        # Stats text
        if self.phase == "CHAOS":
            stats_text = f"Ghosts: {self.stats['ghosts_active']} | OOSM Errors: {self.stats['oosm_packets']} | Pancake: {len(self.drones)}"
        else:
            stats_text = f"Highlander Merges: {self.stats['ghosts_active']} | Late Packets Fixed: {self.stats['latency_corrected']}"
        
        self.debug.draw_string(hud_location, phase_text, color=phase_color, life_time=0.08)
        
    def update_frame(self):
        """Main frame update - draws all visualizations."""
        self.frame += 1
        frame_time = self.frame / FPS
        
        # Phase transitions
        if frame_time > 25 and self.phase == "CHAOS":
            self.phase = "TRANSITION"
            self.transition_frame = self.frame
            print("[DEMO] >>> TRANSITION: Activating GodView <<<")
        
        if self.transition_frame and self.frame > self.transition_frame + FPS * 3:
            self.phase = "CONSENSUS"
            print("[DEMO] >>> CONSENSUS: GodView Active <<<")
        
        # Get spectator position for HUD
        spectator = self.world.get_spectator()
        spec_transform = spectator.get_transform()
        
        # Draw HUD
        self.draw_hud(spec_transform.location)
        
        # Visualize vehicles
        for vehicle in self.vehicles:
            if not vehicle.is_alive:
                continue
                
            true_loc = vehicle.get_location()
            
            if self.phase in ["CHAOS", "TRANSITION"]:
                # Draw RAW (broken) position
                raw_loc = self.get_raw_position(vehicle, true_loc)
                self.draw_bounding_box(raw_loc, COLOR_RAW, label="RAW")
                
                # Draw ghosts
                if vehicle.id in self.ghost_actors:
                    ghost_offset = self.ghost_actors[vehicle.id]
                    ghost_loc = carla.Location(
                        raw_loc.x + ghost_offset.x,
                        raw_loc.y + ghost_offset.y,
                        raw_loc.z
                    )
                    self.draw_bounding_box(ghost_loc, COLOR_GHOST, label="GHOST")
            
            else:
                # Draw GODVIEW (corrected) position
                gv_loc = self.get_godview_position(vehicle, true_loc)
                self.draw_bounding_box(gv_loc, COLOR_GODVIEW, label="GV")
        
        # Visualize drones
        for drone, true_height in self.drones:
            if not drone.is_alive:
                continue
                
            true_loc = drone.get_location()
            true_loc.z = true_height  # Maintain simulated altitude
            
            if self.phase in ["CHAOS", "TRANSITION"]:
                # Pancake world - show drone on ground
                raw_loc = self.get_raw_position(drone, true_loc, is_drone=True, drone_height=true_height)
                self.draw_bounding_box(raw_loc, COLOR_RAW, size=(1.0, 1.0, 0.5), 
                                      label=f"DRONE (Z=0!)", is_drone=True)
            else:
                # GodView - show correct altitude with stem
                gv_loc = self.get_godview_position(drone, true_loc, is_drone=True, drone_height=true_height)
                self.draw_bounding_box(gv_loc, COLOR_GODVIEW, size=(1.0, 1.0, 0.5),
                                      label=f"DRONE", is_drone=True, 
                                      draw_stem=True, stem_height=true_height)
        
        # Visualize Sybil attack
        if self.sybil_injection and self.sybil_injection["active"]:
            sybil_loc = self.sybil_injection["position"]
            
            if self.phase in ["CHAOS", "TRANSITION"]:
                # Show malicious injection as valid obstacle
                self.draw_bounding_box(sybil_loc, COLOR_SYBIL, size=(2.0, 2.0, 2.0),
                                      label="SYBIL ATTACK!")
            else:
                # GodView rejects it - show as crossed out
                self.debug.draw_string(
                    carla.Location(sybil_loc.x, sybil_loc.y, sybil_loc.z + 3),
                    "REJECTED: Invalid Ed25519 Signature",
                    color=COLOR_GODVIEW,
                    life_time=0.08
                )
    
    def follow_hero(self):
        """Move spectator to follow hero vehicle."""
        if self.hero_vehicle and self.hero_vehicle.is_alive:
            transform = self.hero_vehicle.get_transform()
            spectator = self.world.get_spectator()
            
            # Position camera behind and above
            back_offset = transform.get_forward_vector() * -12
            up_offset = carla.Location(0, 0, 6)
            
            cam_location = transform.location + carla.Location(back_offset.x, back_offset.y, 0) + up_offset
            cam_rotation = carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
            
            spectator.set_transform(carla.Transform(cam_location, cam_rotation))
    
    def run(self, duration=SIMULATION_DURATION, follow=True):
        """Run the visualization demo."""
        print("=" * 60)
        print("GODVIEW LINKEDIN DEMO - VISUAL STORYTELLING")
        print("=" * 60)
        print(f"Duration: {duration} seconds")
        print("RECORDING TIP: Screen record the CARLA window now!")
        print("=" * 60)
        
        total_frames = duration * FPS
        start_time = time.time()
        
        try:
            for f in range(total_frames):
                self.world.tick()
                self.update_frame()
                
                if follow:
                    self.follow_hero()
                
                if f % (FPS * 5) == 0:  # Every 5 seconds
                    elapsed = time.time() - start_time
                    print(f"[{elapsed:.1f}s] Frame {f}/{total_frames} | Phase: {self.phase}")
        
        except KeyboardInterrupt:
            print("\n[DEMO] Interrupted by user")
        
        finally:
            print("[DEMO] Complete!")
            self.cleanup()
    
    def cleanup(self):
        """Destroy spawned actors."""
        print("[CLEANUP] Destroying actors...")
        for v in self.vehicles:
            if v.is_alive:
                v.destroy()
        for d, _ in self.drones:
            if d.is_alive:
                d.destroy()
        
        # Reset world settings
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)


def main():
    parser = argparse.ArgumentParser(description="GodView LinkedIn Visual Demo")
    parser.add_argument("--duration", type=int, default=60, help="Demo duration in seconds")
    parser.add_argument("--no-follow", action="store_true", help="Don't auto-follow hero vehicle")
    args = parser.parse_args()
    
    # Connect to CARLA
    print("[INIT] Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    
    world = client.get_world()
    
    # Create and run demo
    demo = GodViewVisualDemo(client, world)
    demo.setup_world()
    demo.spawn_actors()
    demo.run(duration=args.duration, follow=not args.no_follow)


if __name__ == "__main__":
    main()
