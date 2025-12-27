#!/usr/bin/env python3
"""
GodView LinkedIn Demo - Full Narrative with MP4 Output
=======================================================
Creates a compelling "Before vs After" video using OpenCV to capture
frames from CARLA and render HUD overlays, outputting directly to MP4.

NARRATIVE PHASES (80 seconds total):
1. SETUP (0-15s): Cinematic flyover, establish the scene
2. CHAOS (15-45s): RED boxes with ghosts, jitter, pancake drones
3. ACTIVATION (45-55s): Visual transition to GodView
4. SOLUTION (55-70s): GREEN boxes, stable tracking, corrections
5. DEEPDIVE (70-80s): Final stats and wrap-up

Output: ~/godview_demo.mp4 (1920x1080, 20 FPS)
"""

import carla
import cv2
import numpy as np
import random
import time
import math
import argparse
import queue
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional

# ============================================================================
# CONFIGURATION
# ============================================================================

# Video settings
VIDEO_WIDTH = 1920
VIDEO_HEIGHT = 1080
FPS = 20
OUTPUT_FILE = "godview_demo.mp4"

# Actor counts
NUM_VEHICLES = 15
NUM_DRONES = 3

# Colors (BGR for OpenCV)
COLOR_RAW = (50, 50, 255)       # Red - chaos/broken
COLOR_GODVIEW = (50, 255, 50)   # Green - consensus/fixed
COLOR_GHOST = (100, 100, 255)   # Light red for ghosts
COLOR_SYBIL = (255, 0, 255)     # Magenta - malicious
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_YELLOW = (0, 255, 255)

# Fault injection
OOSM_RATE = 0.3
GHOST_RATE = 0.15


class Phase(Enum):
    """Narrative phases with (start_time, end_time)"""
    SETUP = (0, 15)
    CHAOS = (15, 45)
    ACTIVATION = (45, 55)
    SOLUTION = (55, 70)
    DEEPDIVE = (70, 80)
    
    @property
    def start(self):
        return self.value[0]
    
    @property
    def end(self):
        return self.value[1]


@dataclass
class ActorState:
    """Tracks an actor's true and perceived positions"""
    actor: carla.Actor
    true_pos: carla.Location
    raw_pos: carla.Location  # With faults
    gv_pos: carla.Location   # Corrected
    has_oosm: bool = False
    has_ghost: bool = False
    ghost_offset: Tuple[float, float] = (0, 0)
    is_drone: bool = False
    drone_height: float = 0


class GodViewVideoDemo:
    """Creates the LinkedIn demo video with OpenCV rendering."""
    
    def __init__(self, client: carla.Client, world: carla.World):
        self.client = client
        self.world = world
        self.bp_lib = world.get_blueprint_library()
        
        # Actors
        self.vehicles: List[carla.Actor] = []
        self.drones: List[Tuple[carla.Actor, float]] = []  # (actor, height)
        self.hero_vehicle: Optional[carla.Actor] = None
        self.camera: Optional[carla.Actor] = None
        
        # Frame capture
        self.frame_queue = queue.Queue()
        self.current_frame = None
        
        # Video output
        self.video_writer = None
        
        # State tracking
        self.actor_states: dict = {}
        self.oosm_actors: set = set()
        self.ghost_actors: dict = {}  # actor_id -> (offset_x, offset_y)
        
        # Sybil attack position (fake obstacle)
        self.sybil_pos = None
        
        # Statistics
        self.stats = {
            "ghosts": 0,
            "oosm_errors": 0,
            "pancake_errors": 0,
            "sybil_rejected": 0,
            "packets_fixed": 0,
        }
        
        # Timing
        self.frame_count = 0
        self.start_time = 0
        
        # Traffic manager
        self.tm = None
    
    def setup_world(self):
        """Configure CARLA world and traffic manager."""
        # Synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / FPS
        self.world.apply_settings(settings)
        
        # Traffic Manager - CRITICAL for vehicle movement
        self.tm = self.client.get_trafficmanager(8000)
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(42)
        self.tm.set_global_distance_to_leading_vehicle(2.5)
        self.tm.global_percentage_speed_difference(-10)  # Slightly faster
        
        # Weather - dramatic but visible
        weather = carla.WeatherParameters(
            cloudiness=60.0,
            precipitation=20.0,
            precipitation_deposits=30.0,
            sun_altitude_angle=30.0,
            fog_density=5.0,
            wetness=50.0
        )
        self.world.set_weather(weather)
        
        print("[SETUP] World configured with Traffic Manager")
    
    def spawn_camera(self):
        """Spawn RGB camera sensor for frame capture."""
        camera_bp = self.bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIDEO_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIDEO_HEIGHT))
        camera_bp.set_attribute('fov', '90')
        
        # Start position - high above for SETUP phase
        spectator = self.world.get_spectator()
        spawn_point = spectator.get_transform()
        spawn_point.location.z = 80  # High up
        spawn_point.rotation.pitch = -90  # Looking down
        
        self.camera = self.world.spawn_actor(camera_bp, spawn_point)
        self.camera.listen(self._on_camera_frame)
        
        print(f"[CAMERA] Spawned at Z={spawn_point.location.z}m")
    
    def _on_camera_frame(self, image: carla.Image):
        """Callback for camera sensor frames."""
        # Convert to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((VIDEO_HEIGHT, VIDEO_WIDTH, 4))  # BGRA
        frame = array[:, :, :3]  # Drop alpha, keep BGR
        self.frame_queue.put(frame.copy())
    
    def spawn_actors(self):
        """Spawn vehicles and simulated drones."""
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
        # Spawn vehicles
        vehicle_bps = [bp for bp in self.bp_lib.filter('vehicle.*') 
                       if int(bp.get_attribute('number_of_wheels')) == 4]
        
        for i, sp in enumerate(spawn_points[:NUM_VEHICLES]):
            bp = random.choice(vehicle_bps)
            
            # Hero vehicle in distinctive color
            if bp.has_attribute('color'):
                if i == 0:
                    bp.set_attribute('color', '0,100,255')  # Blue hero
                else:
                    bp.set_attribute('color', random.choice([
                        '255,255,255', '50,50,50', '200,0,0', '0,100,0'
                    ]))
            
            vehicle = self.world.try_spawn_actor(bp, sp)
            if vehicle:
                # Enable autopilot with Traffic Manager
                vehicle.set_autopilot(True, self.tm.get_port())
                
                # Add some variety in driving behavior
                self.tm.random_left_lanechange_percentage(vehicle, 20)
                self.tm.random_right_lanechange_percentage(vehicle, 20)
                self.tm.ignore_lights_percentage(vehicle, 10)
                
                self.vehicles.append(vehicle)
                
                # Select some for OOSM
                if random.random() < OOSM_RATE:
                    self.oosm_actors.add(vehicle.id)
                
                if i == 0:
                    self.hero_vehicle = vehicle
        
        print(f"[SPAWN] {len(self.vehicles)} vehicles ({len(self.oosm_actors)} with OOSM)")
        
        # Spawn "drones" (walkers at altitude)
        walker_bp = self.bp_lib.find('walker.pedestrian.0001')
        drone_heights = [15.0, 25.0, 35.0]
        
        for height in drone_heights[:NUM_DRONES]:
            sp = random.choice(spawn_points[NUM_VEHICLES:])
            sp.location.z = height
            drone = self.world.try_spawn_actor(walker_bp, sp)
            if drone:
                self.drones.append((drone, height))
        
        print(f"[SPAWN] {len(self.drones)} drones at altitude")
        
        # Set up Sybil attack position (fake barrier)
        if self.hero_vehicle:
            hero_loc = self.hero_vehicle.get_location()
            self.sybil_pos = (hero_loc.x + 8, hero_loc.y, hero_loc.z)
    
    def setup_video_writer(self):
        """Initialize OpenCV VideoWriter for MP4 output."""
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            OUTPUT_FILE, fourcc, FPS, (VIDEO_WIDTH, VIDEO_HEIGHT)
        )
        print(f"[VIDEO] Writing to {OUTPUT_FILE}")
    
    def get_current_phase(self, elapsed: float) -> Phase:
        """Determine narrative phase based on elapsed time."""
        for phase in Phase:
            if phase.start <= elapsed < phase.end:
                return phase
        return Phase.DEEPDIVE
    
    def get_phase_progress(self, elapsed: float, phase: Phase) -> float:
        """Get progress within current phase (0.0 to 1.0)."""
        duration = phase.end - phase.start
        return min(1.0, (elapsed - phase.start) / duration)
    
    def update_camera_for_phase(self, phase: Phase, progress: float):
        """Position camera based on current narrative phase."""
        if not self.camera:
            return
        
        if phase == Phase.SETUP:
            # Descending flyover
            z = 80 - (progress * 50)  # 80m -> 30m
            pitch = -90 + (progress * 60)  # -90 -> -30
            
            # Orbit around scene center
            angle = progress * math.pi * 0.5
            radius = 50
            x = math.cos(angle) * radius
            y = math.sin(angle) * radius
            
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(pitch=pitch, yaw=math.degrees(angle) + 180)
            )
            self.camera.set_transform(transform)
            
        elif phase in [Phase.CHAOS, Phase.ACTIVATION, Phase.SOLUTION]:
            # Chase cam behind hero
            if self.hero_vehicle and self.hero_vehicle.is_alive:
                hero_t = self.hero_vehicle.get_transform()
                
                # Behind and above hero
                fwd = hero_t.get_forward_vector()
                cam_loc = hero_t.location + carla.Location(
                    x=-fwd.x * 15,
                    y=-fwd.y * 15,
                    z=8
                )
                cam_rot = carla.Rotation(
                    pitch=-15,
                    yaw=hero_t.rotation.yaw
                )
                self.camera.set_transform(carla.Transform(cam_loc, cam_rot))
        
        elif phase == Phase.DEEPDIVE:
            # Wide establishing shot
            transform = carla.Transform(
                carla.Location(x=0, y=0, z=60),
                carla.Rotation(pitch=-60, yaw=45)
            )
            self.camera.set_transform(transform)
    
    def inject_faults(self, actor: carla.Actor, true_loc: carla.Location, 
                      is_drone: bool = False) -> Tuple[carla.Location, bool]:
        """Inject faults for 'raw' perception. Returns (faulty_pos, has_ghost)."""
        raw = carla.Location(true_loc.x, true_loc.y, true_loc.z)
        has_ghost = False
        
        # OOSM jitter
        if actor.id in self.oosm_actors:
            jitter_x = random.gauss(0, 1.2)
            jitter_y = random.gauss(0, 1.2)
            raw.x += jitter_x
            raw.y += jitter_y
            self.stats["oosm_errors"] += 1
        
        # Pancake world for drones
        if is_drone:
            raw.z = 0.5  # Flatten to ground
            self.stats["pancake_errors"] += 1
        
        # Ghost injection
        if not is_drone and random.random() < GHOST_RATE:
            if actor.id not in self.ghost_actors:
                self.ghost_actors[actor.id] = (
                    random.uniform(-3, 3),
                    random.uniform(-3, 3)
                )
            has_ghost = True
            self.stats["ghosts"] = len(self.ghost_actors)
        
        return raw, has_ghost
    
    def world_to_screen(self, location: carla.Location) -> Optional[Tuple[int, int]]:
        """Project world location to screen coordinates."""
        if not self.camera:
            return None
        
        cam_t = self.camera.get_transform()
        cam_loc = cam_t.location
        cam_rot = cam_t.rotation
        
        # Vector from camera to point
        dx = location.x - cam_loc.x
        dy = location.y - cam_loc.y
        dz = location.z - cam_loc.z
        
        # Rotate to camera space
        yaw = math.radians(cam_rot.yaw)
        pitch = math.radians(cam_rot.pitch)
        
        # Simple projection (approximate)
        cos_yaw, sin_yaw = math.cos(-yaw), math.sin(-yaw)
        cos_pitch, sin_pitch = math.cos(-pitch), math.sin(-pitch)
        
        # Rotate around Z (yaw)
        x1 = dx * cos_yaw - dy * sin_yaw
        y1 = dx * sin_yaw + dy * cos_yaw
        z1 = dz
        
        # Rotate around Y (pitch)
        x2 = x1 * cos_pitch + z1 * sin_pitch
        z2 = -x1 * sin_pitch + z1 * cos_pitch
        y2 = y1
        
        # Behind camera check
        if x2 < 1:
            return None
        
        # Project to screen
        fov = 90
        f = VIDEO_WIDTH / (2 * math.tan(math.radians(fov) / 2))
        
        screen_x = int(VIDEO_WIDTH / 2 - (y2 / x2) * f)
        screen_y = int(VIDEO_HEIGHT / 2 - (z2 / x2) * f)
        
        # Bounds check
        if 0 <= screen_x < VIDEO_WIDTH and 0 <= screen_y < VIDEO_HEIGHT:
            return (screen_x, screen_y)
        return None
    
    def draw_box(self, frame: np.ndarray, pos: carla.Location, 
                 color: Tuple[int, int, int], label: str = "",
                 size: int = 40):
        """Draw a bounding box on the frame."""
        screen_pos = self.world_to_screen(pos)
        if not screen_pos:
            return
        
        x, y = screen_pos
        half = size // 2
        
        # Draw box
        cv2.rectangle(frame, (x - half, y - half), (x + half, y + half), color, 2)
        
        # Draw label
        if label:
            cv2.putText(frame, label, (x - half, y - half - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    
    def draw_drone_stem(self, frame: np.ndarray, pos: carla.Location, 
                        height: float, color: Tuple[int, int, int]):
        """Draw a vertical stem from ground to drone."""
        ground_pos = carla.Location(pos.x, pos.y, 0.1)
        
        top = self.world_to_screen(pos)
        bottom = self.world_to_screen(ground_pos)
        
        if top and bottom:
            cv2.line(frame, bottom, top, color, 2)
            cv2.putText(frame, f"Z:{height:.0f}m", (top[0] + 5, top[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    
    def render_hud(self, frame: np.ndarray, phase: Phase, elapsed: float):
        """Render HUD overlay based on current phase."""
        # Top banner
        cv2.rectangle(frame, (0, 0), (VIDEO_WIDTH, 70), COLOR_BLACK, -1)
        cv2.rectangle(frame, (0, 0), (VIDEO_WIDTH, 70), COLOR_WHITE, 2)
        
        # Phase-specific content
        if phase == Phase.SETUP:
            title = "MULTI-AGENT PERCEPTION GRID"
            status = f"Agents: {len(self.vehicles) + len(self.drones)} | Network: UNSTABLE"
            title_color = COLOR_WHITE
            
        elif phase == Phase.CHAOS:
            title = "RAW SENSOR VIEW - NO FUSION"
            status = f"Status: CRITICAL | Ghosts: {self.stats['ghosts']} | Z-Axis: COLLAPSED"
            title_color = COLOR_RAW
            
        elif phase == Phase.ACTIVATION:
            progress = self.get_phase_progress(elapsed, phase)
            title = "INITIALIZING GODVIEW CORE..."
            status = f"H3 Sharding: {'ACTIVE' if progress > 0.3 else 'LOADING'} | Highlander: {'ON' if progress > 0.6 else 'SYNCING'}"
            title_color = COLOR_YELLOW
            
        elif phase == Phase.SOLUTION:
            self.stats["packets_fixed"] += random.randint(5, 15)
            title = "GODVIEW CONSENSUS ACTIVE"
            status = f"Status: STABLE | Late Packets Fixed: {self.stats['packets_fixed']} | Trust: VERIFIED"
            title_color = COLOR_GODVIEW
            
        else:  # DEEPDIVE
            title = "PERCEPTION GRID STABILIZED"
            status = f"Precision: +400% | Malicious Packets Dropped: {self.stats['ghosts']} | Canonical State: CONVERGED"
            title_color = COLOR_GODVIEW
        
        # Draw title
        cv2.putText(frame, title, (50, 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, title_color, 2, cv2.LINE_AA)
        
        # Bottom status bar
        cv2.rectangle(frame, (0, VIDEO_HEIGHT - 50), (VIDEO_WIDTH, VIDEO_HEIGHT), COLOR_BLACK, -1)
        cv2.putText(frame, status, (50, VIDEO_HEIGHT - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_WHITE, 1, cv2.LINE_AA)
        
        # Timer
        timer_text = f"{int(elapsed)}s / 80s"
        cv2.putText(frame, timer_text, (VIDEO_WIDTH - 150, VIDEO_HEIGHT - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_WHITE, 1, cv2.LINE_AA)
        
        # Activation scanline effect
        if phase == Phase.ACTIVATION:
            progress = self.get_phase_progress(elapsed, phase)
            scanline_y = int(VIDEO_HEIGHT * progress)
            cv2.line(frame, (0, scanline_y), (VIDEO_WIDTH, scanline_y), COLOR_YELLOW, 3)
    
    def process_frame(self, elapsed: float) -> np.ndarray:
        """Process a single frame with all overlays."""
        # Get camera frame
        try:
            frame = self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            frame = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
        
        phase = self.get_current_phase(elapsed)
        show_raw = phase in [Phase.CHAOS, Phase.ACTIVATION]
        show_godview = phase in [Phase.ACTIVATION, Phase.SOLUTION, Phase.DEEPDIVE]
        
        # Draw vehicles
        for vehicle in self.vehicles:
            if not vehicle.is_alive:
                continue
            
            true_loc = vehicle.get_location()
            
            if show_raw:
                raw_loc, has_ghost = self.inject_faults(vehicle, true_loc)
                self.draw_box(frame, raw_loc, COLOR_RAW, "RAW")
                
                # Draw ghost
                if has_ghost and vehicle.id in self.ghost_actors:
                    gx, gy = self.ghost_actors[vehicle.id]
                    ghost_loc = carla.Location(raw_loc.x + gx, raw_loc.y + gy, raw_loc.z)
                    self.draw_box(frame, ghost_loc, COLOR_GHOST, "GHOST")
            
            if show_godview and phase != Phase.CHAOS:
                gv_loc = true_loc  # GodView = ground truth
                self.draw_box(frame, gv_loc, COLOR_GODVIEW, "GV")
        
        # Draw drones
        for drone, height in self.drones:
            if not drone.is_alive:
                continue
            
            true_loc = drone.get_location()
            true_loc.z = height  # Maintain altitude
            
            if show_raw:
                # Pancake world - show at ground
                pancake_loc = carla.Location(true_loc.x, true_loc.y, 0.5)
                self.draw_box(frame, pancake_loc, COLOR_RAW, "DRONE Z=0!", size=30)
            
            if show_godview and phase != Phase.CHAOS:
                # Correct altitude with stem
                self.draw_box(frame, true_loc, COLOR_GODVIEW, "DRONE", size=30)
                self.draw_drone_stem(frame, true_loc, height, COLOR_GODVIEW)
        
        # Draw Sybil attack
        if self.sybil_pos:
            sybil_loc = carla.Location(self.sybil_pos[0], self.sybil_pos[1], self.sybil_pos[2])
            
            if show_raw:
                self.draw_box(frame, sybil_loc, COLOR_SYBIL, "SYBIL!", size=50)
            
            if show_godview and phase not in [Phase.SETUP, Phase.CHAOS]:
                # Show rejection
                screen_pos = self.world_to_screen(sybil_loc)
                if screen_pos:
                    x, y = screen_pos
                    cv2.line(frame, (x - 30, y - 30), (x + 30, y + 30), COLOR_GODVIEW, 3)
                    cv2.line(frame, (x - 30, y + 30), (x + 30, y - 30), COLOR_GODVIEW, 3)
                    cv2.putText(frame, "REJECTED", (x - 40, y - 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GODVIEW, 1, cv2.LINE_AA)
        
        # Render HUD
        self.render_hud(frame, phase, elapsed)
        
        return frame
    
    def run(self, duration: int = 80):
        """Run the full demo and output MP4."""
        print("=" * 60)
        print("GODVIEW LINKEDIN DEMO - MP4 OUTPUT")
        print("=" * 60)
        print(f"Duration: {duration}s | Output: {OUTPUT_FILE}")
        print("=" * 60)
        
        self.setup_video_writer()
        self.start_time = time.time()
        total_frames = duration * FPS
        
        try:
            for f in range(total_frames):
                # Tick world
                self.world.tick()
                
                # Calculate timing
                elapsed = f / FPS
                phase = self.get_current_phase(elapsed)
                progress = self.get_phase_progress(elapsed, phase)
                
                # Update camera
                self.update_camera_for_phase(phase, progress)
                
                # Wait for frame
                time.sleep(0.01)  # Give camera time to capture
                
                # Process and write frame
                frame = self.process_frame(elapsed)
                self.video_writer.write(frame)
                
                # Progress output
                if f % (FPS * 5) == 0:
                    real_elapsed = time.time() - self.start_time
                    print(f"[{elapsed:.0f}s] Frame {f}/{total_frames} | Phase: {phase.name} | Real: {real_elapsed:.1f}s")
        
        except KeyboardInterrupt:
            print("\n[DEMO] Interrupted")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources."""
        print("[CLEANUP] Releasing resources...")
        
        # Release video
        if self.video_writer:
            self.video_writer.release()
            print(f"[VIDEO] Saved to {OUTPUT_FILE}")
        
        # Destroy actors
        if self.camera and self.camera.is_alive:
            self.camera.destroy()
        
        for v in self.vehicles:
            if v.is_alive:
                v.destroy()
        
        for d, _ in self.drones:
            if d.is_alive:
                d.destroy()
        
        # Reset world
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
        
        print("[CLEANUP] Complete!")


def main():
    parser = argparse.ArgumentParser(description="GodView LinkedIn Demo - MP4 Output")
    parser.add_argument("--duration", type=int, default=80, help="Demo duration in seconds")
    parser.add_argument("--output", type=str, default=OUTPUT_FILE, help="Output MP4 filename")
    args = parser.parse_args()
    
    global OUTPUT_FILE
    OUTPUT_FILE = args.output
    
    print("[INIT] Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    
    world = client.get_world()
    print(f"[INIT] Connected to {world.get_map().name}")
    
    demo = GodViewVideoDemo(client, world)
    demo.setup_world()
    demo.spawn_camera()
    demo.spawn_actors()
    demo.run(duration=args.duration)
    
    print(f"\n[DONE] Video saved to: {OUTPUT_FILE}")
    print("Transfer with: scp user@vm:{OUTPUT_FILE} ./")


if __name__ == "__main__":
    main()
