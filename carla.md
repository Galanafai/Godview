Project GodView: Cloud-Based Simulation & Orchestration Report
1. Executive Summary
1.1 Objective and Strategic Intent
This report outlines a comprehensive architectural and operational strategy for the production of a high-fidelity technical demonstration video for GodView_core. The primary objective is to visually validate the system’s distributed perception capabilities—specifically its resilience to latency, out-of-sequence measurements (OOSM), vertical spatial ambiguity, identity conflicts, and malicious data injection—within a simulated adversarial environment.
The target deliverable is a 1–2 minute "cinematic" video suitable for professional networks (LinkedIn). This video must transcend typical debugging visualizations by presenting a stark "Before vs. After" narrative: a chaotic "Before" state characterized by sensor ghosts, teleportation artifacts, and security breaches, contrasted with a stable, canonical "After" state managed by GodView.
To achieve this without local high-end hardware, the simulation will be orchestrated using the CARLA Simulator on RunPod cloud infrastructure. The strategy prioritizes a "Record Once, Render Many" workflow to decouple the computational cost of simulation physics from the graphical cost of rendering, ensuring broadcast-quality visuals at a minimal budget.
1.2 The Narrative Arc: From Entropy to Consensus
The technical storytelling relies on making the invisible visible. Perception algorithms like Covariance Intersection (CI) or Augmented State Extended Kalman Filters (EKF) are abstract mathematical concepts. To demonstrate their value, the simulation must visually manifest the errors they correct.
The Baseline (Entropy): A raw sensor feed where network latency creates "rubber-banding" cars, lack of altitude awareness flattens drones into pavement-crawling glitches, and spoofed identities create flickering "ghost" fleets.
The Resolution (Consensus): The activation of GodView. The narrative visualizes the system "locking" onto a canonical state. The ghosts vanish via Highlander convergence, the drones "pop" to their correct altitude via voxel hashing, and the jitter smooths into predictive tracks via the EKF replay buffer.
1.3 Infrastructure & Methodology Snapshot
Simulation Core: CARLA 0.9.13 deployed in a persistent Docker container on RunPod.
Compute Strategy: NVIDIA RTX 4090 instances utilizing "Secure Cloud" pods for reliability, leveraging the card's high rasterization throughput for off-screen rendering.
Execution Workflow: Asynchronous decoupling. The simulation runs in a lightweight headless mode to generate a binary log. This log is then replayed multiple times from different camera angles (Top-down, Drone-follow, Ego-chase) to generate high-resolution frames, which are composited offline.
Visualization: A hybrid approach overlaying CARLA’s photorealistic rendering with a custom, Python-generated "Heads-Up Display" (HUD) that visualizes the GodView state changes (Green = Trusted/Fused, Red = Rejected/Raw).
2. Theoretical Framework: The Engineering "Why"
Before detailing the how, it is critical to articulate the why. The simulation design is dictated by the specific failure modes GodView addresses. Understanding these mathematical underpinnings ensures the visual artifacts we generate are technically accurate representations of real-world failures, not just random noise.
2.1 Latency and Out-of-Sequence Measurements (OOSM)
In distributed multi-agent networks, data packets rarely arrive in perfect chronological order. Variable network latency, processing delays, and packet loss lead to OOSM.
The Problem: A standard Kalman Filter expects measurements to arrive sequentially ($t_k, t_{k+1}, t_{k+2}$). If a packet from $t_{k-1}$ arrives after $t_{k+2}$ has been processed, a naive system discards it (data loss) or forces it in as a current measurement (state corruption), causing the tracked object to "teleport" backward.
GodView’s Solution: An Augmented State EKF maintains a sliding window of historical states. When a late packet arrives, the filter "rewinds" to the correct timestamp, applies the update, and re-propagates the state forward.
Visual Manifestation: In the "Before" view, cars should jitter and jump. In the "After" view, they should move smoothly, perhaps with a visual trail indicating the "corrected" path versus the "reported" path.
2.2 The "Pancake World" and Verticality
Most autonomous driving stacks rely on 2D Bird's Eye View (BEV) projections because cars live on the ground. This assumption fails in multi-domain operations involving drones or bridges.
The Problem: When a 3D object (drone) is projected onto a 2D grid without altitude partitioning, it occupies the same $(x, y)$ coordinates as objects on the ground. This leads to "phantom collisions" where the system believes a drone at 50m altitude is inside a truck at 0m.
GodView’s Solution: H3 global sharding combined with a local sparse 3D voxel hash. This allows efficient neighbor queries that respect altitude ($z$-axis).
Visual Manifestation: The "Before" view must show drones flattened onto the road, overlapping with cars. The "After" view demonstrates the "pop" as the voxel hash activates, separating the layers.
2.3 Identity Conflicts and the "Highlander" Principle
Distributed systems lack a central authority to assign unique IDs. Two agents seeing the same object will assign it different local IDs (e.g., Agent A sees Obj_1, Agent B sees Obj_99).
The Problem: Without reconciliation, the global map shows two objects occupying the same space ("ghosting").
GodView’s Solution: Global Nearest Neighbor (GNN) association combined with Mahalanobis gating (Chi-squared test) determines if Obj_1 and Obj_99 are statistically the same object. If they match, the "Highlander" heuristic (min-UUID wins) deterministically merges them into a single canonical ID.
Visual Manifestation: The "Before" view is crowded with flickering, overlapping bounding boxes. The "After" view stabilizes to a single, solid box per vehicle.
3. Cloud Infrastructure & GPU Orchestration (RunPod)
Running high-fidelity simulations like CARLA in the cloud presents specific challenges regarding GPU passthrough, OpenGL/Vulkan driver compatibility, and display server management (headless rendering). RunPod offers a flexible environment, but selecting the wrong instance type or virtualization model can lead to wasted budget or rendering artifacts.
3.1 Instance Selection and Performance Analysis
CARLA is fundamentally a video game (Unreal Engine 4/5). Unlike pure Deep Learning training workloads which rely on Tensor Cores and high memory bandwidth (A100/H100), CARLA relies heavily on Rasterization performance and Clock Speeds. Enterprise AI cards like the A100 are often less cost-effective for rendering than "Pro-Viz" or high-end consumer cards because they lack display outputs and require specialized grid drivers for OpenGL/Vulkan contexts, though they can work with headless configurations.
Recommended Tiers and Justification
The following table breaks down the recommended GPU instances on RunPod, analyzing their suitability for a 20-agent CARLA simulation with rendering requirements.

Tier
GPU Model
VRAM
RAM
Approx. Cost
Rationale for Selection
Recommended
RTX 4090
24 GB
~128 GB
$0.44 - $0.79 / hr
The RTX 4090 is the current king of rasterization throughput. Its high clock speed benefits Unreal Engine's main thread (which is often the bottleneck). 24GB VRAM is sufficient for CARLA's Town10HD map with 20 agents, provided we don't use excessive high-res textures. The cost-to-performance ratio for rendering is unbeatable. 1
High Memory
RTX A6000
48 GB
~120+ GB
$0.79 - $0.84 / hr
If the simulation crashes due to VRAM limits (OOM errors)—which can happen if you enable multiple RGB cameras per agent—the A6000 provides a massive 48GB buffer. It is a workstation card, offering higher stability but slightly lower raw clock speeds than the 4090. 3
Budget
RTX 3090
24 GB
~60 GB
$0.22 - $0.34 / hr
A viable option for strict budgets. It offers similar VRAM to the 4090 but with slower rendering speeds. It is adequate for capturing 1080p footage but may struggle with 4K "Epic" settings at 60 FPS. 1

Strategic Recommendation: Rent a Secure Cloud RTX 4090 Pod. The "Secure Cloud" tier generally offers better reliability, higher uptime guarantees, and faster download speeds compared to the "Community Cloud." The slightly higher cost (approx. $0.40 difference per hour) is negligible for a short-term project (5-10 hours) and pays for itself by avoiding the frustration of random instance preemption or slow data egress.
3.2 Pods vs. Serverless: The Persistence Imperative
Crucial Decision: Do not use RunPod Serverless for this task.
Serverless infrastructure is designed for stateless inference workloads (e.g., serving LLMs or Stable Diffusion requests) where a container spins up, processes a request, and terminates.4 CARLA is a stateful simulation server that requires:
Long-Running Processes: The simulation loop must run continuously for minutes to generate a coherent scenario.
Persistent Connections: 10–20 agents need to maintain active TCP sockets to the simulator port (2000).
State Retention: You need to install dependencies, compile Rust code, and debug scripts interactively.
Use Standard Pods with Network Volumes:
You must attach a Network Volume (20GB+) to your pod. This volume persists independently of the GPU instance. It allows you to save your simulation logs (.log), rendered frames (.png), and code. You can shut down the expensive GPU instance when you are sleeping or coding, and spin it back up with all your data intact, significantly reducing costs.6
3.3 Container Strategy and Headless Rendering
The recommended base image is the official CARLA Docker image (0.9.13 or 0.9.14). This version is stable and widely compatible with Python APIs. However, running it on a headless cloud server requires specific flags to trigger off-screen rendering.
Base Image: carlasim/carla:0.9.13
Docker Command Structure:
Bash
docker run \
  -p 2000-2002:2000-2002 \
  --runtime=nvidia \
  --gpus all \
  -v /workspace/data:/home/carla/recordings \
  carlasim/carla:0.9.13 \
 ./CarlaUE4.sh -RenderOffScreen

The -RenderOffScreen flag is the key enabler.7 It forces Unreal Engine to initialize a rendering context on the GPU without attempting to create an X11 window (which would fail and crash the server). This allows the GPU to render frames to an internal buffer, which can then be captured by the Python API.
4. The CARLA Simulation Strategy
To achieve a "cinematic" result without fighting Linux desktop GUIs over high-latency connections, we will decouple the Simulation from the Rendering. This "Record Once, Render Many" workflow is standard in high-end simulation production but underutilized in simple demos.
4.1 The Decoupled Workflow
Instead of capturing the video live (screen recording a VNC session), which risks dropped frames, compression artifacts, and mouse cursor jitters, we utilize CARLA’s built-in Recorder feature.9
Phase 1: Simulation (The "Director" Pass)
Objective: Generate the scenario logic and actor movements.
Configuration: Run the CARLA server in non-rendering mode (or low quality) to maximize simulation tick rate and physics stability.
Action: Execute the scenario_runner.py script. This script spawns the 20 agents, injects the adversarial noise (latency, ghosts), and manages the traffic lights.
Recording: The client issues client.start_recorder("godview_scenario_01.log"). This logs every actor's position, velocity, control inputs, and state for every frame into a highly compressed binary file.
Result: A ~200MB binary file containing the "ground truth" of the entire simulation run.
Phase 2: Data Processing (The "GodView" Pass)
Objective: Generate the data overlays.
Action: While the simulation runs (or during a playback), a separate Python script intercepts the raw actor data.
Data Forking:
Stream A (Raw/Broken): Saves the noisy, jittery detections (simulating the "Before" state) to raw_detections.ndjson.
Stream B (GodView): Pipes these detections into the GodView Core binary (Rust). GodView processes them (CI fusion, Highlander voting) and outputs the canonical state.
Stream C (Output): Saves the cleaned GodView state to godview_state.ndjson.
Phase 3: Rendering (The "Cinematographer" Pass)
Objective: Generate broadcast-quality visuals.
Configuration: Restart CARLA with -quality-level=Epic for maximum visual fidelity (shadows, reflections, anti-aliasing).
Action: Load godview_scenario_01.log into the CARLA Replayer.
Multi-View Capture:
Pass 1 (Top Down): Spawn a camera at (x=0, y=0, z=100, pitch=-90). Play back the log. Save frames to disk.
Pass 2 (Drone View): Attach a camera to a specific drone actor. Play back. Save frames.
Pass 3 (Cinematic): Animate a camera path swooping through the intersection. Play back. Save frames.
Result: Thousands of high-resolution .png files (e.g., frame_0001.png, frame_0002.png) stored on the Network Volume.
This approach guarantees a perfect 60 FPS video because the rendering is decoupled from real-time performance. If the GPU takes 1 second to render one frame, it doesn't matter; the final video will still play back smoothly.
4.2 Solving the "Headless" Display Problem
Since RunPod instances do not have physical monitors, CARLA needs to be tricked into rendering.
Option A: VirtualGL/TurboVNC 12
Description: Run an X server inside the container, use VirtualGL to pass draw calls to the GPU, and connect via VNC.
Pros: Live interaction.
Cons: High setup complexity, fragility with driver updates, high latency for the user. Not recommended for recording high-quality video.
Option B: Off-screen Rendering (Recommended) 7
Description: Run CARLA with ./CarlaUE4.sh -RenderOffScreen. Unreal Engine creates a render context on the GPU but outputs to a framebuffer.
Capture: Use the Python API to listen to a sensor.camera.rgb. In the Python script, image.save_to_disk('_out/%06d.png' % image.frame) saves the pixel data directly.
Pros: Extremely robust, scriptable, no GUI overhead, 100% pixel-perfect capture.
5. Scenario Design: The "High Stakes" Intersection
To make the video compelling, the environment must look chaotic and dense. A sterile environment hides the flaws GodView is designed to fix. We will use Town10HD (High Definition), an asset-rich urban environment included with CARLA.
5.1 The Cast of Actors (10–20 Agents)
The "Ego" Vehicle: A Tesla Model 3 or similar recognizable sedan. This is the viewer's reference point.
Background Traffic: 10 diverse vehicles (Fire trucks, Vans, Cyclists) to create visual noise and occlusion lines.
The "Swarm" (Verticality): 3-4 Drones. Note: CARLA does not have a native "drone" vehicle blueprint in older versions. We will simulate them by spawning a Walker or generic static object and manually teleporting it to z=10m or z=20m in the simulation script. This is crucial for demonstrating the "Pancake World" problem.
The "Adversary": A specific vehicle (e.g., a bright Red Cybertruck or sports car) that will be the source of "spoofed" data.
5.2 The Villain Layers (Injected Failures)
We must artificially degrade the "Raw" data stream to prove GodView works. The script data_exporter.py will apply these transformations to the ground truth data before writing the "Raw" log.
The Jitter Storm (OOSM):
Logic: Apply a random Gaussian delay (0.1s to 1.5s) to the timestamps of 30% of the fleet.
Visual Effect: In the "Before" overlay, bounding boxes for these cars will lag behind the visual mesh, then "snap" forward, or jitter back and forth.
The Pancake Flattening (Verticality Failure):
Logic: For the drone actors, hardcode their z coordinate to 0.0 (ground level) in the "Raw" stream.
Visual Effect: The bounding boxes for drones will appear to be on the ground, crashing into cars and pedestrians. This vividly illustrates the danger of 2D perception.
The Ghost Army (Identity Conflict):
Logic: For the "Ego" vehicle, inject a secondary detection stream with a different UUID (e.g., Car_1_Ghost) and a position offset of 2 meters.
Visual Effect: A second, flickering bounding box appears ghosting the real car.
The Malicious Injection (Trust):
Logic: Create a purely synthetic detection of a "Concrete Barrier" directly in the path of the Ego vehicle. This detection will lack a valid Ed25519 signature in the GodView stream.
Visual Effect: In "Before," the barrier appears as a valid obstacle. In "After," it is highlighted red (Rejected) or removed entirely.
5.3 Narrative Beat Sheet (Video Flow)
Time
Narrative Beat
Visual Action (Split Screen or Toggle)
Overlay Text / Metrics
0:00 - 0:15
The Setup
Cinematic flyover of a busy, rainy intersection. High traffic density. Drone swarm visible overhead.
"Multi-Agent Perception Grid"

"Agents: 18"

"Network: Unstable"
0:15 - 0:45
The Chaos (Before)
Switch to "Raw Sensor View". Show Ghosts flickering. Show Drones flattened onto cars (collisions). Show Jittery movement.
Status: CRITICAL

Ghost Tracks: 14

ID Conflicts: DETECTED

Z-Axis: COLLAPSED
0:45 - 0:55
GodView Activation
A visual "scanline" or pulse sweeps the screen. The ghosts dissolve. The flattened drones "pop" up to their correct altitude.
Initializing GodView Core...

H3 Sharding: ACTIVE

Highlander Consensus: ON
0:55 - 1:10
The Solution (After)
Smooth, interpolated movement. Canonical IDs locked. The "Malicious Wall" appears in raw data but is rejected in GodView.
Status: STABLE

Late Packets Resequenced: 450/sec

Malicious Packets Dropped: 12

Canonical State: CONVERGED
1:10 - 1:20
The Deep Dive
Split screen: Left (Raw jitter), Right (GodView Smooth). Show a graph of "Position Variance" dropping to near zero.
Precision: +400%

Trust: Verified

6. Technical Implementation: The Minimal Script Set
The implementation requires a lightweight Python framework to drive the simulation and recording.
6.1 Script 1: scenario_runner.py (The Director)
This script spawns the world and the agents. Crucially, it enables Synchronous Mode 13, which locks the simulation clock to the client's clock. This ensures that even if the RunPod GPU renders slowly, the recorded simulation physics are deterministic and glitch-free.

Python


import carla
import random
import time

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # Load the high-fidelity town
    world = client.load_world('Town10HD')
    
    # 1. Setup Synchronous Mode (Crucial for deterministic recording)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05 # 20 FPS simulation step
    world.apply_settings(settings)

    # 2. Start Recording
    # This logs the binary state of every actor to disk
    client.start_recorder("/home/carla/recording/godview_demo.log")

    # 3. Spawn Actors
    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    
    vehicles =
    # Spawn 15 vehicles
    for i in range(15):
        bp = random.choice(blueprint_library.filter('vehicle.*'))
        # Mark the 'Villain' car with a specific color
        if i == 0: bp.set_attribute('color', '255,0,0') 
        vehicle = world.try_spawn_actor(bp, random.choice(spawn_points))
        if vehicle:
            vehicles.append(vehicle)
            vehicle.set_autopilot(True) # Use CARLA's built-in AI

    # 4. The Simulation Loop
    try:
        print("Simulation running...")
        for frame in range(1200): # 1 minute at 20 FPS
            world.tick() # Advance one physics step
            
            # (Optional) Log Ground Truth for GodView processing
            # extract_and_log_data(world, frame)
            
    finally:
        client.stop_recorder()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("Simulation done. Log saved.")

if __name__ == '__main__':
    main()


6.2 Script 2: render_cameras.py (The Cinematographer)
This script loads the recorder log and re-renders it from specific camera angles. By attaching a sensor to a "Spectator" actor and moving it, we can create cinematic camera moves (dolly shots, drone follows) completely separate from the simulation logic.

Python


import carla
import queue
import cv2
import numpy as np

def render_view(log_file, output_folder):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    # Replay the simulation
    # 0,0,0 means play the whole file with actor 0 (no specific follow)
    print(f"Replaying {log_file}...")
    client.replay_file(log_file, 0, 0, 0) 

    # Spawn a high-res camera
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', '1920')
    bp.set_attribute('image_size_y', '1080')
    bp.set_attribute('sensor_tick', '0.05') # Match sim step
    
    # Position: High overhead view
    camera_transform = carla.Transform(carla.Location(x=0, z=50), carla.Rotation(pitch=-90))
    camera = world.spawn_actor(bp, camera_transform)
    
    # Queue for frame capture
    image_queue = queue.Queue()
    camera.listen(image_queue.put)
    
    # Frame Loop
    for frame_id in range(1200):
        world.tick() # Advance the replay
        image = image_queue.get()
        
        # Convert raw CARLA image to numpy array for saving
        i = np.array(image.raw_data)
        i2 = i.reshape((1080, 1920, 4))
        i3 = i2[:, :, :3] # Remove alpha channel
        
        # Save frame
        cv2.imwrite(f"{output_folder}/frame_{frame_id:04d}.png", i3)

    camera.destroy()

if __name__ == '__main__':
    render_view("/home/carla/recording/godview_demo.log", "./frames/")


6.3 Script 3: generate_adversarial_data.py (The Noise Maker)
This script iterates through the simulation data and generates the NDJSON logs that represent the "Before" (Raw) and "After" (GodView) states. This allows you to "fake" the real-time processing for the video if running the full Rust stack is too complex for the demo timeframe, or to feed the Rust stack with reproducible bad data.
Input: CARLA Ground Truth (Actor ID, X, Y, Z, Timestamp).
Processing:
Jitter: timestamp = timestamp + random.gauss(0, 0.5)
Pancake: z = 0 if actor.type == 'drone'
Ghost: if random.random() < 0.1: yield create_ghost(actor)
Output: raw_sensor_stream.ndjson.
7. GodView Integration Interface (Handoff Format)
To integrate GodView_core, we define a strict schema using NDJSON (Newline Delimited JSON). This allows decoupling between the Python simulation and the Rust Core.
7.1 Per-Agent Detection Schema (Input to GodView)
This represents what a single agent "sees" and transmits to the Hivemind.

JSON


{
  "packet_type": "DETECTION",
  "sensor_id": "agent_04_camera",
  "timestamp_ns": 1678886400000000000,
  "sequence_id": 1042,
  "objects": [
    {
      "local_id": 12,
      "class": "vehicle",
      "confidence": 0.88,
      "pose": {
        "x": 140.5, "y": -23.1, "z": 0.05,
        "yaw": 1.57,
        "roll": 0.0,
        "pitch": 0.0
      },
      "covariance": [0.5, 0.0, 0.0, 0.5] 
    },
    {
      "local_id": 99, 
      "class": "drone",
      "pose": { "x": 140.5, "y": -23.1, "z": 0.0 }, 
      "note": "PANCAKE_FAILURE_EXAMPLE"
    }
  ],
  "signature": "Ed25519_signature_string..."
}


7.2 GodView Merge Event Schema (Output for Visualization)
This log allows the visualization overlay to explain why GodView made a decision.

JSON


{
  "packet_type": "MERGE_EVENT",
  "timestamp_ns": 1678886400050000000,
  "event_code": "ID_MERGE",
  "details": {
    "incoming_id": "agent_04_obj_12",
    "canonical_id": "Highlander_UUID_A8F",
    "confidence_boost": 0.12,
    "method": "MAHALANOBIS_GATE_PASS"
  }
}



JSON


{
  "packet_type": "MERGE_EVENT",
  "event_code": "TRUST_REJECT",
  "details": {
    "sensor_id": "malicious_agent_01",
    "reason": "INVALID_CAPBAC_TOKEN"
  }
}


8. Visualization & Post-Processing (The "Hero" Layer)
Since we are avoiding real-time tools like Rerun, we will build a Post-Process Overlay using Python’s OpenCV or MoviePy. This allows for "broadcast quality" graphics that are perfectly synchronized with the video.
8.1 The Layout Strategy
We will compose the final video frame by frame.
Background Layer: The 1080p/4K rendered footage from CARLA (Cinematic view).
Augmented Reality Layer (The "After" View):
Green Bounding Boxes: Represent the Canonical GodView State. These boxes should move smoothly.
Red Dashed Boxes: Represent the Raw Sensor Data. These boxes will jitter and ghost.
Vertical Stems: Draw a line from the ground ($z=0$) to the object center ($z=10$) for Drones. This visual cue emphasizes the 3D nature of GodView.
The HUD (Right Side Panel):
Live Log: A scrolling text terminal showing MERGE_EVENT logs (e.g., "REJECTED Malicious Object ID: 99").
Metrics Graph: A rolling line chart showing "System Uncertainty" or "Ghost Count" decreasing over time.
8.2 Overlay Implementation
Using opencv-python, we iterate through the rendered frames and the NDJSON logs simultaneously.
Project: Convert the 3D world coordinates $(x, y, z)$ from the JSON logs into 2D image coordinates $(u, v)$ using the Camera Projection Matrix (which we save during the render_cameras.py pass).
Draw: Use cv2.rectangle for bounding boxes and cv2.line for drone stems.
Burn: Write the new frame to final_output.mp4.
9. Runbook: Zero to Video
This is the tactical checklist for execution.
Phase 1: Infrastructure Setup
RunPod: Rent an RTX 4090 Secure Cloud Pod. Select the carlasim/carla:0.9.13 Docker image (or a PyTorch template where you install CARLA manually).
Volume: Ensure a 20GB Network Volume is attached to /workspace.
SSH Access: Connect via SSH.
Dependencies:
Bash
apt-get update && apt-get install -y python3-pip ffmpeg libomp5
pip3 install carla==0.9.13 numpy opencv-python moviepy


Phase 2: Simulation & Recording
Launch Server:
Bash
# Run in background, offscreen, low quality for performance


./CarlaUE4.sh -RenderOffScreen -quality-level=Low -benchmark -fps=20 &
6. **Run Scenario:**bash
python3 scenario_runner.py
```
Result: godview_demo.log is created.
Phase 3: Rendering & Processing
Data Gen: Run generate_adversarial_data.py to create the NDJSON logs.
Render Visuals:
Bash
# Kill the low-quality server
pkill CarlaUE4
# Start high-quality server


./CarlaUE4.sh -RenderOffScreen -quality-level=Epic &
# Render frames
python3 render_cameras.py --log godview_demo.log --out./frames/
```
Phase 4: Compositing
Generate Overlay: Run the OpenCV script to draw boxes and HUD on the frames.
Encode Video:
Bash
ffmpeg -r 20 -i./frames/overlay_%04d.png -c:v libx264 -pix_fmt yuv420p -crf 18 final_godview_demo.mp4


Download: Use scp or RunPod's JupyterLab file browser to download the MP4.
10. Cost Minimization & Risk Management
10.1 Budget Control
Spot Instances: Use RunPod "Spot" instances. They are ~50% cheaper ($0.25/hr vs $0.44/hr). Because we save all data to the Network Volume, if the instance is preempted (shut down) by the provider, we lose only the current rendering job, not the logs or scripts.
Low-Res Prototyping: Do not render at 1080p immediately. Do a pass at 640x360. This runs 10x faster and allows you to verify camera angles and overlay logic without burning GPU hours.
Aggressive Shutdown: Script the instance to shut down automatically after the FFMPEG job finishes to avoid paying for idle time while you sleep.
10.2 Technical Risks
VRAM OOM: If CARLA crashes with "Out of Video Memory," reduce the texture quality or the number of cameras. The 4090 has 24GB, which is plenty for one camera, but spawning 20 high-res sensors simultaneously will crash it. Our "Record Once, Render Many" approach mitigates this by only spawning one camera during the replay pass.
Headless Glitches: If cv2.imwrite produces black images, ensure the -RenderOffScreen flag was passed correctly and that the Docker container has --gpus all access.
11. Conclusion
This execution plan leverages the deterministic nature of CARLA's recorder to bypass the limitations of cloud streaming. By separating the physics simulation from the visual rendering, and overlaying a structured data narrative via Python post-processing, you can produce a broadcast-quality demonstration of GodView's capabilities for under $10 in compute costs. The resulting video will provide undeniable visual proof of the system's ability to create order from chaos.
Works cited
Pricing | Runpod GPU cloud computing rates, accessed December 25, 2025, https://www.runpod.io/pricing
GPU Price Comparison [2025] - GetDeploying, accessed December 25, 2025, https://getdeploying.com/gpus
GPU Pricing - Runpod, accessed December 25, 2025, https://www.runpod.io/gpu-pricing
Serverless GPU Endpoints - Runpod, accessed December 25, 2025, https://www.runpod.io/product/serverless
Serverless GPU Deployment vs. Pods for Your AI Workload - Runpod, accessed December 25, 2025, https://www.runpod.io/articles/comparison/serverless-gpu-deployment-vs-pods
Manage Pod templates - Runpod Documentation, accessed December 25, 2025, https://docs.runpod.io/pods/templates/manage-templates
Rendering options - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/0.9.10/adv_rendering_options/
Rendering options - CARLA Simulator UE5, accessed December 25, 2025, https://www.ncnynl.com/docs/en/carla/adv_rendering_options/
How to record and replay - CARLA Simulator, accessed December 25, 2025, https://carla.readthedocs.io/en/0.9.6/recorder_and_playback/
Recorder - CARLA Simulator, accessed December 25, 2025, https://carla.readthedocs.io/en/0.9.8/adv_recorder/
Recorder - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/latest/adv_recorder/
Carla headless - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/0.9.5/carla_headless/
How to use the CARLA Simulator - Sagnick, accessed December 25, 2025, https://sagnibak.github.io/blog/how-to-use-carla/
