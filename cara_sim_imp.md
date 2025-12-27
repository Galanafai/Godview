Architectural Optimization and Implementation Strategy for Headless CARLA Simulation with External Rust-Based Visualization (GodView)
1. Executive Summary
The rigorous development of autonomous vehicle perception and fusion systems necessitates simulation environments that are both physically accurate and computationally efficient. The "GodView" system, a sensor fusion and visualization platform architected in Rust, represents a paradigm shift away from monolithic simulation architectures toward a decoupled, modular design. This report provides a comprehensive technical analysis and implementation roadmap for deploying the CARLA simulator in a headless configuration on hardware-constrained environments—specifically targeting the NVIDIA GTX 1050 Ti—while leveraging a custom, high-fidelity visualization stack built upon the Rerun SDK.
The central engineering challenge addressed herein is the decoupling of the simulation's physics and logic loop from its rendering pipeline. By isolating the computational workload of the Unreal Engine 4 (UE4) and offloading the visualization tasks to an optimized external Rust application, we can achieve high-frequency simulation updates even on legacy hardware. This architecture relies on a "No-Rendering" operational mode, a high-throughput ZeroMQ (ZMQ) Data Bridge, and a zero-copy serialization strategy to minimize latency.
Furthermore, this report explores the theoretical underpinnings of "cinematic" camera control for professional presentation (LinkedIn), detailing the mathematics of spline interpolation and quaternion rotation to generate smooth, broadcast-quality visualizations from raw telemetry data. We conclude with a deep dive into synchronization primitives, ensuring that the decoupling of simulation and visualization does not compromise temporal determinism.
2. Introduction: The Paradigm Shift in Autonomous Simulation
The traditional approach to autonomous driving simulation has been monolithic: a single application handles physics, sensor generation, AI logic, and rendering. Tools like CARLA, AirSim, and LGSVL were originally designed with this unity in mind, leveraging game engines like Unreal Engine or Unity to provide a cohesive experience. However, as the complexity of sensor fusion algorithms increases, the monolithic model becomes a bottleneck, particularly on resource-constrained hardware.
2.1 The Monolithic Bottleneck
In a standard CARLA deployment, the GPU is responsible for two distinct heavy workloads:
Sensor Rendering: Generating synthetic images for RGB cameras, depth maps, and segmentation masks. This requires complex shading, rasterization, and geometry processing.
Viewport Rendering: Drawing the "Spectator" view to the screen so the human operator can monitor the simulation.
On a GPU like the NVIDIA GTX 1050 Ti, which operates with 4GB of VRAM and 768 CUDA cores (Pascal architecture), attempting to perform both tasks simultaneously with complex physics calculations inevitably leads to resource contention. The frame rate drops, and more critically, the simulation tick rate becomes unstable, leading to non-deterministic sensor data.
2.2 The GodView Decoupled Philosophy
The "GodView" architecture proposes a separation of concerns. The simulation engine (CARLA) is treated purely as a generator of state and sensor data, stripped of its user interface. The visualization—the "God's Eye View" used for debugging and presentation—is offloaded to a separate process.
This decoupling offers three primary advantages:
Resource Isolation: The heavy rendering pipeline of UE4 can be disabled, freeing GPU resources for CUDA-accelerated Lidar simulation or Neural Network inference.
Flexibility: The visualization (GodView) can run on a different machine, or at a different frame rate, without affecting the simulation physics.
Customization: By using Rust and the Rerun SDK, the visualization can be tailored specifically for sensor fusion debugging (e.g., visualizing Kalman Filter covariance ellipsoids) in ways that the native CARLA viewport cannot support.
3. Hardware Constraints and Architectural Implications
To successfully deploy this architecture, one must understand the specific limitations of the target hardware. The NVIDIA GTX 1050 Ti is a mid-range card from the 2016 Pascal generation. Its limitations dictate the boundaries of our architectural decisions.
3.1 Analysis of the NVIDIA GTX 1050 Ti
The GTX 1050 Ti features 4GB of GDDR5 memory and a 128-bit memory bus, resulting in approximately 112 GB/s of memory bandwidth. In the context of Unreal Engine 4 (CARLA's backend), this bandwidth is the primary bottleneck.
3.1.1 The G-Buffer Cost
Deferred rendering, the technique used by UE4, requires the generation of a G-Buffer (Geometry Buffer) containing multiple render targets (Albedo, Normal, Specular, Depth). At 1080p resolution, these buffers consume significant VRAM. When running CARLA in its default mode, the G-Buffer generation competes for bandwidth with the transfer of sensor data (e.g., Lidar point clouds) from the GPU to system RAM.
3.1.2 Compute Capability
With only 768 CUDA cores, the 1050 Ti has limited compute throughput. Physics simulations in CARLA (using PhysX) are CPU-bound, but Lidar simulation is GPU-bound (using ray casting). If the GPU is saturated by rendering the viewport, Lidar simulation latency spikes, causing the "Chatter" effect where sensor data arrives out of sync with the simulation tick.
3.2 Strategic Resource Allocation
Given these constraints, the strategy for GodView is strict resource allocation:
VRAM: Reserved almost exclusively for loading the map assets and textures required for Lidar ray interactions.
Compute: Reserved for Lidar ray casting and any on-device tensor operations.
Rendering: Strictly disabled via the "No-Rendering" mode.
This allocation strategy necessitates that no visual output be generated by the CARLA process itself. All visualization must occur externally, driven by lightweight telemetry data rather than heavy video streams.
4. Headless Simulation: Theory and Implementation
Running CARLA "headless" is a nuanced topic. There are varying degrees of "headlessness," ranging from simply hiding the window to completely disabling the graphics API.
4.1 Rendering Modes Analysis
Understanding the distinction between "No-Rendering" and "Off-Screen" modes is critical for optimizing the GodView workflow.
4.1.1 No-Rendering Mode
The "No-Rendering" mode is the most aggressive optimization available. In this state, the Unreal Engine bypasses the rendering pipeline entirely for the viewport.
Mechanism: When settings.no_rendering_mode = True is set via the Python API, the engine's main loop skips the Draw and Render phases of the frame.1 It continues to execute the Tick phase, which advances the physics engine (PhysX), AI controllers, and traffic manager.
Performance Impact: This reduces GPU load to near-zero for the viewport. It is the ideal configuration for the GTX 1050 Ti when visual output is not required from the server itself.3
Sensor Limitations: A critical side effect is that camera sensors (RGB, Depth, Segmentation) stop producing data. These sensors rely on the render pipeline. However, Lidar, Radar, GNSS, and IMU sensors—which interact with the collision geometry or physics state—continue to function normally.1
Suitability for GodView: Since GodView focuses on "sensor fusion and visualization" of actor positions and presumably Lidar data (which works in this mode), this is the recommended configuration.
4.1.2 RenderOffScreen Mode
The "RenderOffScreen" mode forces UE4 to perform rendering but directs the output to an off-screen framebuffer rather than a physical display output.
Mechanism: Activated via the -RenderOffScreen flag.1 The GPU still performs rasterization, shading, and post-processing.
Performance Impact: This does not significantly save GPU resources compared to running with a window. It merely removes the window manager overhead.
Usage: This mode is only necessary if GodView requires RGB camera feeds for computer vision tasks. If the goal is purely telemetry-based visualization, this mode is wasteful.
4.1.3 The -nullrhi Trap
Research snippets mention the -nullrhi flag, which disables the Render Hardware Interface entirely.5 While this theoretically offers the highest performance by preventing any GPU communication for rendering, it is notoriously unstable. Many internal scripts and initialization routines in CARLA implicitly check for a valid RHI and will crash if it is missing. For a stable GodView pipeline, avoiding -nullrhi in favor of No-Rendering mode is advised.
4.2 Docker Containerization Strategy
Containerization ensures reproducibility. The user's query specifies a Docker workflow.
4.2.1 The Docker Execution Command
To successfully run CARLA on the GTX 1050 Ti within Docker, we must pass through the GPU and configure the display environment variables to prevent the engine from attempting to connect to an X server.
Optimized Command:

Bash


docker run \
  --runtime=nvidia \
  --net=host \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="DISPLAY=" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  carlasim/carla:0.9.15 \
  bash CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Low


Flag Breakdown:
--net=host: This bypasses the Docker network bridge, allowing the Python bridge (running inside the container) to communicate with the Rust GodView (running on the host) via localhost with zero routing overhead.4
-RenderOffScreen: Initializes the engine without a window.
-quality-level=Low: This is crucial for the 1050 Ti. It simplifies materials, disables shadows, and reduces the rendering distance to 50m.3 Even if we disable rendering later via Python, starting in Low mode ensures the initialization phase doesn't spike VRAM usage.
4.3 Runtime Configuration (The Hybrid Approach)
The optimal strategy is a hybrid initialization:
Launch: Start the container with -RenderOffScreen. This satisfies the engine's need for a display context (even if virtual) and prevents startup crashes.5
Runtime Switch: Immediately upon connecting the Python client, send the command to disable rendering.

Python


import carla

def configure_simulation(client):
    world = client.get_world()
    settings = world.get_settings()
    
    # Critical for GTX 1050 Ti performance
    settings.no_rendering_mode = True
    
    # Ensure determinism
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05 # 20 Hz
    
    world.apply_settings(settings)


This sequence ensures a stable boot followed by maximum resource conservation.2
5. The Data Bridge: Bridging the Language Gap
The "Bridge" is the software component responsible for extracting state from CARLA (C++/Python) and transmitting it to GodView (Rust). This component is the primary source of latency in the pipeline.
5.1 The CARLA Python API Internals
CARLA's Python API is a wrapper around a C++ client. Every time a Python script calls a function like actor.get_transform(), the call must cross the language boundary, serialize the request, send it to the server (even if local), wait for the response, deserialization, and return to Python.
5.1.1 The "Chatter" Problem
A naive implementation iterates through all actors to fetch their state:

Python


# Naive Approach - DO NOT USE
for actor in world.get_actors():
    transform = actor.get_transform()  # Costly IPC call
    velocity = actor.get_velocity()    # Costly IPC call


In a scene with 100 actors, this results in 200 distinct network round-trips per frame. This "chatter" will bottleneck the simulation long before the GPU limits are reached.
5.1.2 Batch Retrieval via WorldSnapshot
To solve this, CARLA provides the WorldSnapshot mechanism.6 A snapshot is a serialized binary blob containing the state of all actors for a specific simulation tick.
Efficiency: Retrieving the snapshot requires a single API call.
Data Content: The snapshot contains id, transform, velocity, angular_velocity, and acceleration.
Limitation: The snapshot does not contain static metadata like "Vehicle Model" or "Color."
5.2 The Static/Dynamic Registry Pattern
To build a complete picture for GodView without transmitting redundant data, the Bridge must implement a Registry Pattern.
Data Type
Frequency
Source
Handling Strategy
Static (Model, Color, Role)
Once (at spawn)
world.get_actors()
Cache in Python dict; send "New Actor" event to Rust.
Dynamic (Position, Speed)
Every Tick (20Hz)
world.get_snapshot()
Broadcast highly optimized binary packet.

5.3 Serialization Strategy: Zero-Copy and Memory Layout
High-frequency telemetry demands efficient serialization. Python's native pickle is too slow and produces payloads that are too large. JSON is CPU-intensive to parse.
5.3.1 Structured Binary Packing
The most efficient approach is to mimic a C-struct memory layout using Python's numpy library. This ensures that the data in memory is tightly packed and ready for transmission without complex serialization logic.
Schema Design:
We define a linear memory layout for the dynamic update packet:
[Header: 16 bytes]
Frame ID (uint64)
Timestamp (double)
Actor ID (uint32)
Position X (float32)
Position Y (float32)
Position Z (float32)
Rotation Pitch (float32)
Rotation Yaw (float32)
Rotation Roll (float32)
Velocity X (float32)
Velocity Y (float32)
Velocity Z (float32)
5.3.2 Implementing with Numpy
Using numpy, we can pre-allocate a buffer and fill it directly from the snapshot data. This avoids the overhead of Python object creation for every float value.

Python


import numpy as np

# Define the structured dtype equivalent to the C-struct
dtype_actor = np.dtype([
    ('id', 'u4'),
    ('pos', '3f4'),
    ('rot', '3f4'),
    ('vel', '3f4')
])

def serialize_snapshot(snapshot):
    count = len(snapshot)
    buffer = np.zeros(count, dtype=dtype_actor)
    
    # Optimized loop (or vectorization if possible)
    for i, actor in enumerate(snapshot):
        t = actor.get_transform()
        v = actor.get_velocity()
        buffer[i]['id'] = actor.id
        buffer[i]['pos'] = (t.location.x, t.location.y, t.location.z)
        buffer[i]['rot'] = (t.rotation.pitch, t.rotation.yaw, t.rotation.roll)
        buffer[i]['vel'] = (v.x, v.y, v.z)
        
    return buffer.tobytes()


The .tobytes() method returns a raw byte string that is a direct copy of the memory buffer, which is extremely fast.7 This binary blob is then ready for immediate transmission over the IPC layer.
6. Inter-Process Communication (IPC) Design
The link between the Python Bridge and the Rust GodView application must be low-latency and high-throughput. The user query explicitly asks for a comparison and decision between protocols like ZeroMQ and gRPC.
6.1 Transport Layer Analysis
6.1.1 gRPC (Google Remote Procedure Call)
Mechanism: gRPC uses HTTP/2 as its transport layer and Protocol Buffers for serialization.8
Pros: It offers strong typing, rigid schemas, and bi-directional streaming. It is excellent for microservices communicating over a network.
Cons: The HTTP/2 framing overhead is significant for high-frequency (60Hz+) local loops. It requires header compression, stream management, and often TLS handshakes. On a CPU-limited system (where the CPU is feeding the GPU), this overhead steals cycles from the physics engine.9
Verdict: For GodView, gRPC adds unnecessary complexity and latency.
6.1.2 ZeroMQ (ZMQ)
Mechanism: ZeroMQ is a "brokerless" messaging library. It acts as an abstraction over raw TCP or Unix Domain Sockets.8
Pros: It is extremely lightweight. When using IPC (Inter-Process Communication) transport, it bypasses the system's network stack entirely, copying data directly from the sender's memory to the receiver's memory via the kernel.
Pattern: The Publish/Subscribe (PUB/SUB) pattern is ideal here. The Bridge "Publishes" state updates. GodView "Subscribes." This decoupling means GodView can be restarted without crashing the simulation.10
Verdict: ZeroMQ is the optimal choice.
6.2 Protocol Configuration
To maximize performance on the local machine:
Transport: Use tcp://127.0.0.1:5555 if running in Docker with --net=host. If running natively, use ipc:///tmp/godview.pipe for even lower latency (10-20% improvement over TCP loopback).11
Socket Options:
ZMQ_CONFLATE: This is a critical option for visualization. If GodView falls behind (rendering takes too long), we do not want to process old frames. ZMQ_CONFLATE tells the subscriber to keep only the latest message in the queue.12 This ensures that whatever GodView renders is always the most current state of the simulation, preventing "death spirals" of latency.
7. GodView Core: The Rust Implementation
The godview_core structure serves as the ingestion and processing engine. Written in Rust, it prioritizes memory safety and zero-cost abstractions to handle the high-throughput data stream.
7.1 Architecture Overview
Based on standard high-performance robotics practices (and inferring the godview_core context), the application follows a pipeline architecture:
Ingestor (Async): A Tokio-based task that listens to the ZMQ socket.
Deserializer (Zero-Copy): Casts raw bytes to Rust structs.
State Manager (ECS): Updates the internal representation of the world.
Visualizer (Rerun): Logs the state to the visualization SDK.
7.2 Zero-Copy Deserialization
Rust's strict memory model allows for a powerful optimization: casting a byte slice directly to a struct slice. This eliminates the parsing step entirely.
Rust Implementation Details:
We use the bytemuck or zerocopy crates to safely perform this cast. The Rust struct must have the #[repr(C)] attribute to guarantee it matches the C-layout generated by Numpy in Python.13

Rust


use zerocopy::{FromBytes, AsBytes};

#[repr(C)]
#
struct ActorUpdate {
    id: u32,
    pos: [f32; 3],
    rot: [f32; 3],
    vel: [f32; 3],
}

fn handle_packet(data: &[u8]) {
    // Zero-copy cast: 'actors' is a slice pointing to the raw 'data' memory
    let actors: &[ActorUpdate] = zerocopy::LayoutVerified::new_slice(data)
       .expect("Alignment/Size mismatch")
       .into_slice();

    for actor in actors {
        // Process actor...
    }
}


This operation is effectively instantaneous, regardless of the number of actors, as it involves no memory allocation and no data copying.13
8. Visualization and The Rerun SDK
Rerun is the "screen" of the GodView system. Unlike a game engine that draws pixels, Rerun records "events" in a database and visualizes them. This is a subtle but powerful distinction for sensor fusion debugging.
8.1 The Rerun Data Model
Rerun organizes data into Timelines, Entities, and Components.14
Timeline: The axis of history. We will use two: sim_time (simulation seconds) and log_time (wall clock).
Entity Path: A hierarchical identifier, e.g., world/actors/vehicle_42.
Archetype: A high-level object, like Mesh3D or Transform3D.15
8.2 Entity Hierarchy Strategy
Rerun's hierarchy system allows for efficient data logging.
Root Transform: We log the Transform3D at world/actors/{id}.
Children: We log the vehicle mesh at world/actors/{id}/mesh and debug markers at world/actors/{id}/debug.
Benefit: When the Bridge sends a new position for Actor 42, we only update the transform at the root. Rerun automatically propagates this transform to the mesh and all debug markers.16 This drastically reduces the bandwidth required; we don't re-log the mesh or the sensor transforms every frame, only the parent actor's movement.
8.3 Handling Static Assets (Meshes)
Streaming geometry (meshes) over ZMQ is prohibitive. GodView implements a caching strategy.
Actor Spawn Event: When a new Actor ID appears, GodView looks up its type (e.g., "vehicle.tesla.model3").
Asset Loading: GodView loads a simplified .gltf or .obj proxy mesh from its local disk.
Timeless Logging: GodView logs this mesh to Rerun with the static=true flag (or on a timeless timeline).
Rust
rr.log(
    format!("world/actors/{}/mesh", id),
    rr.Mesh3D::new(...).with_class_ids(...)
);

This mesh is logged once. Rerun retains it. Future updates only change the position.17
9. Cinematic Visualization and Camera Control
The user request specifically highlights "LinkedIn" visualization, implying a need for aesthetic polish. A jittery, mechanically-locked camera is jarring. GodView must implement a "Cinematic Camera" system that smoothly follows the action.
9.1 Mathematical Framework for Camera Pathing
To create a broadcast-quality view, we cannot simply snap the camera to the Ego vehicle's coordinate frame. We must use a Lazy Follow or Spline Interpolation algorithm.
9.1.1 Catmull-Rom Splines
A Catmull-Rom spline is ideal for this application because the curve is guaranteed to pass through the control points (the vehicle's history).
Let $P_0, P_1, P_2, P_3$ be four consecutive historical positions of the target vehicle. The interpolated position $P(t)$ for $t \in $ between $P_1$ and $P_2$ is given by:

$$P(t) = 0.5 \cdot \begin{bmatrix} 1 & t & t^2 & t^3 \end{bmatrix} \begin{bmatrix} 0 & 2 & 0 & 0 \\ -1 & 0 & 1 & 0 \\ 2 & -5 & 4 & -1 \\ -1 & 3 & -3 & 1 \end{bmatrix} \begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix}$$
Implementation Strategy:
GodView maintains a ring buffer of the last 20 positions of the target vehicle. It calculates a spline path through these points and places the camera at a position corresponding to $t_{current} - \delta$, where $\delta$ is a delay factor (e.g., 0.5 seconds). This creates a smooth "towing" effect where the camera gently swings behind the car during turns rather than snapping rigidly.18
9.1.2 Quaternion Interpolation (SLERP)
Interpolating rotation is harder than position. Linear interpolation of Euler angles leads to "Gimbal Lock" and weird artifacts. We must use Spherical Linear Interpolation (SLERP) on Quaternions.

$$\text{Slerp}(q_1, q_2, t) = \frac{\sin((1-t)\Omega)}{\sin(\Omega)}q_1 + \frac{\sin(t\Omega)}{\sin(\Omega)}q_2$$
where $\Omega$ is the angle subtended by the arc between $q_1$ and $q_2$. Rerun supports quaternions natively in Transform3D, so GodView computes the interpolated quaternion and logs it directly.19
9.2 Rerun Viewport Control
Rerun allows the logging of a Pinhole archetype, which effectively defines a camera's lens properties (Field of View, Aspect Ratio).

Rust


// Log the Cinematic Camera
let cam_path = "world/cinematic_camera";

// 1. Position and Orientation (calculated via Spline/SLERP)
rr.log(cam_path, rr.Transform3D::new(pos, rot));

// 2. Lens Properties (e.g., 35mm equivalent)
rr.log(cam_path, rr.Pinhole::new([1920.0, 1080.0]).with_focal_length(1000.0));


In the Rerun Viewer, the user selects "world/cinematic_camera" to look through this virtual lens, achieving the desired high-quality output.
10. Synchronization and Determinism
A distributed system (CARLA + GodView) risks temporal drift. If the visualization lags, the data becomes misleading.
10.1 The Time Domain Problem
Network jitter means packets arrive at irregular intervals. If GodView logs data using SystemTime::now() (Wall Clock), the visualization will stutter.
10.2 Explicit Time Setting in Rerun
The solution is to force Rerun to use Simulation Time as its primary index.
Source of Truth: CARLA generates a timestamp (e.g., snapshot.timestamp.elapsed_seconds).
Transmission: This float value is included in the ZMQ header.
Logging: Before logging any entities, GodView explicitly sets the global time cursor.

Rust


// Rust GodView Logic
let packet = receive_zmq();
let sim_time = packet.timestamp;

// Critical: Set the time index for all subsequent logs in this batch
rr.set_time_seconds("sim_time", sim_time); 

for actor in packet.actors {
    rr.log(actor.path, actor.transform);
}


By strictly adhering to sim_time, Rerun reconstructs the timeline perfectly. Even if the Rust application pauses for 500ms (e.g., OS context switch), when it resumes, it logs the data at the correct historical timestamps, ensuring the recording is mathematically perfect and replayable without jitter.21
11. Performance Analysis and Optimization
To verify the system meets the constraints of the GTX 1050 Ti and the latency requirements of GodView, we employ a rigorous optimization loop.
11.1 Profiling Strategy
CARLA Server: Use the built-in server_fps metric. With No-Rendering and a fixed step of 0.05s, the server FPS should lock to 20. If it drops, the CPU is the bottleneck (physics).
GodView: Use flamegraph for Rust to visualize CPU time. Ensure that deserialization (the Zero-Copy cast) takes negligible time (< 1ms).
11.2 Latency Budgets
Component
Target Latency
Optimization Technique
CARLA Tick
< 25ms
No-Rendering Mode, low poly assets.
Python Bridge
< 2ms
Batch snapshot retrieval, Numpy structured arrays.
ZMQ Transport
< 0.5ms
IPC (Unix Sockets), No-Copy Send.
Rust Ingest
< 0.1ms
Zero-copy deserialization (bytemuck).
Rerun Log
Async
Rerun handles batching internally.

11.3 Physics Substepping
If the visualization looks "jerky" at 20Hz, but the 1050 Ti cannot handle 60Hz, use Physics Substepping.
Concept: CARLA calculates physics multiple times per frame (e.g., 10 substeps of 0.005s) but only sends the final result to GodView every 0.05s.
Benefit: This keeps the vehicle physics stable (suspension doesn't explode) while maintaining a low visualization frame rate manageable by the GPU.22
12. Conclusion
The "GodView" architecture represents a sophisticated application of distributed systems theory to the problem of autonomous vehicle simulation. By acknowledging the hard limits of the GTX 1050 Ti and responding with a radically decoupled design, we achieve a workflow that rivals high-end workstations.
The combination of CARLA's "No-Rendering" mode, ZeroMQ's lightweight IPC, and Rerun's data-centric visualization creates a pipeline where the simulation is free to compute physics unburdened by graphics, and the visualization is free to render "cinematic" outputs unburdened by simulation overhead. The rigorous application of zero-copy serialization and spline-based camera mathematics ensures that the final output—destined for professional networks like LinkedIn—is not just performant, but visually polished and deterministic. This report serves as the definitive blueprint for implementing godview_core.
Works cited
Rendering options - CARLA Simulator - CARLA documentation, accessed December 25, 2025, https://carla.readthedocs.io/en/latest/adv_rendering_options/
Rendering options - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/0.9.10/adv_rendering_options/
Rendering options - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/0.9.11/adv_rendering_options/
CARLA in Docker - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/latest/build_docker/
RenderOffScreen with Carla Docker Image - Vulkan Driver Problems ..., accessed December 25, 2025, https://github.com/carla-simulator/carla/issues/8079
1st. World and client - CARLA Simulator UE5, accessed December 25, 2025, https://www.ncnynl.com/docs/en/carla/core_world/
Numpy array: get the raw bytes without copying - Stack Overflow, accessed December 25, 2025, https://stackoverflow.com/questions/69544408/numpy-array-get-the-raw-bytes-without-copying
Comparative Analysis OF GRPC VS. ZeroMQ for Fast Communication - JETIR, accessed December 25, 2025, https://www.jetir.org/papers/JETIR2002540.pdf
Like for Like HTTP vs gRPC Comparison : r/rust - Reddit, accessed December 25, 2025, https://www.reddit.com/r/rust/comments/169t5ce/like_for_like_http_vs_grpc_comparison/
Chapter 2 - Sockets and Patterns - ZeroMQ Guide, accessed December 25, 2025, https://zguide.zeromq.org/docs/chapter2/
Low latency network service libraries? : r/rust - Reddit, accessed December 25, 2025, https://www.reddit.com/r/rust/comments/gnap4k/low_latency_network_service_libraries/
Why is it faster sending data as encoded string than sending it as bytes? - Stack Overflow, accessed December 25, 2025, https://stackoverflow.com/questions/71022022/why-is-it-faster-sending-data-as-encoded-string-than-sending-it-as-bytes
Zero-Copy in Rust: Challenges and Solutions - GitHub, accessed December 25, 2025, https://github.com/Laugharne/rust_zero_copy
Operating Modes - Rerun, accessed December 25, 2025, https://rerun.io/docs/reference/sdk/operating-modes
Mesh3D in rerun::archetypes - Rust - Docs.rs, accessed December 25, 2025, https://docs.rs/rerun/latest/rerun/archetypes/struct.Mesh3D.html
Transforms & Coordinate Frames - Rerun, accessed December 25, 2025, https://rerun.io/docs/concepts/spaces-and-transforms
Raw mesh - Rerun, accessed December 25, 2025, https://rerun.io/examples/feature-showcase/raw_mesh
Cubic Spline I. does weird camera paths. - Minecraft Replay Mod Forums, accessed December 25, 2025, https://www.replaymod.com/forum/thread/2266
Structure from motion - Rerun, accessed December 25, 2025, https://rerun.io/examples/3d-reconstruction/structure_from_motion
Transforms - Rerun Python APIs, accessed December 25, 2025, https://ref.rerun.io/docs/python/v0.2.0/common/transforms/
Logging functions - Rerun Python APIs, accessed December 25, 2025, https://ref.rerun.io/docs/python/0.12.1/common/logging_functions/
Synchrony and time-step - CARLA Simulator - Read the Docs, accessed December 25, 2025, https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/
