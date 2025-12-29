GodView V2 Demo Script: From Sensor Chaos to
Consensus
Introduction and Objectives
This script outlines a production-grade demo for GodView V2, a distributed perception fusion system for
autonomous vehicles. The goal is to create an ~80-second cinematic video (optimized for LinkedIn and
professional audiences) demonstrating how GodView transforms chaotic, unreliable sensor data into a
stable, trusted world view . The narrative will emphasize a stark "Before vs. After" contrast: a chaotic
pre-GodView scene filled with sensor ghosts, jitter, and glitches, followed by an orderly post-GodView scene
where data conflicts are resolved and truth emerges . To achieve high visual fidelity on a budget, we
use a "Record Once, Render Many" workflow on cloud GPU (RunPod) – i.e. record the simulation once,
then replay it from multiple camera angles for rendering . The result is a polished, technically rich
demonstration of GodView’s capabilities in an adversarial sensor environment.
Technical Parameters
Simulation Platform: CARLA 0.9.15 (Docker container on RunPod cloud) running on an NVIDIA RTX
4090.
Video Output: 1920×1080 resolution, 30 FPS, ~80 seconds duration, encoded as MP4 (H.264).
Input Data: Pre-recorded CARLA simulation log godview_demo.log (scenario replay) and NDJSON
detection streams ( raw_broken.ndjson for raw detections, and godview_merged.ndjson for
GodView’s fused output). These files contain timestamped events and object data recorded during
the scenario.
Workflow: The simulation is first run headlessly to produce godview_demo.log . This log is then
replayed multiple times with different camera setups to generate frames. Overlays (HUD graphics,
bounding boxes, etc.) are drawn on the frames using OpenCV, and finally frames are encoded into
the final video. This decoupling ensures simulation physics are computed once, while rendering can
be repeated from various perspectives for a cinematic result .
Narrative Timeline & Phases
The demo is structured into five phases, each highlighting a stage in the transformation from entropy to
consensus:
SETUP (0–15s): Establishing the Scene. We open with a bird’s-eye view of the entire environment,
showing all ~18 agents (vehicles, drones, etc.) in a city block. The camera slowly orbits from above,
providing context and scale. During this phase, we introduce minimal overlays: perhaps subtle labels
or markers on each agent. The system is in its baseline state – no fusion yet – but we foreshadow
the chaos to come (e.g. a drone hovering appears slightly mis-positioned or flat against the ground,
1
2 3
4
•
•
•
•
5
1.
1
hinting at a perspective issue). A top HUD banner can label this phase as "Phase 1: Setup – Baseline
Chaos" for clarity. This quiet setup gives viewers a clear layout of the scene before chaos ensues.
CHAOS (15–45s): Raw Sensor Mayhem. Camera cuts to a dynamic hero chase cam behind the ego
vehicle (our protagonist car), following it through traffic. Here we visualize the uncorrected, raw
sensor data in all its problematic glory . Red dashed bounding boxes jitter around objects,
reflecting noisy and unsynced detections. Some vehicles appear to rubber-band or momentarily
teleport backward due to network latency and out-of-sequence sensor updates (OOSM). Drones that
should be flying are instead “pancaked” on the road (altitude ambiguity causing them to render at
ground level). You might see duplicate ghost objects flickering – e.g. a car that appears in two
places at once – representing ID conflicts or malicious spoofing. One of these ghosts can be a Sybil
attack example: a fake object rendered in magenta, clearly out-of-place. The HUD may show
warnings (e.g. "GPS spoofing detected" or "Unknown sensor ID"). This chaotic segment illustrates the
entropy in the system before GodView, where each sensor’s perspective conflicts with others.
ACTIVATION (45–55s): GodView Engages. At 45s, the GodView system activates, transitioning the
narrative from chaos to resolution . Visually, we indicate this with a dramatic HUD effect – for
example, a bright yellow scanline sweeps down the screen, symbolizing a synchronization or
scanning of all data. As the scanline passes, the scene begins to change: jittery red boxes start to
turn into stable green ones, and ghost duplicates start aligning. We overlay trust badges next to
each agent: a green ✓ for trusted inputs and a red ✗ for any malicious/untrusted source. The bottom
HUD can display messages like “ACTIVATING GODVIEW…”, “SIGNATURE VERIFIED for Agent 7”, or
“INVALID TOKEN from Sensor 3” to simulate a security/authentication layer kicking in. This phase is
brief but critical, showing the moment of transformation – the system performing the “Highlander”
merge (there can only be one object for each real entity) and filtering out bad data. By the end of this
10-second activation window, GodView has locked onto a single, consistent world model.
SOLUTION (55–70s): Stabilized World View. We continue with the hero chase cam, but now the chaos
has subsided. This is the "After" scene – a stable consensus where all sensors agree. Green solid
bounding boxes smoothly track each object (no more jitter). The once-duplicated ghost objects have
vanished or merged into one – an animation can show two flickering red ghost boxes converging
into a single green box, illustrating the Highlander convergence logic (only one canonical object
remains) . The drone that was flat on the ground in chaos now pops up to its correct altitude,
with a translucent vertical line (stem) drawn from the drone down to the ground to emphasize its
elevation . Any malicious object (magenta in the chaos phase) is now gone or struck through with
a bold “REJECTED” label before fading out. The HUD can mark this phase as "Phase 4: Solution – Fused
Consensus" with a green status indicator. This segment highlights the improvements: objects move
predictively and smoothly (no teleportation), and the overall scene looks coherent and trustable. It’s
a direct before/after contrast with the Chaos phase – viewers clearly see how GodView cleaned up
the scene.
DEEPDIVE (70–80s): Behind the Scenes & Metrics. In the final 10 seconds, we switch back to a bird’seye (top-down) view to recap and highlight technical details of the fusion. The camera might pause
or move to a high vantage point capturing the whole scene. We overlay a faint H3 hexagonal grid
on the ground, illustrating the spatial partitioning used (this helps explain how the drone vs ground
vehicle conflict was resolved by separating altitude layers). Active grid cells can be highlighted where
agents are present. We also draw LIDAR-style range rings around the ego vehicle (e.g. 10m, 20m,
2.
3
3.
6
4.
2
6
6
5.
2
30m arcs) and maybe detection rays connecting the ego to nearby object positions, to indicate
sensing range and coverage. A side panel or subtitle text can show live metrics/logs: e.g. “Ghost
objects merged: 5”, “Average position error corrected: 0.2m”, “Malicious signals rejected: 1”, “Latency
compensation: 120ms”. The top HUD could label "Phase 5: DeepDive – System Diagnostics". This is
where we give the engineering audience a peek into the mechanisms behind the magic – reinforcing
that GodView’s trust engine and sensor fusion algorithms are what fixed the chaos. The video
then fades out, likely ending on the GodView logo or a tagline like “Distributed Consensus Achieved.”
Implementation Architecture
To produce this demo, we divide the task into three coordinated modules (scripts), aligning with the Record
Once, Render Many approach:
replayer.py – Simulation Replay & Camera Capture: This script loads the recorded CARLA
scenario ( godview_demo.log ) and replays it in the simulator. It programmatically attaches
cameras and cinematic rigs as defined (see camera paths below) and captures frames. We will run
the replay multiple times for different views (top-down, chase cam, etc.), saving frames to disk (e.g.
as PNG images). CARLA’s off-screen rendering ( image.save_to_disk ) is used to achieve pixelperfect captures without a GUI. This replay script essentially acts as our virtual film crew, recording
the scene from various angles without altering the underlying scenario.
overlay_renderer.py – HUD Overlay & Visualization: This module post-processes the
rendered frames, adding the 2D overlay graphics that illustrate sensor data and GodView outputs. It
reads the NDJSON logs ( raw_broken.ndjson and godview_merged.ndjson ) and, for each
frame (timestamp), it draws all necessary elements: bounding boxes, labels, ghost effects, grid lines,
etc. Using OpenCV, it projects 3D world coordinates to 2D screen coordinates for each detected
object based on the camera perspective, then draws the appropriate shapes and text. This is where
the visual storytelling elements (red vs green boxes, scanline, trust icons, etc.) are composited on top
of the raw CARLA footage. The overlay renderer ensures that for each video frame, we have a
corresponding fused HUD that reflects the system’s state at that moment.
composer.py – Video Composition: Finally, the composer script assembles the processed frames
into the final video. Using FFmpeg (or OpenCV’s video writer), it stitches the sequence of images
(with overlays) into a 80-second MP4 file at 30 FPS. This script handles adding any fade transitions
between phases, inserting an audio track or sound effects (e.g. the Activation scanline sound) if
needed, and ensures encoding settings for a high-quality output (e.g. H.264 codec with appropriate
bitrate or CRF). The result is final_godview_demo.mp4 , ready for publishing. By separating this
step, we can easily re-export the video with different encoding settings or minor edits without
rerunning the heavy simulation.
•
•
•
3
Camera Path Definitions
Multiple virtual camera views are used to make the demo engaging and informative. We define three key
camera configurations in CARLA:
Bird’s-Eye View Camera: A stationary top-down camera looking straight down from high altitude.
For example, Transform(x=0, y=0, z=100) with Rotation(pitch=-90°, yaw=0) gives a
nadir view over the scene. This view is used in the Setup and DeepDive phases to show the global
context. In Setup, we add a slow orbit: over 15 seconds the camera yaw rotates ~90° around the
scene center, creating a dramatic aerial sweeping shot. This top-down perspective ensures all
agents are visible at once and is ideal for overlaying global elements like the hex grid and sensor
range rings.
Hero Chase Cam: A trailing camera attached behind and slightly above the ego vehicle. For instance,
a transform ~15 meters behind the car and 8 meters up (relative to the car’s frame) with a slight
downward tilt. This camera sticks with the ego as it drives, providing an on-road viewpoint. It’s
utilized during Chaos, Activation, and Solution phases. The chase cam gives a driver’s perspective
on how objects move around the ego, making issues like jitter or ghosting very apparent. It also
keeps the viewer engaged with a cinematic “follow-car” shot, while still allowing overlay elements
(like bounding boxes on other cars or drones in view) to be seen clearly from a familiar angle.
Orbiting Drone Cam: A free-moving camera that circles around the scene for dynamic effect. We
use this in the Setup phase (perhaps overlapping with the bird’s-eye view) to introduce motion in the
otherwise static overview. For example, the camera could start at a high angle and then move in an
arc or circle around the periphery of the scene, yawing to keep the center in frame. Over the first 15
seconds, it might complete a 90° orbit. This gives a parallax view of the environment and highlights
depth (vehicles and buildings shifting perspective) even before we introduce any overlays. It sets a
cinematic tone, as if a drone cameraman is capturing the city scene. (Note: This orbit cam can be
implemented by smoothly varying the camera’s transform each frame during the Setup phase.)
All camera views use a CARLA RGB sensor with the desired resolution (1920×1080) and a
sensor_tick=0.033 (33ms) to lock the capture at 30 FPS. The cameras are configured for a wide field-ofview to cover much of the scene. Optionally, we can also attach a semantic segmentation camera in the
same position to aid in generating detection masks or depth info (useful for more advanced overlay effects),
but this is not required if the NDJSON provides all needed data. Each camera’s output is saved to a separate
frames folder (e.g., frames/top_down/*.png , frames/chase_cam/*.png ), to be overlayed and edited
together in post.
Sensor Data Overlay Pipeline (OpenCV HUD)
The core of the demo’s message comes from the heads-up display (HUD) overlays drawn on the video
frames, which illustrate the differences between raw and fused perceptions. The overlay_renderer.py
pipeline works as follows:
Frame & Timestamp Alignment: We parse the NDJSON files containing detection data. Each entry
has a timestamp and details of detected objects (position, class, source, etc.). We correlate each
1.
2.
3.
•
4
video frame with the closest timestamped data – since the simulation and camera run at ~30 FPS, an
easy mapping is frame_id ≈ time_sec * 30 . This ensures that for each rendered frame, we
fetch the corresponding raw detection and GodView output data.
Data Association: To visualize “ghosts” and merging, the system must decide which raw detections
correspond to which fused objects. We use spatial proximity (and perhaps object IDs from the data)
to match raw vs fused detections per frame. If needed, a spatial index (e.g., an R-tree or KD-tree on
object coordinates) can be constructed each frame to efficiently match nearby detections between
the raw set and the fused set. This helps identify duplicates or misalignments that GodView is
resolving.
Interpolation & Smoothing: For visual smoothness, we can interpolate object bounding boxes
between frames or apply a simple predictive filter so that movements look fluid. This simulates the
effect of an EKF (Extended Kalman Filter) or similar smoothing – whereas raw data might jump, our
drawn fused boxes can interpolate to glide along the path. Essentially, minor timing differences in
the NDJSON data can be evened out so the overlay animations feel continuous.
Once the data for a frame is prepared, we draw the following visual elements on the frame (using OpenCV
drawing functions) to represent the raw vs fused perceptions:
Object Bounding Boxes: Each detected object is enclosed in a box. Raw detections (from
raw_broken.ndjson ) use a red, dashed rectangle – conveying uncertainty and instability (jitter).
GodView fused objects (from godview_merged.ndjson ) use a solid green rectangle – conveying
confidence and stability. The class label (e.g. “CAR”, “TRUCK”, “DRONE”) is shown near the box to
identify the object. In the chaotic phase, red boxes may appear misaligned or double-placed
(ghosts), whereas in the resolved phase, a single green box hugs the true object tightly.
Ghost Tracks: For any object that had duplicate or conflicting reports (e.g., two sensors saw the
same car in slightly different positions, or a malicious clone), we visualize the discrepancy by drawing
semi-transparent or flickering duplicates. For instance, a ghost car might be indicated by a
second red bounding box that blinks or fades, to show it’s not real. As GodView merges them, you
can animate the ghost box moving toward the real object and eventually disappearing. This
“Highlander” merge visualization communicates that there can only be one authoritative detection
for each real object .
Drone Altitude Stems: For aerial objects like drones, we draw a vertical line (stem) from the object
down to the ground plane. In the raw view (Chaos phase), a drone might be erroneously at ground
level (z=0) – effectively flattened. In the fused view, the drone’s green box will be elevated at the
correct altitude, and a green stem connects it to the ground, indicating height. If a drone’s altitude
was mis-reported, the stem might appear red or broken in the raw view. Once corrected in the
Solution phase, the stem turns green and extends correctly, demonstrating restoration of vertical
position awareness.
Sybil/Malicious Object Highlight: Any object identified as malicious or a sensor spoof (e.g., a
phantom car inserted by a compromised agent) is drawn in magenta in the raw phase. This makes it
clearly stand out against normal objects. Alongside, we might stamp a bold “REJECTED” text over it
after GodView flags it. In the Solution phase, this object is gone (filtered out), but to drive the point
home, we show the magenta box being crossed out or fading with a rejection stamp at the moment
of GodView activation.
•
•
1.
2.
7
3.
4.
5
LIDAR Overlay: To reinforce the sensing context, we overlay a stylized LIDAR point cloud or range
indication. Concentric range rings (e.g., at 10m intervals) centered on the ego vehicle are drawn as
faint white or neon circles on the ground. We can also plot a subset of LIDAR points as semitransparent dots – colored by depth (e.g., white for near, blue to purple for far) – to give a sense of
sensor scanning. These points could be static for the frame (if we have a sample point cloud for that
time). Additionally, drawing thin detection rays (lines) from the ego or from sensors to the objects
they detect adds a dynamic sense of how the ego car “sees” the environment. For example, in
Activation or DeepDive, as trust is established, you might show green connection lines from ego to
each valid object and a red broken line to a rejected object.
H3 Spatial Grid: We overlay a hexagonal grid (from Uber’s H3 geospatial indexing) on the map to
illustrate how space is partitioned. The grid cells can be very light grey lines on the ground for
reference. When GodView is active, highlight the cells that contain objects or that are being actively
fused. This is especially useful for showing altitude separation: the drone might be in a different
vertical layer (but same x,y cell) – with GodView, you could briefly highlight the cell in one color for
ground objects and another for aerial objects. This communicates the concept of combining a global
grid + local 3D voxel to avoid treating flying drones as ground-level conflicts. It’s an abstract concept
visualized in an intuitive way.
Trust Badges: Next to each agent (or on their vehicle), display a small icon indicating trust. A green
checkmark ✓ means the agent’s data is trusted (e.g., authenticated and not detected as malicious),
and a red cross ✗ means the agent or its data was rejected by the trust logic. During the Activation
phase, these badges can appear and possibly animate (e.g., a flip or fade-in) as the system validates
each sensor. We also include text pop-ups in the HUD for events like “SIGNATURE VERIFIED” for when
an agent’s data is accepted, or “INVALID TOKEN” if an agent fails authentication. This adds a
cybersecurity dimension to the visual story, making it clear that GodView isn’t just merging data but
also checking trustworthiness.
Scanline Initialization Effect: To mark the GodView activation moment (45s–55s), we implement a
horizontal scanline graphic. A bright yellow line will sweep from the top of the screen to the bottom
over the Activation phase. As it passes, it can leave a slight trail or glow. This effect is a visual
metaphor for the system scanning/synchronizing all inputs. It also serves as a transition indicating
“something big is happening now.” We may synchronize a sound effect (e.g., an electronic sweep or
drum beat) with this scanline for added drama (audio handled separately in post-production).
HUD and Metrics Display: We reserve screen space for textual info and stats. A top bar can display
the phase name and a simple system status (e.g., “Baseline – Unfused” vs “GodView – Active”). A
bottom bar might show a timeline or timer (current timestamp in the simulation) and key metrics
like “Ghosts: 7” (number of duplicate objects currently), “Trusted Agents: 5/5”, “Latency: 120ms”. A
right-side log panel (optional) scrolls through events from the NDJSON logs (e.g., “Merge event:
Car#12 & Car#7 -> merged”, “Alert: Drone#3 altitude corrected”, “Attack detected from Sensor#9 –
dropped”). This text should be subtle (perhaps translucent) so as not to overwhelm the visuals, but it
provides technical viewers an extra layer of information to appreciate what’s happening under the
hood. The HUD elements collectively reinforce the before/after contrast: during Chaos, metrics show
lots of errors (ghost count high, warnings present), and after Solution, metrics normalize (ghost
count 0, all systems green).
5.
6.
7.
8.
9.
6
These overlay elements are drawn using OpenCV in the overlay_renderer.py . For each frame’s data,
we convert world coordinates to screen via a projection function (using the known camera intrinsics/
extrinsics). Then we draw shapes and text onto the frame image. For example, the following code snippet
illustrates some of the drawing steps in the overlay renderer:
# Project 3D to 2D
screen_pos = world_to_screen(x, y, z, camera_transform)
# Draw bounding box
cv2.rectangle(frame, (x-20, y-20), (x+20, y+20), color, thickness)
# Draw trust badge
cv2.putText(frame, '✓', (x+25, y-25), font, 1.0, (0,255,0), 2)
# Draw LIDAR ring
cv2.circle(frame, (cx, cy), radius_px, (200,200,255), 1)
# Draw scanline
if in_activation_phase:
y_line = int(HEIGHT * progress) # progress = 0.0→1.0 over activation
duration
cv2.line(frame, (0, y_line), (WIDTH, y_line), (0,255,255), 2)
(The above snippet assumes you have utility functions and coordinates prepared; world_to_screen would
apply the 3D-to-2D projection based on the camera view. The values (x±20, y±20) for rectangle are
placeholders for the object’s 2D bounds, which in practice would come from projected 3D bounding box corners or
a known 2D box size. Colors are in BGR format for OpenCV.)
After drawing, the frame is written out (or kept in memory) for video assembly. By modularizing this overlay
logic, we can iterate on the visuals quickly (e.g., adjust colors or add a new label) without rerunning the
simulation.
Testing & Validation Checklist
Before finalizing the video, we verify that each planned element appears correctly and the narrative is
conveyed:
Full Scene Visibility: All ~18 agents are visible during the SETUP phase’s bird’s-eye view,
establishing the environment clearly.
Drone Altitude Fix: In the CHAOS phase, drones appear erroneously flattened on the ground, and
by the SOLUTION phase they visibly rise to their proper altitude (the transition is noticeable and
correctly timed).
Ghost Merging Animation: Duplicate “ghost” object boxes are present and flickering in CHAOS,
then gradually converge and disappear by the end of the Solution phase (illustrating the merge). No
ghost duplicates remain afterward.
•
•
•
7
Malicious Object Rejection: The Sybil attack object (rendered in magenta) is shown in the Chaos
phase and is clearly marked as “REJECTED” and/or removed once GodView activates.
Stability of Fused Boxes: Green fused bounding boxes move smoothly without jitter in the
Solution phase, in contrast to the shaky red raw boxes in the Chaos phase. This before/after
difference in motion stability is visually obvious.
LIDAR & Range Indicators: The concentric range rings and any point cloud dots align with the ego
vehicle and correctly scale with perspective. They are visible in DeepDive (and possibly faintly during
other phases) and do not occlude critical scene elements.
H3 Grid Overlay: The hexagonal grid is rendered in the DeepDive phase (or where appropriate),
and the correct cells light up corresponding to agent positions. It updates or highlights in sync with
the scenario (e.g., showing which cell a drone vs a car occupies).
Trust Indicators: Every agent’s data feed gets a trust badge. All trusted agents show a green
check, and any compromised agent shows a red X. The HUD messages for verification events appear
at the right times (especially during Activation).
Timing and Synchronization: The frame overlays align with the action – e.g., the scanline
animation spans exactly the Activation phase (10s), and text/metric updates happen without lag. The
final composed MP4 is ~80s long, with correct resolution and all overlays visible, matching the
planned phase durations.
By rigorously checking the above points, we ensure the demo is technically accurate and visually
polished. The end result is a compelling demonstration where viewers witness a chaotic multi-sensor
scenario be literally brought into alignment by GodView – a transformation from entropy to consensus .
This not only showcases the robustness of GodView’s fusion algorithms (resolving latency, frame
misalignment, identity conflicts, and security issues) but does so in a memorable, cinematic fashion suitable
for a professional audience. The video will clearly communicate how GodView turns sensor chaos into
reliable truth, leaving a strong impression of improved safety and trust in autonomous vehicle perception.
CARLA Simulation for GodView Demo.pdf
file://file_00000000063c71fd8f8d5232e9e37e03
•
•
•
•
•
•
2
1 2 3 4 5 6 7
8