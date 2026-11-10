# Phase D: real detectors, four agents, and a demo on real footage

Brief for a Claude Code session on a RunPod GPU pod. Read `CLAUDE.md`,
`docs/WRITEUP.md` and `docs/PHASE_C_HANDOFF.md` first. The data rules there
still apply: zips are untrusted, each goes in its own empty directory, check
paths before unpacking, run Python that reads them with `-I`.

## Goal

Every real-data number so far feeds GodView the dataset's labels with added
noise. This phase replaces them with a real lidar detector, moves from two
cars to four real agents, and ends in a demo cut from real footage. Three
results, each its own post:

1. **Proof (V2V4Real):** GodView against the dataset's published late-fusion
   baseline, same detector, the dataset's own evaluation.
2. **Four agents (V2X-Real):** two cars and two roadside units, a real
   detector, and whether a roadside unit (fixed, surveyed position) gives the
   GPS offset estimation the anchor it lacked with two cars.
3. **Demo:** an object hidden from one agent's camera, seen by another,
   shown on the hidden side's real camera footage before it comes into view.

## Rules for every step

* Each step ends with a commit and push on `feat/real-detector`, and a short
  report to Galanafai. Do not merge.
* Settings are chosen on train or val only and committed before anything
  runs on test. Test runs once. If a test number must be redone because of a
  bug, keep the first result and add the new one as a separately labeled run.
* Gates: if a gate fails, stop and report. Do not work around it by changing
  what is measured.
* Quick checks only, no long sweeps. Note the GPU hours used in each report.
* The writing rules in `CLAUDE.md` apply: plain, measured, and say what is
  not proven.

## D0. Environment (gate)

Both datasets' code is OpenCOOD with PointPillars: Python 3.7, PyTorch 1.12
with CUDA 11.3, spconv-cu113. Use a GPU with compute capability 8.6 or lower
(RTX 3090, A5000, A40); CUDA 11.3 builds have no kernels for newer cards.
Install under `/workspace` (conda or venv) so it survives a pod stop.

On a newer GPU (RTX 4090, L4, L40S: compute capability 8.9), the CUDA 11.3
builds will not run. Use Python 3.8 or later, PyTorch 2.0.x with CUDA 11.8 and
spconv-cu118 instead, and fix only what the newer versions break (record each
change). The gate below decides whether that environment is acceptable.

**Gate:** the V2V4Real repository's own evaluation of its published
checkpoints reproduces its README on the test split to within about 1 AP
point: No Fusion AP@0.5 39.8 and AP@0.7 22.0; Late Fusion 55.0 and 26.7
(sync). If a checkpoint link is missing or broken, stop and report.

## D1. Proof on V2V4Real

1. Export each car's detections (boxes in its own lidar frame, with scores)
   for train (or a documented part of it), val and test, from the per-car
   detector the late-fusion baseline uses. Record the score threshold.
2. Detector error model, from train and val only: match detections to labels
   and measure position error against range, size error, yaw error
   (including front and back flips), false positives and misses. Turn it into
   the replay's noise and declared systematic errors.
3. **Commit the settings and this evaluation plan before any test run.**
4. Run once on test (test1, test2, test3), alignment off and on:
   * The dataset's evaluation (AP@0.5, AP@0.7 in each ego car's frame) of
     GodView's fused world. A fused object's score is its existence
     probability (the maximum of its members' detector scores). Put it next
     to No Fusion and Late Fusion from D0.
   * GodView's own metrics as in `docs/results/v2v4real.md`.
   * Caveats: the truth is the labels, and GodView keeps uncertain objects
     rather than suppressing them, which AP may punish.

## D2. V2X-Real, first contact with labels

Project page: https://mobility-lab.seas.ucla.edu/v2x-real (direct Google
Drive and Box links per split). Two vehicles (128-beam lidar, four stereo
cameras at 1920x1080) and two roadside units (Ouster lidar, two cameras),
10 Hz, multi-class (cars, trucks, pedestrians, cyclists and more).

1. Download one split first (val), report its size, then the rest only if
   needed. Check the format the way Phase C did for V2V4Real (poses, frames,
   class names, box conventions, timestamps per agent, camera calibration)
   and write the findings into a new section of this file.
2. Extend the converter to four agents and the classes GodView supports.
3. Labels plus noise, as in Phase C: settings chosen on val. This checks the
   converter and the four-agent pipeline before a detector is involved.

## D3. V2X-Real with a real detector

1. Look for published V2X-Real checkpoints for single-agent (no fusion)
   detection. If there are none, train PointPillars no-fusion with the
   repository's own config on train. **Gate:** the repository's evaluation
   reproduces the paper's No Fusion mAP (about 28.2 at IoU 0.3 and 21.2 at
   0.5, vehicle-centric) to within about 2 points. Report GPU hours.
2. Error model on train and val, commit, then one test run, as in D1, with
   alignment off, on, and on with the roadside units as anchors (their poses
   held fixed). Pre-register the anchor variant with the others. Report
   GodView's metrics and, if the evaluation code allows it, the dataset's mAP
   next to its No Fusion and Late Fusion.

## D4. Choose the demo moment by rule, then export it

Write the rule down and commit it before searching. Proposed rule: among test
scenes, the longest interval in which an object is within 50 m of a vehicle,
inside that vehicle's front camera field of view, has no lidar points from
that vehicle inside its box (occluded), is in that vehicle's GodView world
because another agent detected it, and afterwards becomes directly visible
to that vehicle. Prefer a pedestrian or cyclist if one qualifies; otherwise
take the longest. Report the top three candidates and which one the rule
picks. Do not choose by appearance.

Export a scene package (a few hundred MB at most) for rendering:
* per frame: every agent's pose; each agent's detections; the chosen
  vehicle's GodView world (position, covariance, box, heading, ID, class,
  and whether the vehicle saw it itself or only through sharing); labels;
* which agents contributed to each fused object (for the arcs between them);
* the chosen vehicle's front camera images and calibration for the interval;
* a static environment point cloud (lidar aggregated over the scene, voxel
  downsampled) and per-frame moving points, subsampled.

Also record the same interval with Rerun (`rerun-sdk`) to an `.rrd` file,
the inspectable version for engineers.

## D5. Render (with Galanafai, and the session that wrote this brief)

Blender (Cycles, on the GPU) in headless mode, from the scene package:
* each object as a generic, unbranded 3D model (car, truck, pedestrian,
  cyclist) placed at GodView's estimate and scaled to its box; models CC0 or
  credited;
* solid for objects the vehicle sees itself; translucent glowing ghosts for
  objects known only through sharing; a soft halo for each covariance;
* the real street as a subtle particle field from the static point cloud;
* arcs between agents when they share reports, only for real messages;
* a camera overlay: the ghost box projected into the real front camera
  frames, drawn over the occluder, until the object comes into view.

Edit: real footage of the blocked view, the 3D flythrough, the ghost
appearing, back to the real footage as the object comes into view, then an
end card with the measured numbers. No retiming except labeled slow motion.

## D6. Write up

New sections in `docs/results/` (separately labeled, existing tables
unchanged) and updates to `docs/WRITEUP.md`: "Not a real detector" and "Two
cars only" under failures, and whatever replaces them.

## D2 step 1 findings: V2X-Real val (2026-11-10)

Measured on the val split of the LiDAR-Cameras version. Scripts:
`scripts/phase_d/survey_v2xreal.py`, `survey_v2xreal_ids.py`; outputs in
`docs/results/phase_d/d2/` (`val_survey.json`, `val_survey_ids.json`,
`val_overlap.txt`). CPU only, no GPU time. Nothing else was downloaded.

### What was downloaded

The project page (https://mobility-lab.seas.ucla.edu/v2x-real/) links three
versions of the dataset in OPV2V layout, each with a test zip, a val zip and
four train zips on UCLA Box (shared single files, plain download works with
`index.php?rm=box_download_shared_file&shared_name=...&file_id=f_...`):

| version | what | val.zip |
|---|---|---|
| LiDAR-64 | vehicle lidar with 64 lines; infrastructure lidar 128 and 64 lines | 2.39 GB |
| LiDAR-128 | vehicle lidar 128 lines; infrastructure 128 and 64 | 4.90 GB |
| LiDAR-Cameras | all LiDAR-128 data plus cameras (the page calls it the most complete) | 9.22 GB |

Downloaded: LiDAR-Cameras `val.zip`, 9,215,643,036 bytes, SHA-1
`2f8803c60d82e578ccc47f3d1ecf43a4eb84def2` (equals Box's recorded value), to
`/workspace/data/v2xreal/zips/val_cameras/`. Chosen because it is the superset
and the only version with the cameras and their calibration (the LiDAR-128 val
at 4.90 GB would do for lidar only; not compared against this one). Safety:
entries listed first (13,782, no absolute paths, `..` or symlinks), unpacked
into its own directory (11 GB), read with `python -I` and a safe YAML loader
(the files hold no Python tags).

### Layout

* `val/<scenario>/<agent>/<frame>.{bin,yaml}` and `<frame>_cam<k>.jpeg`;
  6 scenarios (`2023-03-17-16-03-02_11_1`, `2023-03-17-16-10-12_1_1`,
  `2023-04-04-14-28-53_45_0`, `2023-04-04-15-43-17_15_0`,
  `2023-04-04-16-01-17_33_0`, `2023-04-05-16-09-26_6_1`) of 98 to 140 frames
  (717 frames in all, 2,770 agent-frames). The number pair after the date is
  not explained by any file.
* **Agents:** positive folder ids are vehicles (`1`, `2`), negative ones are
  roadside units (`-1`, `-2`; the yaml has `infra: true`). Five scenarios have
  all four; `45_0` has vehicle 2 and the two units only. Frame ids are the
  same set for every agent of a scenario.
* **Point cloud** `.bin`: float32, 4 values per point (x, y, z, intensity 0 to
  255) in the sensor's own frame. Per frame: vehicles 189,728 to 226,455
  points (median 220,728); unit `-1` always 65,536 (64 lines x 1,024), unit
  `-2` always 131,072 (128 x 1,024). Points reach about 192 m.
* **yaml keys:** `lidar_pose`, `true_ego_pose`, `ego_speed`, `infra`,
  `vehicles` (labels), `cam1` to `cam4` (vehicles) or `cam1`, `cam2` (units).
  No timestamps, no GPS, no calibration of the lidar itself.

### Conventions, as measured

* **Poses.** `lidar_pose` is `[x, y, z, roll, yaw, pitch]` in metres and
  degrees, in one shared map frame (OpenCOOD's `x_to_world` matrix). Vehicle 1
  of the first scenario drives from y = -50.6 to -2.2 with yaw 88 to 96 degrees,
  which is facing +y, so yaw is counterclockwise from +x. `true_ego_pose` equals
  `lidar_pose` in every file checked (23 of 23 agent-scenarios, first 20
  frames), so there is no separate noisy GPS pose in the files; how
  `lidar_pose` was obtained is not documented. **The two roadside units have
  identical poses in all six scenarios** (unit `-1` at (61.98, -177.00, 4.08),
  unit `-2` at (29.25, -157.99, 2.50)) and the map frame is the same across
  scenarios, so there is one fixed frame for the whole split. How far the unit
  poses are surveyed is not documented.
* **Labels are in the world (map) frame.** Tested directly: of 10,316 labels
  within 80 m of an agent, 99.3% have at least 5 of that agent's own lidar
  points inside the box (median 64) when the label is transformed with the
  agent's `lidar_pose`; read as already in the lidar frame, 0.2% do (median 0).
  This differs from V2V4Real, whose labels were in each car's lidar frame.
* **Box fields.** `location` = centre (m, map frame); `extent` = half size
  `[l/2, w/2, h/2]` (a Car is about 4.5 x 2.0 x 1.5 m; the half-size reading is
  what gives the point counts above); `angle` = `[roll, yaw, pitch]` in degrees
  (yaw is `angle[1]`, same convention as the poses); `center` is `[0, 0, 0]` in
  every label seen; `obj_type` is the class name; `attribute` empty.
* **Class names (sampled frames, all agents):** Pedestrian 12,880 (59%), Car
  6,527, Bus 656, TrashCan 342, Van 337, Truck 330, ConcreteTruck 329,
  ScooterRider 200, BicycleRider 69, Scooter 4. The PointPillars config in the
  V2X-Real repository uses 3 classes (vehicle, pedestrian, truck; anchors
  3.9 x 1.6 x 1.56, 0.8 x 0.6 x 1.73, 8 x 3 x 3 m; lidar range +-70.4 x +-40
  m; score threshold 0.20; NMS 0.15). The mapping from the ten names to those
  three was not read.
* **Label ids and sets.** Ids are consistent across agents: the same id in two
  agents is the same object, with the two agents' boxes 0.29 m apart at the
  median and 0.67 m at the 90th percentile (7,860 pairs identical to 1 cm, 20,176
  differing; maximum 40 m). That is much closer than V2V4Real's two cars (0.7
  to 3.8 m, 90th percentile by scenario) and needs no `ass_id` linking. Each
  agent's label list is its own set, about 34 labels per frame (vehicles
  median 34, max 114; units median 33, max 58); only 6.8% of label occurrences
  appear in all agents' lists. Of the 42,679 labels whose id is missing from the
  other agent of a pair, 1,260 (3%) have a same-class label within 1 m under
  another id, so id mismatches are rare and the sets differ by what each agent
  can see.
* **A recording vehicle is not in its own list.** The nearest label to a
  vehicle's own lidar position is 7.1 m away at the median (none within 2.5 m,
  272 frames sampled). Another vehicle's list holds it within 2.5 m 64% of the
  time (median 0.72 m); a unit's list only 26%. This is the same structure as
  V2V4Real, where the scored car's own report arrives in its own world
  (`docs/results/phase_d_d1_test2.md`).
* **Time.** The files have no timestamps; the V2X-Real code itself calls the
  file names "mocked timestamps" and assumes all agents have the same frame
  list. `ego_speed` is 0 in every frame, so the frame rate cannot be checked
  from it. The paper's 10 Hz is assumed; vehicle 1 of the first scenario moves
  0.86 m per frame, 8.6 m/s at 10 Hz, which is plausible. No per-agent time
  offsets are given.
* **Cameras.** All 1,920 x 1,080 JPEG. Per camera: `intrinsic` 3 x 3 (focal
  length 966 to 1,247 px, principal point near the image centre; they differ
  per camera), `extrinsic` 4 x 4, and `cords` (the camera's world pose, 6
  values). **`extrinsic` is camera-to-lidar with OpenCV camera axes** (x right,
  y down, z forward): for vehicle 1 `cam1` its columns are (-y, -z, +x) of
  the lidar frame, and projecting labeled objects in front of a camera with
  the inverse of `extrinsic` lands 93% of them inside the image (1,531 of
  1,638) against 42% (2,381 of 5,675) with `extrinsic` used the other way.
  Vehicles have 4 cameras (front, two rear, back by the OPV2V convention, not
  verified); the units 2.
* **Images.** Real footage; the unit camera looks down on a signalized
  intersection (Westwood Plaza) with pedestrians; the vehicle front camera
  shows the street ahead and the hood.

### How much the agents see of each other (val)

(`docs/results/phase_d/d2/val_overlap.txt`) The two units are 37.9 m apart and
share about 20 labels per frame in every scenario. The two vehicles are close
in four scenarios (median 8 to 39 m, 22 to 45 shared labels per frame) and far
in the other two (256 m and 194 m, 0 to 1 shared labels), so the first
scenario (`11_1`) has no vehicle-to-vehicle overlap at all. Units and vehicles
overlap from 2 to 33 shared labels per frame depending on the scenario.

### Not determined

* Where the map frame is on the Earth: there is no GPS field. The Earth anchor
  will have to be chosen as for V2V4Real val (shared by all agents and the
  truth); whether the unit poses are surveyed, and how accurate the vehicles'
  `lidar_pose` is, is not documented, so the pose sigmas for GodView need to
  be declared and justified in step 2.
* The real frame rate and the agents' time alignment.
* The mapping of the ten class names to the repository's three classes, and to
  GodView's classes.
* Anything about train and test (not downloaded): their scenarios, whether any
  scenario appears in more than one split (V2V4Real had one), and the pretrained
  checkpoints for D3.
