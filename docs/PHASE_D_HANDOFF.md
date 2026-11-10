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
