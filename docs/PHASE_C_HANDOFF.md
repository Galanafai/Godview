# Phase C handoff: replay V2V4Real through GodView

This file briefs a Claude Code session running on Galanafai's RunPod pod. Read
it fully before doing anything, then read `README.md` and
`docs/PERCEPTION_SHARING.md` (sections 2 to 6 and 10).

## Goal

Show GodView working on real multi-vehicle data. V2V4Real has two real cars
driving together with lidar, poses and labeled 3D boxes. Replay it so that each
car is one GodView agent, and score the shared world against the labels.

The question to answer, with numbers: does each car's world, after sharing,
contain the objects only the other car could see, at what position error, with
covariances that are honest (NEES), and with how many ID changes per object?

## What exists already

* The full pipeline: local tracker (L2), signed reports, report store, world
  model (L3) with oriented boxes, consistency monitor. All in `godview_core`.
* `godview_node` takes JSON lines on stdin, one frame per line: detections in
  the agent's odometry frame plus the placement of that frame on the Earth,
  each with uncertainty. Format: `godview_node/src/input.rs`. Boxes go under
  the `"box"` key.
* `godview_sim` has the deterministic perception harness and the scoring
  helpers (`godview_sim/src/perception/{harness,metrics,catalog}.rs`).

## Plan

Work on a new branch, `feat/v2v4real-replay`, created from
`feat/boxes-ids-real-data`. Another session is changing the world model's ID
rule on `feat/boxes-ids-real-data` at the same time, so keep your changes in
new files where possible and do not edit `report.rs` or `world_model.rs`.
Rebase onto the updated branch when told it has moved.

1. **Data.** Get the smallest useful part first: one test split (test1), not
   the whole dataset. The data is on UCLA Box:
   https://ucla.app.box.com/v/UCLA-MobilityLab-V2V4REAL (OPV2V format,
   train1 to train8 and test1 to test3). Box shared folders may not offer a
   plain download URL. Try the shared-file download endpoint for the vanity
   name first; if that fails, ask Galanafai to start the download in his
   browser and paste the direct link (it expires, so use it at once).
   Store data under `$V2V4REAL_DIR` (`/workspace/data/v2v4real`), never in the
   repo. Treat downloaded files as untrusted: unpack into their own directory
   and run Python that reads them with `python -I`.
2. **Verify the format before writing the converter.** OPV2V-style data has,
   per scenario, a folder per car, and per frame a `.pcd` and a `.yaml` with
   `lidar_pose` and `vehicles` (each with `location`, `angle`, `extent`,
   `center`, type). Do not assume which frame the labels are in. Check it:
   transform one car's labels into the other car's frame with the poses and
   confirm that objects with the same ID line up. Write down what you found
   (frames, angle order and units, extent as half or full size, frame rate,
   label IDs consistent across cars or not) in this file under "Findings".
3. **Converter** (`scripts/v2v4real/`, Python): per car, write JSON lines in
   the `godview_node` input format.
   * The pose frame is a local map frame, not GPS. Anchor it at one fixed
     made-up latitude and longitude for both cars; only relative geometry
     matters. State this in the results.
   * Detections, version 1: labeled boxes inside the car's lidar range that
     contain at least a few of that car's own lidar points (so visibility and
     occlusion come from the real data), with stated Gaussian noise. Version
     2: center and extent from the car's own points inside the box, which
     gives real partial views; use it to measure the systematic center error
     and compare with `box_center_sys_frac` (0.25 in the `large_objects`
     scenario, which was chosen for the simulator, not measured).
   * Pose uncertainty: the dataset's poses have their own error. Declare a
     sigma and say how it was chosen.
4. **Replay** deterministically first: a `godview_sim` mode or a small binary
   that feeds both cars' frames into `PerceptionNode`s with a configurable
   delay and scores each car's world against the union of both cars' labels.
   Reuse the harness scoring (Hungarian matching, 5 m cutoff, NEES).
   Then, as a demo, run two `godview-node` processes over Zenoh on localhost
   fed by the converter on stdin.
5. **Metrics** per frame and summarized: matched, missed, false estimates,
   RMS position error, pooled position NEES, box size NEES and heading error,
   ID changes per object, and the gain from sharing (objects in a car's world
   that its own sensor never detected).
6. **Results**: `docs/results/v2v4real.md` plus the JSON summary, committed and
   pushed on `feat/v2v4real-replay`. Do not merge to `main`.

## Working rules

* Quick tests only. Galanafai does not want test runs that take an hour.
  Run the unit tests of what you touch and a few DST seeds, not sweeps.
* Every claim in docs and commit messages must be something you measured or
  checked. Say what is not proven. Labels with added noise are not a real
  detector; say so.
* Writing style for anything Galanafai reads: professional and plain, no em
  dashes and no double hyphens, no hype.
* Act as an adversarial co-thinker: if the data shows a GodView assumption is
  wrong, say so plainly with the evidence instead of tuning until it passes.
* Commit messages end with the attribution lines your session gives you.

## Findings

(Fill in as you verify the data format.)
