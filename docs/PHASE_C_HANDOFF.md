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
`feat/boxes-ids-real-data`. Another session may keep working on
`feat/boxes-ids-real-data`, so keep your changes in new files where possible
and do not edit `report.rs` or `world_model.rs` without asking. World-object
IDs now follow tracks through handoffs (`IdRule::Lineage`); count ID changes
per object in the replay, since real data is the first real test of that.

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

Checked on the test1 split only (2026-11-05). Scripts:
`scripts/v2v4real/v2v4real_io.py` (safe readers) and
`scripts/v2v4real/verify/*.py`; each takes `$V2V4REAL_DIR/test1`.

### Download

* The shared-file endpoint works without a browser:
  `https://ucla.app.box.com/index.php?rm=box_download_shared_file&vanity_name=UCLA-MobilityLab-V2V4REAL&file_id=f_<id>`.
  File IDs come from the `Box.postStreamData` JSON in the folder page
  (`/v/UCLA-MobilityLab-V2V4REAL/folder/279924274808` is `Data`).
  `test_01.zip` is `f_1619923463037`, 633582640 bytes, which matches the size
  Box lists. Other IDs: test_02 `f_1619910080470`, test_03
  `f_1619915891150`, val `f_1619910191481`, train_01 `f_1619935662984`.
* test1 unpacks to 1.77 GB under `$V2V4REAL_DIR/test1`: three scenarios
  (`testoutput_CAV_data_2022-03-15-09-54-40_0`, `..._10-29-43_3`,
  `..._10-29-43_4`) with 147, 114 and 144 frames, 405 frames in total, two
  cars (folders `0` and `1`) each. No path in the zip escapes the target
  directory.

### File format

* The YAML files hold numpy objects under Python tags. `yaml.unsafe_load`
  would run arbitrary constructors, so `v2v4real_io.V2VLoader` handles only
  the numpy scalar, dtype and ndarray tags and decodes bytes with
  `np.frombuffer`.
* Keys per frame: `ego_speed`, `gps`, `lidar_pose`, `true_ego_pos`,
  `vehicles`. `lidar_pose` is a 4x4 homogeneous matrix (not the 6-vector of
  OPV2V); `true_ego_pos` is the same object (a YAML alias). `ego_speed` is 0
  in every frame checked, so it is unusable.
* `gps` is `[lat, lon, alt, heading_rad, a, b]`, near 39.98, -83.02
  (Columbus, Ohio). The heading equals the `lidar_pose` yaw plus a constant per
  car (1.00 degrees for car 0, 2.50 for car 1, standard deviation 0.00 over
  all frames). The per-frame displacement from GPS and from `lidar_pose`
  agree to 0.1 to 0.3 percent. So `lidar_pose` is a metric local map frame
  built from the GPS/INS solution, not an independent localization.
* `.pcd` files are ASCII with fields `x y z rgb`, about 35k to 45k points,
  maximum range 130 m, in the car's own lidar frame.
* Each vehicle label has `location`, `angle`, `extent`, `center` (always
  `[0, 0, 0]` where seen), `obj_type` (`Car`, `Truck`, `Van`,
  `ConcreteTruck` in test1) and `ass_id`. The recording car itself is not in
  its own list; the other car is.

### Frames and conventions

* Labels are in each car's own lidar frame. With that reading, 51 of 52
  sampled boxes contain at least 5 of the car's own lidar points (median 82).
* `extent` is half size: 98 percent of boxes hold at least 5 points when
  `extent` is used as half size, 35 percent when used as full size.
* `angle` is `[roll, yaw, pitch]` in degrees; yaw is the second element and
  adds to the pose yaw (world yaw = pose yaw + `angle[1]`). For the same
  object seen by both cars the world yaw differs by a median of -0.6, 1.0 and
  0.7 degrees (95th percentile of the absolute difference 8.5, 40.8 and 3.6
  degrees); with the sign of `angle[1]` flipped the medians are -27, 2.6 and
  -24.5 degrees. Radians are ruled out the same way (fewer points inside the
  box). The 40.8 degree tail in scenario 3 is one misannotated object; see
  "Heading tail in scenario 3" below.
* `location` is the box center: relative to `location[2]`, the lidar points
  inside a box span from -0.96 to between +0.84 and +0.95 times `extent[2]`
  (medians per scenario).
* Labels reach 118 to 130 m in scenarios 0 and 3, but stop at 72 m in
  scenario 4. Annotation range is not uniform across scenarios.
* Frame rate: the files contain no timestamps. The car moves a median 1.6 to
  2.1 m per frame, which is 56 to 75 km/h at the 10 Hz stated in the
  V2V4Real paper. 10 Hz is assumed, not verified from the data.

### Label IDs across cars

* Numeric label IDs are per car. Of 389 car 0 labels that land within 2 m of
  a car 1 label after the pose transform, none has the same numeric ID.
* The cross-car link is `ass_id` on car 1's labels: it holds the car 0 label
  ID of the same object (`-1` when there is none). All 492 car 1 labels with
  `ass_id` set have that car 0 ID present in the same frame. `ass_id` on car
  0's labels is -1 in all 951 car 0 labels of test1.
* Label IDs are stable over time within a car (for example car 0 id 1 maps
  to car 1 id 7 in 128 frames of scenario 0). Not yet checked: whether a label
  ID is ever reused for a different object.

### Relative pose error between the cars (matters for GodView)

The same object seen by both cars (linked by `ass_id`) does not land in the
same place after the pose transform:

| scenario | pairs | mean offset in car 1 frame (x, y) m | std (x, y) m | median abs xy m | max m |
|---|---|---|---|---|---|
| 09-54-40_0 | 280 | 0.96, -0.63 | 0.63, 0.45 | 1.26 | 2.73 |
| 10-29-43_3 | 150 | 2.15, -0.04 | 0.64, 0.86 | 2.09 | 3.84 |
| 10-29-43_4 | 62 | -0.21, 0.26 | 0.44, 0.14 | 0.51 | 1.35 |

* It is a common offset per frame, not independent noise per object: in
  25-frame windows the mean offset moves slowly (scenario 0: 0.8 m, then 1.2,
  1.6, 1.8, 1.1, 0.7 m in magnitude) while the spread inside a window is 0.1
  to 0.7 m.
* It is not a time offset between the two lidar sweeps: the offset along the
  object's motion does not grow with object speed (correlation -0.28, -0.23,
  -0.06), and nearly stationary objects in scenario 3 (17 pairs) are still
  off by 1.9 m on average.
* Vertical offsets are larger (std 0.8 to 3.1 m), consistent with pitch or
  altitude error over long ranges.
* What this does not separate: error in car 0's pose, in car 1's pose, in the
  lidar to GPS/INS mounting, and annotation differences between the cars.
  Only the relative error is observable without ground truth poses.

Consequence for the replay (corrected after review): GodView does not
average pose error down over time. Tracks live in each car's odometry frame,
the pose covariance is added fresh at every publish, and the world model fuses
the latest reports with Covariance Intersection. So a pose sigma that
understates this offset does not show up as one overconfident object; it
shows up as the object appearing twice, because the pair fails the 99.99%
position gate. The declared sigma has to cover a relative offset of up to
about 4 m horizontally (and several metres vertically), correlated over many
seconds. With two cars the consistency monitor never judges either of them
(`min_partners` is 3), so nothing downstream catches an understated sigma.
The replay confirms this: with a 1 cm RTK sigma, 994 extra estimates
(duplicates) appear against 385 with the nominal sigma (docs/results/v2v4real.md).

### Heading tail in scenario 3 (explained)

* The 40.8 degree 95th percentile comes from one object only (car 0 id 3 =
  car 1 id 2, frames 15 to 31, 17 pairs). The disagreement grows smoothly
  from 15 to 44 degrees and back; it is not a front/back flip and not a wrap
  artifact (the wrap is handled; car 0's raw yaw sits near +-180 degrees).
* The object pulls away from a near stop and turns: its direction of motion
  from the labeled positions goes from -88 to -153 degrees. Car 1's label
  (24 to 170 lidar points) follows the turn. Car 0's label (5 to 25 points)
  stays at about -95 degrees. So car 0 has one wrong heading annotation on a
  sparsely seen object.
* Labels do not flip front/back in test1: 0 of 492 cross-car pairs differ by
  about 180 degrees, and 0 of 2,348 consecutive-frame steps of one car's own
  label tracks jump by more than 20 degrees, except two 24 degree steps of
  one car 1 object in scenario 0. Converted detections therefore carry
  `yaw_ambiguous: false`.
* Script: `scripts/v2v4real/verify/check_heading_tail.py`.

### Earth anchor from the gps field

* Per scenario, a 2-D rigid fit of `lidar_pose` (both cars, all frames) to
  the gps latitude and longitude in local ENU (`scripts/v2v4real/gps_fit.py`):

| scenario | map x axis from East | horizontal residual rms / max | height residual rms |
|---|---|---|---|
| 09-54-40_0 | 1.39 deg | 0.16 / 0.43 m | 1.29 m |
| 10-29-43_3 | 1.02 deg | 0.22 / 0.56 m | 2.34 m |
| 10-29-43_4 | 1.36 deg | 0.18 / 0.56 m | 0.44 m |

* The residual includes the GPS antenna to lidar lever arm, which is not
  modeled. The three fits put the map origin (about 780 m from where the
  cars drive) 5 to 23 m apart, so the scenarios plausibly share one map
  frame, but the fits do not agree exactly. Each scenario uses its own fit.
* After the rotation, gps heading minus pose yaw is -0.4 to 0.0 degrees for
  car 0 and 1.1 to 1.5 degrees for car 1. Which car's heading is right is not
  known.

### Sizes and classes across cars

* The two cars label the same object with different sizes: over the 8 linked
  objects the absolute difference has median 0.14, 0.05, 0.13 m (length,
  width, height) and maximum 1.06, 0.48, 0.52 m. Within one car a label ID
  keeps one size over time for 23 of 36 IDs.
* Linked pairs disagree on class in 79 pair-frames of scenario 3 (one object:
  Van for car 0, Truck for car 1). GodView only fuses equal class ids, so the
  converter maps every vehicle type to one class.
* Script: `scripts/v2v4real/verify/check_size_agreement.py`.

### test2 and test3

Checked on 2026-11-05 with the same scripts. The format matches the
findings above; the differences (front/back flips across cars, more object
types, denser scenes, car 1 labels linked to two car 0 IDs over time,
possible missing links, stationary cars) are listed in
`docs/results/v2v4real.md`, "Held-out confirmation (test2, test3)".
