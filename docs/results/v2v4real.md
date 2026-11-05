# V2V4Real replay, test1 split (version 1 detections)

Measured 2026-11-05 on branch `feat/v2v4real-replay`. Raw numbers:
`docs/results/v2v4real_summary.json`. Data format findings:
`docs/PHASE_C_HANDOFF.md`, "Findings".

## What was run

* Data: V2V4Real test1, 3 scenarios (`09-54-40_0`, `10-29-43_3`,
  `10-29-43_4`; 147, 114 and 144 frames), two real cars each. Frames are
  replayed at 10 Hz (assumed from the paper; the files have no timestamps).
* Each car is one GodView `PerceptionNode`, fed through the same JSON-lines
  parser as `godview-node` (`godview_node::input::parse_line`). Peer batches
  arrive 100 ms after publishing. Default `NodeConfig` everywhere, except in
  the runs marked "no size gate" (a diagnostic, not a proposed setting).
* Detections (version 1): a car detects a labeled box if the box is within
  120 m and holds at least 5 of the car's own lidar points; the detection is
  the label plus seeded Gaussian noise (position 0.3 m, size 0.2 m, heading
  3 degrees, all declared). **This is not a real detector.** Visibility and
  occlusion come from the real lidar, but positions, sizes and headings come
  from human labels. Version 2 (boxes from the car's own points) is not done.
* All vehicle types map to one class id, because the cars disagree on class
  for one object (Van and Truck) and GodView fuses only equal classes.
* Earth anchor: each scenario's map frame is placed on the Earth by a rigid
  fit of `lidar_pose` to the dataset's gps field (residual 0.16 to 0.22 m
  rms horizontally). Both cars and the truth use the same anchor, so its
  error cancels; it is not a made-up position.
* Truth: the union of both cars' labels, linked by `ass_id`. An object's
  position is the mean of the labels of the cars that labeled it, its size
  the mean size, its heading that of the car with more lidar points on it.
* Scoring per car and frame: Hungarian matching against every labeled
  object, 5 m cutoff. An object is expected if some car detected it in this
  frame and the 3 frames before. Each car is excluded from its own truth.
  * Duplicate: an extra estimate within 5 m of a labeled object, so the
    object appears twice. "Split" means the two estimates come from
    different cars.
  * False: an extra estimate farther than 5 m from every label.
  * Gained: an object in a car's world that the car's own sensor never
    detected up to that frame.
  * ID change: the world ID matched to an object differs from the one
    matched last time.

## Pose uncertainty declarations

| declaration | sigma h (per axis) / v | how it was chosen |
|---|---|---|
| (a) nominal | 1.27 / 2.0 m, yaw 0.1 deg | Datasheet, standalone GNSS (no RTK is documented for V2V4Real). OxTS RT3000 v3: SPS position 1.5 m CEP, which is 1.27 m per axis for a circular Gaussian, and heading 0.1 deg 1 sigma. The NovAtel E1 family sheet gives 1.5 m RMS single point L1, so 1.27 m covers it as well. Vertical 2.0 m is an assumption: neither sheet found gives a vertical figure. |
| (b) from 09-54-40_0 | 0.73 / 0.93 m | Leave one out. sigma_h = sqrt(E\|d_h\|^2 / 4) and sigma_v = sqrt(E[d_z^2] / 2) over the relative offsets d of objects both cars labeled in that scenario, so that two independent cars' difference is calibrated. Scored on the other two. |
| (b) from 10-29-43_3 | 1.31 / 2.33 m | Same rule. |
| (b) from 10-29-43_4 | 0.29 / 0.57 m | Same rule. |
| reference RTK | 0.01 / 0.02 m | RT3000 headline RTK figure. Not a candidate: the measured offsets rule it out. Shown to make the failure mode visible. |

Yaw sigma is 0.1 deg in every declaration. GodView applies pose yaw error
about the odometry origin (here the scenario's mean car position), not about
the car, so the lever arm is the distance to that point (detections are up to 167
to 180 m from it) instead of the range from the sensor. At 0.1 deg this adds
at most about 0.3 m; the mismatch itself is not measured.

Sources for the datasheet figures: the V2V4Real project page lists the units
(Tesla: RT3000, Ford: NovAtel SPAN E1) but gives no accuracy figures
(https://mobility-lab.seas.ucla.edu/?p=2004). The RT3000 v3 datasheet
(https://positics.fr/wp-content/uploads/2020/09/RT3000-V3.pdf, read through
search results) gives SPS 1.5 m CEP, RTK 0.01 m and heading 0.1 deg. The
NovAtel PwrPak7-E1 sheet
(https://www.navtechgps.com/wp-content/uploads/PwrPak7-E1_DS.pdf) gives
single point L1 1.5 m RMS. The exact unit version and correction mode used
in the dataset are not documented. The V2V4Real paper itself could not be
fetched (too large).

## Results, default configuration

Counts are summed over both cars and the scored scenarios. RMS and NEES are
given as "all matched / objects labeled by both cars". Only the second is
informative about pose: for an object only one car labeled, the truth went
through that car's own pose, so the pose error cancels.

| declaration | scored on | matched | missed | duplicates (split) | false (coasting) | RMS m | NEES 3D (expect 3) | NEES horiz (2) / vert (1) | both detect: fused into one | ID changes / objects | gained objects (object-frames) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| (a) nominal | all 3 | 3001 | 3 | 385 (327) | 392 (369) | 0.93 / 1.41 | 0.35 / 0.79 | 0.24 / 0.12 | 742 / 997 | 118 / 50 = 2.36 | 18 (679) |
| (b) from 40_0 | 43_3, 43_4 | 1846 | 0 | 368 (306) | 345 (322) | 0.97 / 1.74 | 1.26 / 4.02 | 0.67 / 0.60 | 147 / 424 | 98 / 34 = 2.88 | 12 (389) |
| (b) from 43_3 | 40_0, 43_4 | 1833 | 3 | 143 (111) | 107 (107) | 0.72 / 0.98 | 0.21 / 0.37 | 0.17 / 0.04 | 607 / 697 | 53 / 28 = 1.89 | 11 (426) |
| (b) from 43_4 | 40_0, 43_3 | 2323 | 3 | 393 (342) | 381 (358) | 0.99 / 1.44 | 4.73 / 9.98 | 3.34 / 1.39 | 611 / 873 | 109 / 38 = 2.87 | 13 (543) |
| reference RTK | all 3 | 3001 | 3 | 994 (908) | 424 (401) | 0.87 / 1.31 | 15.94 / 36.87 | 6.83 / 9.11 | 177 / 997 | 234 / 50 = 4.68 | 18 (679) |

Boxes (default configuration, nominal): box size NEES 3.35 (expect 3).
Heading error median 1.6 to 2.1 deg per car and scenario, 95th percentile at
most 8.0 deg, maximum 46.5 deg. The maximum is the turning car that car 0
misannotated (see Findings).

Per scenario, nominal declaration, default configuration:

| scenario | car | matched | duplicates | false | RMS both-labeled m | NEES 3D both-labeled | fused / both detect | ID changes / objects |
|---|---|---|---|---|---|---|---|---|
| 09-54-40_0 | 0 | 579 | 18 | 32 | 1.00 | 0.44 | 284 / 287 | 8 / 8 |
| 09-54-40_0 | 1 | 576 | 19 | 32 | 1.07 | 0.51 | 284 / 286 | 7 / 8 |
| 10-29-43_3 | 0 | 584 | 119 | 143 | 2.08 | 1.65 | 66 / 150 | 35 / 11 |
| 10-29-43_3 | 1 | 584 | 123 | 142 | 2.02 | 1.56 | 69 / 150 | 30 / 11 |
| 10-29-43_4 | 0 | 339 | 56 | 21 | 0.66 | 0.15 | 17 / 62 | 23 / 6 |
| 10-29-43_4 | 1 | 339 | 50 | 22 | 0.68 | 0.15 | 22 / 62 | 15 / 6 |

## Diagnostics

Size gate disabled in the world model (everything else the same):

| declaration | fused / both detect | duplicates (split) | ID changes / objects |
|---|---|---|---|
| (a) nominal | 895 / 997 (was 742) | 202 (147) (was 385 (327)) | 62 / 50 = 1.24 (was 2.36) |
| (b) from 40_0 | 303 / 424 (was 147) | 194 (135) | 43 / 34 = 1.26 |
| (b) from 43_3 | 684 / 697 (was 607) | 66 (34) | 20 / 28 = 0.71 |
| (b) from 43_4 | 629 / 873 (was 611) | 359 (311) | 100 / 38 = 2.63 |

Horizontal and vertical sigma crossed (smallest and largest leave-one-out
values), scored on 09-54-40_0 and 10-29-43_3, objects both cars detect that
were fused into one:

| sigma h / v | default config | no size gate | NEES horiz / vert |
|---|---|---|---|
| 1.31 / 2.33 | 703 / 873 | 779 / 873 | 0.27 / 0.10 |
| 1.31 / 0.57 | 661 / 873 | 722 / 873 | 0.27 / 1.44 |
| 0.29 / 2.33 | 633 / 873 | 703 / 873 | 3.36 / 0.10 |
| 0.29 / 0.57 | 611 / 873 | 629 / 873 | 3.34 / 1.39 |

## Relative offset between the cars, per frame

The same object, labeled by both cars and placed with each car's own pose
(ENU, car 0 minus car 1). Per-frame means are in the JSON summary
(`relative_offset.<scenario>.per_frame`).

| scenario | pairs | mean E, N, U m | std E, N, U m | median \|horiz\| m | max \|horiz\| m | per-frame mean \|horiz\|, min to max m |
|---|---|---|---|---|---|---|
| 09-54-40_0 | 290 | -0.34, 1.18, 0.25 | 0.50, 0.59, 1.29 | 1.30 | 3.91 | 0.29 to 3.89 |
| 10-29-43_3 | 170 | 0.16, 2.31, -1.10 | 0.88, 0.88, 3.11 | 2.11 | 4.46 | 0.82 to 3.85 |
| 10-29-43_4 | 62 | -0.02, -0.35, 0.24 | 0.35, 0.29, 0.77 | 0.54 | 1.36 | 0.09 to 1.36 |

These counts are a little higher than in the Findings (280, 150, 62 pairs;
largest offset 3.84 m), because the converter links a car 1 label ID to car
0 for the whole scenario once `ass_id` links it in any frame.

## Findings

1. **Sharing adds objects.** 679 of 3001 matched object-frames (23%) under
   the nominal declaration are objects the car's own sensor had never
   detected up to that frame: 18 distinct objects summed over cars and
   scenarios. Missed expected objects: 3 of 3004.
2. **An understated pose sigma creates duplicates, as the design predicts.**
   With the 1 cm RTK sigma only 177 of 997 objects both cars detect become
   one world object, against 742 under the nominal sigma. Duplicates rise
   from 385 to 994. No overconfident single object hides the problem. The
   consistency monitor never judges anyone with two cars (`min_partners` is
   3), so nothing catches it.
3. **A sigma chosen on one scenario does not transfer.** The rule that
   calibrates one scenario gives NEES on both-labeled objects of 4.02, 0.37
   and 9.98 on the other two (expect 3). The relative offset differs by a
   factor of about 4 between scenarios, even between two segments of the same
   drive (10-29-43_3 and 10-29-43_4: median 2.11 and 0.54 m). The largest leave-one-out sigma (1.31 m) is close to the
   datasheet value (1.27 m) and overcovers the other two scenarios.
4. **Vertical has to be declared on its own.** With horizontal covered (1.31
   m) and the size gate off, lowering vertical sigma from 2.33 to 0.57 m
   reduces fusion of objects both cars detect from 779 to 722 of 873
   (default configuration: 703 to 661), and vertical NEES rises from 0.11 to
   1.53 (default: 0.10 to 1.44). Most of this comes from
   10-29-43_3, where the vertical offset std is 3.1 m.
5. **The box size gate is the largest single cause of splits, even when the
   pose is covered.** The cars' annotators give the same object sizes that
   differ by up to 1.06 m. Each car's labels keep one size per object, so a
   track's size variance shrinks as it averages frames, and two converged
   tracks then fail the 99.99% size gate. Disabling only that gate raises
   nominal fusion from 742 to 895 of 997 and halves ID changes (118 to 62).
   This is the size counterpart of the systematic center error that
   `box_center_sys_frac` handles. GodView has no way to declare a systematic
   size error, and the input format has no field for it. Whether a real
   detector shows the same effect is not tested; with version 1 the
   disagreement comes from the annotators.
6. **ID changes are frequent.** Nominal: 2.36 changes per object (22 of 50
   objects change at least once), most of them the object switching between
   one fused and two split world objects. With the size gate off: 1.24.
   Lineage IDs were not tested here against stable fusion, because fusion is
   not stable in this data.
7. **False estimates are coasting tracks.** 369 of 392 (nominal) carry the ID
   of an object matched earlier. They appear in runs of about 15 frames
   (1.5 s, `terminate_after_s`) after an object leaves the labeled set, for
   example at scenario 4's 72 m annotation limit. Whether those objects were
   really gone is not known.
8. **Heading.** Label headings never flip front/back in test1. The heading
   gate still splits one object in 10-29-43_3, which one car misannotated by
   up to 44 deg while it turned. In that scenario the velocity gate also
   splits some pairs (with the size and heading gates off, splits fall from
   46 to 16 frames for car 0 when the velocity gate is also off); this was
   not analyzed further.

## What is not proven

* Labels with noise are not a real detector. The results test association,
  fusion and pose handling on real geometry, real visibility and real pose
  error, not detection quality.
* There is no ground truth for the poses. The truth is the mean of the two
  cars' labels, so every number about pose accuracy is relative to the cars'
  mean, and NEES on both-labeled objects is only a consistency check against
  that mean. For single-labeled objects, NEES and RMS do not include pose
  error at all.
* 10 Hz is assumed. The 100 ms delay is a chosen value, not measured.
* Three scenarios and about 400 frames are a small sample. The leave-one-out
  result shows the pose offset varies between scenarios more than the sample
  can characterize.
* The pose yaw lever arm is modeled about the scenario origin, not the car.
* The two-process Zenoh demo (step 4, second part) is not done: `godview-node`
  stamps tracks with the input clock and expects it to be live, so the
  converter would need to rebase timestamps to the current time and a feeder
  would need to pace the lines at 10 Hz.

## Reproduce

```sh
# data: see docs/PHASE_C_HANDOFF.md (Download); about 2 minutes in total
cargo build --release -p godview_sim --bin v2v4real-replay
/workspace/venv/bin/python -I scripts/v2v4real/run_all.py \
    $V2V4REAL_DIR/test1 $V2V4REAL_DIR/converted docs/results/v2v4real_summary.json
```

The replay is deterministic: two full runs gave identical numbers.
