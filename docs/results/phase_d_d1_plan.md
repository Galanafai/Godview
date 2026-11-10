# Phase D1 step 3: settings and evaluation plan

This file is written in two commits, both before any GodView run on test.

* **Part A, the `box_size_sys_m` rule.** Committed before any GodView replay
  of val. It fixes how the one open setting is chosen.
* **Part B, the settings and the test evaluation plan.** Committed after the
  rule has been applied on val, before anything runs on test.

## Part A: rule for choosing `box_size_sys_m` (committed before any val replay)

The error model (`docs/results/phase_d_d1_error_model.md`) gives two
candidates: 0.97 m (persistent length error over all objects, set by 7 large
vehicles) and 0.50 m (cars only). The choice is made by measurement on val,
not by argument.

**Runs.** The GodView replay (`godview_sim` binary `v2v4real-replay`) on the
three val scenarios (`2022-03-15-15-20-01_0`, `2022-03-21-09-35-07_0`,
`2022-03-21-09-35-07_10`), from the step 1 export, converted by
`scripts/phase_d/convert_detections.py`. Each candidate runs with alignment
off and with alignment on (the two configurations step 4 runs on test).
Everything else is as proposed in the error model write-up, with the two
changes asked for since:

| input | value |
|---|---|
| per-detection position covariance | anisotropic: along the box `sqrt(0.267^2 + (0.00373 r)^2)`, across `sqrt(0.092^2 + (0.00347 r)^2)`, vertical `sqrt(0.067^2 + (0.00234 r)^2)` m, rotated by the detection's heading; r is the detection's horizontal range from its car. Val per-scan maximum-likelihood fits, each axis separately (`noise_fit_a_b` in `docs/results/phase_d/d1/error_model_val.json`). |
| `box_center_sys_frac` | 0.14 |
| `box_center_sys_z_m` (new) | 0.22 m: the root mean square of the val height bias (0.11 m) and persistent spread (0.19 m) |
| `sigma_size_m` per detection | 0.37 m |
| `box_size_sys_m` | **0.50 or 0.97 m (the candidates)** |
| `sigma_yaw_deg` per detection | 7.5 |
| `box_yaw_sys_deg` | 5.7 |
| `yaw_ambiguous` | true |
| `class_prob` | the detector score; class id 1 for every detection |
| pose sigmas | 1.27 m horizontal, 2.0 m vertical, 0.1 degrees (Phase C nominal) |
| replay | delay 100 ms, seed 1, scoring cutoff 5 m, all gates on |

**Attribution.** Each detection is matched to its own car's labels in the
car's lidar frame (Hungarian on horizontal centre distance, 3 m gate, the
error model's matching), and labels of the two cars are linked by `ass_id`
as in Phase C. A world object's member (one car's track) is attributed to the
label of the latest detection that track fused, as of the batch that
delivered the report. Members whose latest detection matched no label are
attributed to nothing.

**Counts,** per car and frame, summed over both cars, the three scenarios,
and alignment off and on:

* **wrong merges:** world objects whose members are attributed to two or
  more different labels (object-frames);
* **splits:** labels to which two or more world objects of the same car are
  attributed (label-frames).

**Decision.** The candidate with the smaller sum of wrong merges and splits
is chosen. A tie goes to 0.50 m. Both candidates' counts are reported,
including off and on separately, and the choice is committed in Part B. No
other value is tried.

**Known limits of the counts.** A label pair that both cars labeled but
`ass_id` does not link looks like two labels, so its correct merge counts as
a wrong merge; the same affects both candidates. A track's latest detection
may lag its report by one frame.

## Height in GodView's matching (checked)

Height enters association in three places:

1. the local tracker's gate is a 3-D NIS on position (`gate_chi2`);
2. the world model's position gate is a 3-D chi-square on the difference of
   two reports (`gate_pos_chi2`, `S = P_a + P_b`);
3. the box size gate includes the height dimension (covered by
   `sigma_size_m` and `box_size_sys_m`).

The persistent vertical error cannot split objects with these settings:

* within one car, a persistent error is the same on every scan of an object,
  so its track follows it and no gate sees it;
* between cars, `S` holds each car's declared pose vertical variance (2.0 m
  per car, so at least 8 m^2 in total). The relative persistent error of two
  cars' detections of one object is about `sqrt(2) x 0.19 = 0.27 m`, which
  adds about 0.009 to the chi-square against a gate of 21.11. The 0.11 m bias
  is the same detector on both cars and cancels in the difference.
  Alignment adds its own covariance and keeps the declared one
  (`godview_core/src/alignment.rs`), so this holds with alignment on.

It is still declared (`box_center_sys_z_m` 0.22 m), so that published
covariances cover the detector's vertical error against the labels. With a
much smaller declared pose vertical sigma (for example RTK), this argument
would not hold.

## The rule applied on val (2026-11-10)

12 replays (2 candidates, alignment off and on, 3 scenarios), CPU only.
Outputs: `docs/results/phase_d/d1/val_rule_runs.json`. Every member of every
world object had an attribution snapshot; no batch was rejected.

Wrong merges (object-frames) / splits (label-frames), both cars summed:

| scenario | 0.50 m, align off | 0.97 m, align off | 0.50 m, align on | 0.97 m, align on |
|---|---|---|---|---|
| 2022-03-15-15-20-01_0 | 80 / 1,227 | 80 / 1,227 | 60 / 1,223 | 60 / 1,223 |
| 2022-03-21-09-35-07_0 | 32 / 214 | 32 / 214 | 32 / 214 | 34 / 215 |
| 2022-03-21-09-35-07_10 | 139 / 396 | 139 / 396 | 107 / 364 | 107 / 364 |
| total | 251 / 1,837 | 251 / 1,837 | 199 / 1,801 | 201 / 1,802 |

**Sum of wrong merges and splits: 0.50 m 4,088, 0.97 m 4,091. The rule
chooses `box_size_sys_m` = 0.50 m.**

What this does and does not show:

* The two candidates differ by 3 counts out of about 4,090, all in one
  scenario with alignment on. With alignment off the counts are identical.
  On val, the size gate almost never decides an association at either value,
  so the choice matters little here; it is not evidence that 0.50 m is
  better in general.
* A likely reason, not checked: both cars run the same detector, so its
  size bias (trucks 3.5 m short) is common to both cars' boxes and cancels
  in the size gate's difference.
* Splits outnumber wrong merges about 7 to 1 at both values. Their cause
  (two tracks of one car, or one car's track not fused with the other car's)
  is not broken down yet.

## Part B: settings and test evaluation plan (committed before any test run)

### Settings

As in the Part A table, with `box_size_sys_m` = **0.50 m**. Nothing else
changes. Input conversion:

```
python -I scripts/phase_d/convert_detections.py EXPORT SCENARIO_DIR OUT_DIR \
  --sigma-h 1.27 --sigma-v 2.0 --sigma-yaw-deg 0.1 \
  --along 0.2673 0.00373 --across 0.0917 0.00347 --vert 0.0666 0.00234 \
  --sigma-size 0.37 --det-sigma-yaw-deg 7.5
v2v4real-replay --dir OUT_DIR --out RESULT.json --box-size-sys-m 0.50 \
  --box-yaw-sys-deg 5.7 --box-center-sys-frac 0.14 --box-center-sys-z-m 0.22 [--align]
```

Test has the gps field, so its map frame is anchored by the Phase C fit
(`gps_fit.py`); val used a made-up anchor shared by both cars and the truth.

### Runs on test (once)

* test1, test2, test3: 9 scenarios, 1,993 frames, from the step 1 export
  (SHA-256 `796590db...6fe7`).
* Alignment off and alignment on, everything else fixed.
* Reference, labeled as such: each car alone (`--no-sharing`), alignment
  off, for the gained and false-from-sharing comparison and for AP of
  GodView without sharing.
* If a run has to be redone because of a bug, the first result is kept and
  the new one is added as a separately labeled run.

### Metrics

1. **The dataset's AP@0.5 and AP@0.7** (`opencood` `eval_utils`, as in D0
   and step 1) of car 0's GodView world, in car 0's lidar frame, against the
   ground truth the dataset's evaluation builds (`gt_corners_ego` in the
   export: both cars' labels, deduplicated by object ID, x within 100 m and
   y within 40 m), every test frame. Car 0 is the only ego the dataset's
   evaluation uses (step 1, finding 4).
   * Each world object becomes a box in car 0's lidar frame: centre from its
     fused ECEF position through the shared anchor and car 0's `lidar_pose`
     (both exact here), size from the fused box, heading from the fused ENU
     heading minus the anchor's yaw and car 0's heading. Objects without a
     box are left out (none expected: every detection has one). The same
     x and y range mask is applied.
   * **Score: the world object's `class_prob`,** the largest over its
     members of each track's running mean of its detections' scores. This
     differs from the brief's wording ("existence probability, the maximum
     of its members' detector scores"): GodView's `p_exist` is
     `hits / (hits + 1)` of the best member's track and is not a detector
     score. As a second, labeled AP, `p_exist` is used as the score.
   * Next to it: No Fusion 39.81 / 22.02 and Late Fusion 55.07 / 26.63 from
     D0, and the same AP by range (0 to 30, 30 to 50, 50 to 100 m).
   * Before test, the AP script is checked on val: applied to car 0's raw
     export it must reproduce step 1's val No Fusion AP (46.0 / 25.7) to
     within 0.1. That check is reported with the test results.
2. **GodView's own metrics** as in `docs/results/v2v4real.md`: matched,
   missed, duplicates (split across cars), false (coasting), RMS and NEES
   (all matched and objects labeled by both cars), fused / both detect, ID
   changes per object, gained objects, box size NEES and heading error. The
   expected rule now uses the detector: an object is expected if a detection
   of some car was attributed to it in this and the previous 3 frames.
3. **Association by attribution** (Part A definitions): wrong merges
   (object-frames) and splits (label-frames).
4. **False objects introduced by sharing, per car and frame:** world objects
   of that car that the scorer leaves unmatched and that are farther than
   5 m from every label (the "false" category), whose only source is the
   other car. Reported per frame, next to objects gained (object-frames per
   frame and distinct objects): what a car gains from sharing beside the
   false objects it receives.
5. All of the above for all 9 scenarios, and separately without
   `2022-03-15-09-54-40_0`, the test scenario also present in `train_01`
   (step 1, finding 1).

### Caveats to state with the results

* The truth is the labels, which have their own errors (sizes differ by up
  to about 1 m between the two cars' annotators).
* GodView keeps uncertain objects rather than suppressing them, which AP may
  punish: about 2 false positives per car and frame reach the world.
* The detector most likely saw `2022-03-15-09-54-40_0` in training.
* Heading enters AP@0.7 through the box overlap, and every detection is
  ambiguous by 180 degrees; the IoU is symmetric under that flip, so the
  ambiguity itself costs nothing in AP.
* GPU time: none for steps 2 and 3 (CPU only).
