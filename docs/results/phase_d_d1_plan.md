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

## Val diagnostics before step 4 (2026-11-10, val only, no setting changed)

### Where the 1,837 splits come from (alignment off; alignment on 1,801)

`scripts/phase_d/split_breakdown.py` on `v2v4real-replay --split-diag`
(`docs/results/phase_d/d1/val_splits_breakdown_*.json`). Each split label-frame
is classed from the tracks attributed to the label.

| class | cause | label-frames |
|---|---|---|
| (a) two tracks from the same car | track restart: the younger track was created after the older one's last update | 1,115 |
| | both tracks started by detections of the label, overlapping in time | 456 |
| | track started by a false detection beside the real one | 126 |
| | other | 10 |
| (b) the two cars' tracks of one label not fused | the other car's partner track already sits in the object (same-source rule) | 104 |
| | velocity gate | 18 |
| | heading gate | 8 |
| (c) other | | 0 |

* (a) is 93 percent of splits (1,707). It is not about the cars: 852 label-frames
  are two tracks of the scored car, 855 two tracks of the other car.
* Of the restarts, 934 start the frame after the old track's last update and
  181 two to five frames after. The old track stays in the world while it
  coasts, so one dropped track is counted over many frames. False detections
  beside real ones are 126 of 1,707 (7 percent).
* What starts the second track: of 169 tracks started next to a live track of
  the same label, 120 fail the tracker's position gate (the rest: heading 33,
  size 17). Those innovations are large along the box: the median is 1.5 m
  along against 0.5 m across, and the tracks' median speed is 25 m/s (the
  odometry frame is fixed to the map, so oncoming cars move fast and a 100 ms
  step is 2.5 m).
* A second cause is outside the settings: 1,208 of the 1,707 (a) label-frames
  are in one scenario (`2022-03-15-15-20-01_0`) and car 1's step between
  consecutive poses jumps there (85 frames deviate more than 0.5 m from the
  local median step, against 9 for car 0). 73 of 120 position-gate failures
  are at such frames, and 796 of 1,115 restarts begin at one. This is
  attributed by timing only; whether the cause is the pose or the detector is
  not separated.
* Other split sources are small: one dominant cause with a single-setting fix
  does not exist (the largest, restarts, mixes pose jumps with fast
  oncoming objects). **No committed setting is changed.**

### Alignment moves the world away from the dataset's frame

On val, car 0's world scored by the dataset's AP (below) loses AP with alignment
on, mostly at IoU 0.7. Alignment shifts fused objects by a median of 0.62 m
(car 1's reports 0.61 m; car 0's own objects 0.0 m, 90th percentile 0.93 m)
compared with alignment off. The dataset's ground truth is built from the same
`lidar_pose` of both cars, without this correction. Whether the alignment
corrects a real pose error that the dataset's evaluation ignores, or errs, is
not decided by this. GodView's own metrics, which score against the average of both cars'
labels, change little: RMS per car and scenario is 0.82 to 1.25 m with
alignment off and 0.84 to 1.27 m on, with the first scenario slightly worse
on (0.84 / 0.82 to 0.92 / 0.88 m).

### AP on val (car 0's world; the AP script is validated)

`scripts/phase_d/ap_eval.py` reproduces the library's AP implementation
exactly on every method (asserted to 1e-9) and the step 1 val numbers: No
Fusion 45.95 / 25.73 (step 1: 45.95 / 25.73), Late Fusion 52.01 / 28.80
(step 1: 52.01 / 28.80), AP@0.5 / AP@0.7. 9,447 ground-truth boxes, all
identified with a label.

| method (val, 748 frames) | all frames | excluding each label's first 3 frames |
|---|---|---|
| No Fusion | 45.95 / 25.73 | 46.14 / 25.83 |
| Late Fusion | 52.01 / 28.80 | 52.30 / 28.99 |
| GodView, alignment off, score class_prob | 50.23 / 29.80 | 51.34 / 30.47 |
| GodView, alignment off, score p_exist | 50.24 / 29.79 | 51.35 / 30.46 |
| GodView, alignment on, score class_prob | 39.58 / 9.13 | 40.37 / 9.27 |
| GodView, alignment on, score p_exist | 39.58 / 9.13 | 40.38 / 9.27 |

Full output: `docs/results/phase_d/d1/ap_val.json`. Read with care: these are
val numbers from a detector that did better on train, for settings chosen on
val; the AP scoring compares only within frames (see below).

## Additions to the test evaluation plan (committed before any test run)

1. **AP on all frames and, as a labeled second number, excluding each
   label's first 3 frames.** GodView publishes confirmed tracks only (three
   hits) and delivers peers' reports 100 ms late, while late fusion scores
   every frame's raw detections, so GodView loses recall at the start of
   each object's track. The second number ignores a label's first 3 frames
   in the ground truth of its scenario (not counted as ground truth, and a
   detection matched to one is neither a true nor a false positive). It
   removes a start-up effect; it does not correct the first number. Both
   are reported for No Fusion, Late Fusion and GodView, overall and in the
   ranges 0 to 30, 30 to 50 and 50 to 100 m. The script is
   `scripts/phase_d/ap_eval.py`.
2. **Reading the size setting.** The cross-car association gate is
   dominated by the frozen pose sigma (1.27 m horizontal and 2.0 m vertical
   per car), which sits in the position gate's covariance, and the box size
   gate has the same detector bias in both cars' boxes. That is why changing
   `box_size_sys_m` from 0.50 to 0.97 m made no measurable difference on val.
   Results on test are not evidence about `box_size_sys_m` either.
3. **AP scoring detail.** The library concatenates true and false positives
   in frame order and does not sort them globally by score, so a score
   ranks detections only within a frame. Both the library and
   `ap_eval.py` do so, and the numbers above and on test are comparable to
   D0 for that reason.
4. **Alignment on is expected to lose AP on test** if the val behaviour
   holds (it shifts car 1's reports by a median of 0.6 m relative to the
   frame the dataset's truth uses). Pre-stated here so that the test result
   is not read as a surprise: the test number is reported as measured.

## Tracker ablation on val: protocol and selection rule

Written and committed before the ablation runs. Val only; test is not touched.
Everything not named below is the committed setting (Part B above: 0.50 m
size, 0.14 centre fraction, 0.22 m vertical, 5.7 degrees heading, anisotropic
covariance, pose 1.27 / 2.0 m / 0.1 degrees, 100 ms delay, alignment off
unless stated). AP is car 0's, all frames, from `scripts/phase_d/ap_eval.py`.
New tracker and publishing options (all default to the committed behaviour,
tested in `godview_core`): `--fresh-only`, `--score-mode`, `--raw-box`,
`--overlap-iou`, `--overlap-keep`, `--confirm-hits`, `--pub-min-score`.

**1. Passthrough (no sharing).** The unchanged tracker, scored four ways:
the world object's own box and score (the committed way); its member's latest
raw detection box, centre and score (`pass`); filtered box with the latest raw
score (`rawscore`); latest raw box and centre with the filtered score
(`rawbox`). Next to No Fusion. If `pass` recovers No Fusion's AP, the loss
is in boxes and scores; if it does not, it is in which tracks exist.

**2. Ablations (no sharing), one at a time against the baseline:**
(a) `--fresh-only`: publish only tracks updated by a detection at the publish
time (a track that stops qualifying gets one tombstone-form report, so it
leaves the world at once);
(b) `--overlap-iou 0.3`: of two tracks of one class whose predicted boxes
overlap by more than 0.3 bird's-eye-view IoU, drop one, keeping the older
(`--overlap-keep older`, as asked). The value 0.3 is fixed in advance, not
swept. Added by me: (b2) the same keeping the more recently updated track,
because the dominant split cause on val is an old coasting track next to its
restart, which "keep the older" keeps;
(c) score = latest detection (`latest`) or maximum (`max`) instead of the
running mean;
(d) `--raw-box`: published size and heading are the latest detection's
(association still uses the filtered box);
(e) `--confirm-hits 1` instead of 3.
Reported for each: AP@0.5 / AP@0.7, duplicates (estimates within 5 m of a
label, unmatched), split label-frames, coasting object-frames, world objects
per frame.

**3. Selection rule.**

* An alternative is *adopted* if it raises no-sharing AP@0.5 by at least 0.5
  points over the baseline. Within (b) and (c) only the better of the two
  alternatives can be adopted.
* Stage S1, with sharing (alignment off): the baseline, each adopted
  alternative alone, and all adopted alternatives together. The best is the
  highest AP@0.5; differences under 0.1 points are ties, resolved by fewer
  false objects introduced by sharing per car-frame (the plan's definition).
* Stage S2: the S1 winner with a publish threshold on the track score of 0.2,
  0.3 and 0.4 (tracks below it are not published). Same criterion over the
  S1 winner and the three thresholds. The detector's own threshold is 0.2, so
  0.2 changes nothing under the running-mean score and serves as a check.
* The result of S2 is the proposed tracker setting. AP@0.7, the counts and
  alignment-on results are reported and do not enter the rule.

**4. Alignment gauge (not in the rule).** With alignment on and the shared world
unchanged, each car's view is scored shifted back by that car's own estimated
translation (`shift`), against the unshifted view, for the committed tracker
and for the S2 winner. It tests whether the AP@0.7 loss from alignment is the
frame offset only.

Limits stated now: val has three scenarios and one dominates the splits;
a setting chosen here is not shown to hold on test. A second test run, if
run, is labeled separately and does not replace the first.
