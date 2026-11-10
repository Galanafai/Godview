# Phase D1: test run 2 (tracker settings chosen on val)

**Separately labeled second run on V2V4Real test.** Run 1
(`docs/results/phase_d_d1_test.md`, the settings committed in
`docs/results/phase_d_d1_plan.md`) is unchanged. The runs below are exactly the
ones declared in `docs/results/phase_d_d1_test2_plan.md` (commit `f21cb18`,
before the run): 9 scenarios, 1,993 frames, alignment off, CPU only. Raw
outputs: `docs/results/phase_d/d1/ap2_test_*.json`,
`godview_test2_metrics*.json`.

Two things to keep in mind when reading the numbers:

* **The tracker settings were chosen on val** among a few options, scored on
  the same frames (`docs/results/phase_d_d1_ablation.md`). This is their first
  test. The 0.3 publish threshold in particular is a val choice.
* **The dataset's AP rewards fewer boxes.** It pools true and false positives
  in frame order and never ranks them by score across frames, so removing boxes
  raises it. A standard AP (VOC 2010 all-point, all frames pooled and ranked by
  score; validated on val, `docs/results/phase_d/d1/ap_validation.json`) is
  reported next to it. The same 0.3 threshold applied to the baselines shows
  the effect: No Fusion rises from 39.85 to 45.59 and Late Fusion from 55.00 to
  65.49 under the dataset AP, and both fall under the standard AP.

## Runs

| run | what |
|---|---|
| R1 (primary) | GodView, selected setting (fresh-only, overlap prune 0.3 keep recent, publish threshold 0.3), sharing |
| R2 | the same, no sharing |
| R3 | the same without fresh-only (coasting tracks kept), sharing |
| NF@T, LF@T | No Fusion (car 0 alone) and Late Fusion (both cars, joint NMS) on the export's detections with score at least T = 0.2 (published) and 0.3 |

## AP, all 9 scenarios (AP@0.5 / AP@0.7, percent)

| method | dataset AP | standard AP |
|---|---|---|
| No Fusion @0.2 | 39.85 / 22.05 | 57.47 / 38.00 |
| No Fusion @0.3 | 45.59 / 27.90 | 53.61 / 36.85 |
| Late Fusion @0.2 | 55.00 / 26.75 | 77.84 / 42.87 |
| Late Fusion @0.3 | 65.49 / 33.87 | 74.69 / 42.13 |
| **R1 GodView, sharing** | **62.95 / 34.82** | **72.46 / 42.10** |
| R2 GodView, no sharing | 46.63 / 28.41 | 54.09 / 35.00 |
| R3 GodView, coasting kept, sharing | 57.13 / 30.11 | 72.04 / 41.48 |

Boxes in range: NF 29,843 (0.2) / 21,857 (0.3); LF 42,536 / 32,269; R1 30,585;
R2 21,694; R3 34,764. The dataset AP reproduces step 1's No Fusion and Late
Fusion numbers (39.85 / 22.05, 55.00 / 26.75) and the dataset AP of
every method equals the library's functions to 1e-9 (asserted in the run).

### The lead of R1 (primary), in points

| R1 minus | dataset AP@0.5 / @0.7 | standard AP@0.5 / @0.7 |
|---|---|---|
| No Fusion @0.2 | +23.1 / +12.8 | +15.0 / +4.1 |
| No Fusion @0.3 | +17.4 / +6.9 | +18.9 / +5.3 |
| Late Fusion @0.2 | **+8.0 / +8.1** | **-5.4 / -0.8** |
| Late Fusion @0.3 | **-2.5 / +1.0** | **-2.2 / 0.0** |

* **Over No Fusion, GodView leads under both definitions and both thresholds.**
* **Over Late Fusion there is no robust lead.** The +8 points over the
  published Late Fusion under the dataset AP is a threshold effect: with the
  baseline at the same 0.3 threshold the lead is gone at AP@0.5 (-2.5) and
  1.0 point at AP@0.7. Under the standard AP, GodView is behind Late Fusion
  at AP@0.5 at both thresholds (-5.4, -2.2) and level at AP@0.7 (-0.8, 0.0).
* The 0.3 baselines are matched to GodView's threshold, not tuned; a
  baseline-optimal threshold was not searched for.
* Without `2022-03-15-09-54-40_0` (the scenario also present in `train_01`;
  1,846 frames) the picture is the same: dataset AP R1 62.83 / 34.14 against
  Late Fusion @0.3 65.70 / 33.70; standard AP 72.24 / 41.51 against 74.62 /
  41.65 (`ap2_test_*_no09.json`).

### What sharing and keeping coasting tracks do

| | dataset AP | standard AP |
|---|---|---|
| sharing (R1 minus R2) | +16.3 / +6.4 | +18.4 / +7.1 |
| tracker without sharing (R2 minus NF@0.3) | +1.0 / +0.5 | +0.5 / -1.9 |
| tracker without sharing (R2 minus NF@0.2) | +6.8 / +6.4 | -3.4 / -3.0 |
| keeping coasting tracks (R3 minus R1) | **-5.8 / -4.7** | **-0.4 / -0.6** |

* Sharing gives about 16 to 18 AP@0.5 points under either definition: that
  result does not depend on the AP definition.
* Without sharing, the tracker is level with the detector at the same
  threshold under both definitions, and below it at AP@0.7 under the standard
  AP when the detector is at its published threshold. No tracker gain beyond
  filtering boxes is shown.
* **Keeping coasting tracks costs 5.8 / 4.7 points under the dataset AP and
  0.4 / 0.6 under the standard AP.** Most of the dataset-AP penalty of
  persistence is how that metric counts boxes; ranked by score, the coasting
  objects cost under one point.

### By range (AP@0.5 / AP@0.7)

| method | 0 to 30 m | 30 to 50 m | 50 to 100 m |
|---|---|---|---|
| dataset AP | | | |
| No Fusion @0.2 / @0.3 | 69.2 / 42.7, 71.8 / 47.3 | 29.3 / 14.4, 34.9 / 20.2 | 4.8 / 1.6, 5.8 / 2.4 |
| Late Fusion @0.2 / @0.3 | 73.2 / 36.9, 78.5 / 41.4 | 44.0 / 22.4, 53.9 / 29.6 | 36.3 / 17.3, 50.6 / 25.8 |
| R1 | 77.7 / 46.7 | 53.4 / 28.9 | 47.3 / 22.5 |
| R2 | 71.5 / 46.7 | 38.4 / 22.0 | 7.4 / 3.1 |
| R3 | 75.2 / 44.7 | 47.3 / 24.9 | 39.1 / 17.6 |
| standard AP | | | |
| No Fusion @0.2 / @0.3 | 79.6 / 55.2, 75.9 / 53.9 | 47.0 / 29.5, 42.6 / 28.3 | 6.7 / 2.7, 5.4 / 2.5 |
| Late Fusion @0.2 / @0.3 | 86.1 / 46.9, 83.6 / 46.4 | 66.4 / 40.1, 62.6 / 39.0 | 66.7 / 36.4, 63.6 / 35.7 |
| R1 | 83.1 / 51.8 | 59.2 / 33.8 | 55.0 / 25.4 |
| R2 | 75.1 / 50.7 | 45.1 / 28.8 | 6.9 / 3.1 |
| R3 | 83.3 / 51.7 | 58.5 / 33.1 | 52.0 / 23.5 |

* Near range (0 to 30 m) GodView's AP@0.7 is at least 5 points above Late
  Fusion under both definitions (51.8 against 46.4 to 46.9 standard; 46.7
  against 36.9 to 41.4 dataset), and level at AP@0.5 against Late Fusion @0.3.
* From 30 m on, Late Fusion is ahead under the standard AP (by 3.4 to 11.7 points
  at AP@0.5, 5 to 11 at AP@0.7) and, at 0.3, under the dataset AP at 50 to 100 m.
* With each label's first 3 frames ignored (`excl_first_3` in the json), the
  ordering is unchanged (R1 64.2 / 35.5 dataset, 74.1 / 43.0 standard; Late
  Fusion @0.3 65.9 / 34.0 and 75.4 / 42.5).

## GodView's own metrics (both cars, 3,986 car-frames)

| | run 1 (committed tracker, sharing) | R1 | R2 | R3 |
|---|---|---|---|---|
| matched | 53,351 | 51,021 | 39,524 | 51,302 |
| **missed** | 307 | **2,637** | 14,134 | 2,356 |
| duplicates (split across cars) | 4,598 (2,868) | 779 (625) | 178 (0) | 1,285 (938) |
| false | 48,224 | 8,233 | 4,701 | 15,685 |
| coasting object-frames | n/a | 0 | 0 | 5,703 |
| world objects per car-frame | n/a | 16.4 | 11.8 | 18.9 |
| RMS, all matched / labeled by both (m) | 0.69 / 0.74 | 0.68 / 0.76 | 0.72 / 0.75 | 0.69 / 0.76 |
| NEES 3D, labeled by both (expect 3) | 0.27 | 0.28 | 0.28 | 0.28 |
| box size NEES (expect 3) | 1.47 | 1.36 | 1.29 | 1.37 |
| both detect: fused into one | 22,355 / 23,160 | 19,861 / 22,281 | 0 / 21,364 | 20,081 / 22,319 |
| ID changes per object | 1.02 | 1.03 | 0.52 | 0.68 |
| wrong merges (object-frames) | 2,154 | 2,010 | 0 | 2,149 |
| splits (label-frames) | 3,327 | 860 | 178 | 1,092 |
| objects gained, per car-frame (distinct) | 2.83 (194) | 2.71 (179) | 0.49 (47) | 2.72 (180) |
| **false objects introduced by sharing, per car-frame** | 4.81 | **0.89** | 0 | 1.62 |

* The selected setting cuts false objects from sharing from 4.81 to 0.89 per
  car-frame and keeps 2.71 gained object-frames, at the cost of 2,637 misses
  against 307 (of about 53,000 expected object-frames): objects whose
  detections drop for a frame leave the world.
* R3 (coasting kept) has 281 fewer misses than R1 (2,356 against 2,637) and
  1.62 against 0.89 false objects from sharing per car-frame. The expected
  rule counts an object only if it was detected in the last 4 frames, so it
  cannot value persistence beyond that; the benefit of remembered objects
  (the demo's case) is not measured here.
* Wrong merges are 2,010 on test against 251 on val. Two test scenarios have a
  car 1 label linked to several car 0 labels, which this count reads as wrong
  merges (see run 1); the rest was not investigated.
* Same table without `2022-03-15-09-54-40_0`:
  `godview_test2_metrics_no09.json` (false from sharing 0.93, 0.0 and 1.66 per
  car-frame for R1 to R3).

## Not shown

* That the selected setting is better than other settings on test: it was
  chosen on val and run once here. No other setting was run on test.
* A lead over Late Fusion: under the published threshold and the dataset AP
  there is one; under every other combination it is absent or reversed.
* The value of remembered objects: no metric here counts them.
* Why GodView's far-range AP trails Late Fusion's (not analysed).

## Explanation only: wrong merges and the far-range gap (test run 2, R1)

Added after the run, to explain two numbers. **No setting was changed and
nothing here was re-selected.** Any fix will be validated on V2X-Real, not on
V2V4Real test. The replay got two diagnostic outputs for this (`--merge-diag`,
and a `labels` field per world object in the dump); rerunning R1 and R3 with
them reproduced the headline counts exactly (R1: 2,010 wrong merges, 860
splits, 2,637 missed, 3,535 false-from-sharing object-frames; R3: 2,149,
1,092, 2,356, 6,441). Scripts: `scripts/phase_d/explain_merges.py`,
`scripts/phase_d/far_range_analysis.py`; outputs
`docs/results/phase_d/d1/test2_wrong_merges_r1.json`,
`test2_far_range.json`.

### 1. The 2,010 wrong merges

A wrong merge is a world object whose members' latest detections were
attributed to two or more labels, counted per car and frame.

| class | object-frames | share |
|---|---|---|
| counting artifact: one car 1 label linked by `ass_id` to several car 0 labels (re-numbered labels of one object) | 329 | 16.4% |
| two labels at the scale of the cars' own disagreement (at most the scenario's 90th percentile, 2 to 3 m), cannot be told from one object labeled twice | 14 | 0.7% |
| label absent in this frame | 3 | 0.1% |
| **two distinct labels (2 to 10 m apart)** | **1,664** | **82.8%** |

* **By scenario:** `09-35-07_7` 744 (37%, one label pair in both cars for the
  whole scenario); `11-02-23_2` 447 (22%); `10-50-46_1` 395 (20%);
  `11-02-23_1` 393 (20%); `10-50-46_0` 31 (1.5%); the other four scenarios 0.
  The two scenarios with a multi-linked car 1 label (`10-50-46_1`,
  `11-02-23_1`) hold 788 events; 329 of them are the artifact (21 and 308), the
  other 459 are not. Three label pairs, each seen from both cars, make 72% of
  all events (episodes of 372, 207 and 149 frames per car).
* **Distance between the merged labels (distinct, 1,664):** 2 to 3 m: 170;
  3 to 5 m: 970; 5 to 10 m: 524; none closer than 2 m.
* **Range from the scored car (distinct):** 0 to 20 m: 1,524 (92%); 20 to
  40 m: 103; 40 to 60 m: 31; 60 to 80 m: 6.
* **What they are:** 1,116 of the 1,664 (67%; 56% of all 2,010) contain the
  scored car itself, as the other car labels it (a label only the other car
  has, within 2.5 m of the scored car): the other car's detection of this car
  arrives in this car's world and is fused with a neighbour or with this car's
  detection of the other car. The three dominant pairs are (a) a Van 3.9 m
  from car 0 with car 1's label of car 0 (`09-35-07_7`, static); (b) the two
  recording cars labeling each other, 2.5 to 15.6 m apart, one of them
  moving (`11-02-23_2`); (c) the same, 7.6 m apart and static
  (`10-50-46_1`).
* **Verdict:** a counting artifact for 16%, not for the bulk. The rest are
  real merges of distinct objects, mostly an unrecognized self-report. They
  are allowed by the cross-car gate, which is dominated by the declared pose
  sigma (1.27 m per axis per car, so a gate radius of up to about 8 m for
  reports on different cars) and nothing in the node recognizes a report about
  itself. The same count on val was 251, because val has no scenario where
  the two cars see each other this closely; this was not tested further.
  The "same object labeled twice" class is small (0.7%), so label noise does
  not explain the count. Label positions in this breakdown use the current
  frame's labels, while a member's latest detection can be older.

### 2. The far-range gap (30 m and beyond)

Ground-truth boxes with centroid 30 to 100 m from car 0: 8,575 (30 to 50 m)
and 6,164 (50 to 100 m). "Covered" = a box of the method with bird's-eye IoU
of at least 0.5.

| method | covered, 30 to 50 m | covered, 50 to 100 m | boxes in range (of which cover a ground-truth box) 30 to 50 / 50 to 100 m |
|---|---|---|---|
| Late Fusion @0.2 | 6,736 (78.6%) | 5,044 (81.8%) | 11,785 (6,716) / 11,523 (5,043) |
| Late Fusion @0.3 | 6,238 (72.7%) | 4,689 (76.1%) | 8,265 (6,219) / 7,235 (4,688) |
| R1 GodView | 5,984 (69.8%) | 4,420 (71.7%) | 8,012 (5,980) / 7,103 (4,464) |
| R3 (coasting kept) | 6,122 (71.4%) | 4,604 (74.7%) | 9,332 (6,121) / 9,288 (4,671) |

R1 and Late Fusion @0.3 have nearly the same share of boxes that cover a
ground-truth box (30 to 50 m: 74.6% against 75.2%; 50 to 100 m: 62.8% against
64.8%). The gap is in recall.

Why R1 does not cover the boxes Late Fusion @0.3 covers (1,648 box-frames
over both ranges; first reason that applies):

| reason | count | share |
|---|---|---|
| a box of R1 near the object but IoU below 0.5 | 1,138 | 69% |
| detected, no R1 or R3 track (confirmation after 3 hits, gating, pruning) | 342 | 21% |
| not detected by either car (this frame, or the previous for car 1) | 93 | 6% |
| detected only below the 0.3 publish threshold | 53 | 3% |
| **R3 covers it (fresh-only dropped it)** | **14** | **0.8%** |
| **wrong merge near the box** | **8** | **0.5%** |

**Neither wrong merges nor fresh-only drives the gap.** What does:

* The localization cases (1,940 for R1 over both ranges, against all
  Late-Fusion-covered or not): 97% would pass IoU 0.5 if the centre were
  right, so the box shape is not the problem. The offset is mostly across the
  box (median 0.84 m) rather than along it (0.32 m).
* They are mostly slow or parked objects: 1,477 of 1,940 under 3 m/s. By
  object speed, R1 covers more than Late Fusion @0.3 for objects over 10 m/s
  (2,464 against 2,438 of 3,099) and from 3 to 10 m/s (1,453 against 1,431
  of 2,089), and fewer under 3 m/s (6,375 against 6,882 of 9,232). So the
  100 ms delay on fast objects is not the cause either.
* 1,111 of the 1,940 (57%) are objects fused from both cars, 829 (43%) from
  one car. The cause is not determined. A candidate: the ground truth is both
  cars' labels, which disagree by 0.7 to 3.8 m (90th percentile, by
  scenario), and GodView averages the two cars' reports where Late Fusion
  keeps whichever box survives the NMS; this was not tested here.

Not shown: the cause of the across-box offset, and whether the same happens
on V2X-Real.
