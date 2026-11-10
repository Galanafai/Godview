# Phase D1 step 4: GodView on V2V4Real test with the real PointPillars detector

Run once on 2026-11-10, CPU only (no GPU time), with the settings fixed in
`docs/results/phase_d_d1_plan.md` (committed before this run; replay built
from the tree at the commit in `HEAD` when the run started, no setting changed
after seeing test). Test: 9 scenarios, 1,993 frames, detections from the
step 1 export (SHA-256 `796590db...`). Two cars each, so 3,986 car-frames.
Raw outputs: `docs/results/phase_d/d1/ap_test_{all,no09}.json`,
`godview_test_runs.json`, `godview_test_metrics.json`.

## Result in one paragraph

With the real detector, GodView with sharing and alignment off reaches AP@0.5
41.1 and AP@0.7 20.6 on car 0's frame. That is 1.3 AP@0.5 points above the
detector alone (No Fusion, 39.85) and 1.5 AP@0.7 points below it (22.05), and
well below the dataset's Late Fusion baseline (55.0 / 26.7). GodView does not
beat the baseline on this metric. Sharing helps GodView against its own
no-sharing run (32.5 / 17.1), mostly beyond 30 m. Alignment on is worse
(38.6 / 13.1). GodView without sharing is below the raw detector (32.5 /
17.1 against 39.85 / 22.05), so most of the gap to Late Fusion starts in the
per-car tracker, not in the sharing.

## AP (car 0's lidar frame, the dataset's evaluation)

AP@0.5 / AP@0.7 in percent. `AP_all` counts every frame. `AP_excl3` is the
labeled second number of the plan: each label's first 3 frames are ignored.
GodView's score is the world object's `class_prob`; with `p_exist` the
numbers agree to 0.01. The scorer reproduces the library's AP exactly (asserted
on every method) and step 1's test numbers: No Fusion 39.85 / 22.05, Late
Fusion 55.00 / 26.75 (D0: 39.81 / 22.02 and 55.07 / 26.63).

| method | AP_all | AP_excl3 |
|---|---|---|
| No Fusion (detector, car 0 alone) | 39.85 / 22.05 | 40.11 / 22.21 |
| Late Fusion (both cars, joint NMS) | 55.00 / 26.75 | 55.05 / 26.75 |
| GodView, no sharing | 32.48 / 17.05 | 33.13 / 17.42 |
| GodView, sharing, alignment off | 41.10 / 20.59 | 41.75 / 20.93 |
| GodView, sharing, alignment on | 38.60 / 13.06 | 39.19 / 13.27 |

By range (AP_all):

| method | 0 to 30 m | 30 to 50 m | 50 to 100 m |
|---|---|---|---|
| No Fusion | 69.2 / 42.7 | 29.3 / 14.4 | 4.8 / 1.6 |
| Late Fusion | 73.2 / 36.9 | 44.0 / 22.4 | 36.3 / 17.3 |
| GodView, no sharing | 62.1 / 37.1 | 23.8 / 11.1 | 4.3 / 1.3 |
| GodView, alignment off | 62.3 / 36.6 | 33.0 / 15.4 | 23.7 / 10.4 |
| GodView, alignment on | 59.7 / 25.3 | 30.5 / 9.0 | 20.7 / 5.5 |

Without `2022-03-15-09-54-40_0` (the scenario also present in `train_01`;
1,846 frames): No Fusion 40.30 / 22.16, Late Fusion 55.64 / 26.82, GodView
alignment off 41.60 / 20.65, on 39.11 / 13.25, no sharing 32.90 / 17.18. The
ordering is the same.

What the numbers support and what they do not:

* The start-up effect of confirmed tracks is small: ignoring each label's
  first 3 frames adds 0.6 to 0.7 AP@0.5 points to GodView and 0.3 to No Fusion.
  It does not explain the gap to the raw detector (7 AP@0.5 points without
  sharing).
* What does explain it is not tested. Candidates, none measured here: the
  tracker needs 3 detections to confirm and drops flickering low-score
  detections; fused boxes are smoothed and their size follows the track's
  estimate; GodView publishes about 30 percent more boxes than Late Fusion
  (53,074 against 42,536), so more false positives share the ranking (the score
  only ranks within a frame in this AP).
* Near range (0 to 30 m) GodView is 7 AP@0.5 points under the detector alone
  in both runs, so the cost there is the tracker, and sharing adds nothing.
  Beyond 30 m sharing adds 9 to 19 AP@0.5 points to GodView, which is the
  expected effect of a second view.
* AP@0.7 drops with alignment on at every range (0 to 30 m: 36.6 to 25.3),
  as predicted from val. Alignment moves car 1's reports about 0.6 m. The
  dataset's truth does not apply that correction. Whether the alignment is
  right and the truth wrong in that frame is not decided here.
* 102 of 31,419 ground-truth boxes (0.3 percent) could not be tied to a
  label's identity and are never ignored in the second number.
* The truth is the labels, which have their own errors; the detector most
  likely trained on `2022-03-15-09-54-40_0`.

## GodView's own metrics (both cars, 9 scenarios, 3,986 car-frames)

| | no sharing | sharing, alignment off | sharing, alignment on |
|---|---|---|---|
| matched | 45,131 | 53,351 | 53,346 |
| missed | 8,527 | 307 | 312 |
| duplicates (split across cars) | 2,285 (0) | 4,598 (2,868) | 4,568 (2,868) |
| false | 29,201 | 48,224 | 48,249 |
| RMS, all matched (m) | 0.86 | 0.69 | 0.76 |
| RMS, objects labeled by both cars (m) | 0.77 | 0.74 | 0.75 |
| NEES 3D, objects labeled by both (expect 3) | 0.29 | 0.27 | 0.27 |
| box size NEES (expect 3) | 1.52 | 1.47 | 1.46 |
| both detect: fused into one | 0 / 23,064 | 22,355 / 23,160 | 22,359 / 23,161 |
| ID changes per object | 0.57 | 1.02 | 1.16 |
| wrong merges (object-frames) | 0 | 2,154 | 2,198 |
| splits (label-frames) | 1,394 | 3,327 | 3,358 |

"Expected" objects (the replay's missed and matched counts) are those a
detection was attributed to in this and the 3 previous frames, in either car.
Missed is therefore small when sharing: the other car's detection puts the
object in the car's world.

**Objects gained and false objects introduced by sharing, per car-frame:**

| | no sharing | alignment off | alignment on |
|---|---|---|---|
| objects gained (matched, never detected by the car's own sensor, object-frames) | 0.93 | 2.83 | 2.83 |
| distinct objects gained | 94 | 194 | 194 |
| false objects introduced (unmatched, more than 5 m from every label, only source the other car) | 0 | 4.81 | 4.82 |

Sharing brings about 1.9 gained objects and 4.8 false objects per car-frame.
The "false" category counts everything farther than 5 m from every labeled
object, so it includes real objects the annotators did not label; it is an
upper bound on false positives. The other car's false detections also arrive
as new objects, and without sharing each car already holds about 7.3 false
objects per frame (29,201 / 3,986) in its own world.

Without `2022-03-15-09-54-40_0`: gained 2.98, false from sharing 4.79 (alignment
off); all other rows move by 1 to 4 percent
(`godview_test_metrics.json`).

Caveats specific to these counts: two scenarios (`2022-03-17-10-50-46_1`,
`2022-03-17-11-02-23_1`) have a car 1 label linked to several car 0 labels; it
is attributed to the lowest id, which turns some correct merges into
wrong merges. The no-sharing GodView run is a reference, not a baseline of the
dataset.

## Not shown

* Why the single-car tracker costs AP (no ablation of confirmation, box
  smoothing or score handling was run).
* Whether alignment on is right in the physical frame.
* Any run with a different setting: settings were fixed before this run, and
  test ran once.
