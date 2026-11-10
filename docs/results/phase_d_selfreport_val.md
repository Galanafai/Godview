# Self reports in GodView core, validated on V2V4Real val only

2026-11-10, CPU only. V2V4Real val (3 scenarios, 748 frames, 1,496 car-frames),
the PointPillars export of step 1, sharing, alignment off. **Test was not
rerun**; the real test of this feature is V2X-Real. The feature is off by default.
Raw outputs: `docs/results/phase_d/d1/selfrep_val_runs.json`,
`selfrep_val_coverage.json`, `selfrep_offset_vs_alignment_val.json`; scripts
`scripts/phase_d/selfreport_val.sh`, `selfreport_coverage.py`,
`selfreport_offset_vs_alignment.py`.

## Caveat that governs every number below

On V2V4Real val, with self reports on and the self object's position fused from
the self report and its assigned detections, only **59 percent** of the two
recording cars' ground-truth boxes are covered (IoU 0.5), against 97 percent
without self reports. The cause is structural, not a tuning matter: a report that
one agent shares carries that agent's own pose uncertainty (the pose sigma is added
to its covariance when the report is placed on the Earth). Fusion in the common
frame therefore cannot prefer the observer's detection, which is precise
relative to the observer, over the sender's report, which is only as good as the
sender's pose. The fused self object sits a median 0.34 of the way from the
self report to the other car's detection because the two carry comparable
covariances (see "Update" at the end). The same mechanism explains the AP@0.7 loss
under alignment (the shifted reports leave the frame the dataset's truth uses). The
fix proposed in `docs/PHASE_D_HANDOFF.md` (ego view: each agent scores its own
objects with sensor covariance only and the others' reports with their covariance
plus its own pose covariance, with its own translation held at zero in alignment)
is a separate step; the numbers in this file are for the shared world.

## What was implemented

* **Self report.** With `NodeConfig::self_reports`, an agent publishes its own
  position and footprint box (`SelfState`: centre, velocity, heading, size and
  their standard deviations, in its odometry frame) as an ordinary report with
  the reserved track id `SELF_TRACK` (`u64::MAX`, never issued by a tracker). The
  wire format is unchanged. It is placed on the Earth exactly like a detection, so
  it carries the agent's pose uncertainty. Nothing is published without a self
  state or a global pose.
* **World model** (`WorldModelConfig::self_reports`, set from the node config).
  Each self report is its own world object (`WorldObject::self_agent`, id =
  the self key) and keeps the self report's state. Any other report whose centre
  lies within `self_gate_sigma` (default 2) combined standard deviations of a
  self footprint (position covariances added, read along the box, across it and
  vertically), with the same class and a box that passes the box gate, is assigned
  to the nearest such agent: listed as a member for provenance, never fused into it
  and never clustered with another object, and left out of alignment.
* **Two optional refinements, off by default** (added after the first val run
  showed neighbours being taken, below): `self_same_source_exact` (for a report
  from the agent itself the pose error is common to both reports, so only the
  unshared covariance counts) and `self_compete` (a report of another source is not
  assigned if the agent's own detections contain one that passes the world gates
  against it at a smaller normalized distance than the report lies from the
  footprint: the agent cannot see itself). With both off the rule is exactly the
  one specified.
* **Tests** (159 in `godview_core`, all pass; DST seeds 1 to 3: all 60 runs pass,
  flag off): assignment of a detection on the footprint while a neighbour, a
  pedestrian-sized box and another class stay separate; the gate scales with the
  uncertainty; snapshots do not depend on arrival order; each refinement; the self
  report is published only when on, with a state and a global pose, and decodes as
  the footprint; and two agents that see each other, with a 1.5 m pose uncertainty,
  merge their two detections into one object without self reports (2 objects) and
  do not with them (the 2 agents and the common car, 3 objects).

## Validation on val

AP@0.5 / AP@0.7 (dataset's AP; standard AP in the json), car 0's frame:

| tracker, self reports | dataset AP | standard AP | wrong merges | splits | missed | matched | reports assigned to a self object (to the recording car's label / to another label) |
|---|---|---|---|---|---|---|---|
| committed, off | 50.23 / 29.80 | 67.58 / 46.20 | 251 | 1,837 | 78 | 15,887 | n/a |
| committed, literal rule | 36.36 / 16.99 | 48.48 / 23.70 | 768 | 1,845 | 404 | 15,561 | 4,144 (3,266 / 821) |
| committed, both refinements | 38.38 / 18.19 | 50.72 / 25.32 | 243 | 1,741 | 74 | 15,891 | 3,183 (3,071 / 81) |
| selected, off | 67.17 / 42.06 | 70.72 / 48.98 | 202 | 52 | 669 | 15,296 | n/a |
| selected, literal rule | 49.11 / 24.57 | 50.87 / 25.37 | 696 | 186 | 972 | 14,993 | 3,776 (3,086 / 685) |
| selected, same-source only | 51.03 / 25.72 | 52.84 / 26.48 | 626 | 484 | 639 | 15,326 | 3,333 (2,988 / 342) |
| selected, compete only | 49.88 / 24.54 | 51.67 / 25.50 | 646 | 485 | 671 | 15,294 | 3,377 (2,989 / 383) |
| selected, both refinements | 51.46 / 26.05 | 53.23 / 26.76 | 227 | 85 | 639 | 15,326 | 2,934 (2,891 / 40) |

With the flag off the AP and counts equal the earlier val numbers exactly.
False objects from sharing (2.89 committed, 0.44 selected per car-frame) and
objects gained (1.46, 1.36) do not change; objects per car-frame change by under 2
percent (committed 21.20 off, 20.85 literal). 2,986 self objects (both cars, 1,496
car-frames); `selfrep_val_runs.json` has every count.

### What this shows

* **The literal rule takes real neighbours.** 18 to 20 percent of the reports
  assigned to a self object are attributed to a label other than the recording
  car's (821 of 4,144; 685 of 3,776): the combined uncertainty is wide (pose sigma
  1.27 m per car) and the gate reaches the cars next to the agent. In the committed
  run 495 of the assigned reports came from the agent itself, and 377 of those (76
  percent, 46 percent of all 821 wrong assignments) are its own detections of
  neighbours, where the pose error is common and should not widen the test. The
  wrong assignments raise wrong merges (251 to 768; a self object that holds two
  labels counts as one) and misses (78 to 404).
* **The refinements remove that cost.** With both, matched and missed return
  to the baseline (15,891 and 74 against 15,887 and 78) and wrong assignments fall
  to 81 of 3,183 (2.5 percent) and 40 of 2,934 (1.4 percent). Wrong merges are at
  the baseline (243 against 251 committed; 227 against 202 selected). Neither
  refinement alone is enough.
* **Val cannot show the benefit.** Only 77 of the 251 baseline wrong merges
  on val involve a label within 4 m of a recording car (the dominant case on test,
  where the cars see each other closely), so there was little to remove;
  the refined rule leaves 243. Whether it removes the test-style merges (1,116 of
  2,010 on test run 2 contained the scored car itself) is for V2X-Real.
* **AP falls with self reports on val, mostly for a reason that is not about
  association.** The dataset's ground truth for a recording car is the other car's
  label of it, placed in car 0's frame by the two `lidar_pose`s. Without self
  reports the world holds exactly that detection (96 to 97 percent of the 1,540
  recording-car boxes are covered at IoU 0.5). A self report is the car's own
  position in its own pose frame, and the two cars' pose frames disagree: the
  nearest world box to a recording car's ground-truth box is 1.1 m away at the
  median (90th percentile 2.2 m), against 0.19 m without, and coverage of those
  boxes falls to 44 to 47 percent. The dataset's evaluation therefore scores a
  physically sensible self report as a miss and a false positive whenever the poses
  disagree. All other objects: coverage 74.7 percent without self reports, 72.6 literal
  (committed tracker); 70.9 without, 68.9 literal, 70.9 with both refinements
  (selected tracker).
  (`selfrep_val_coverage.json`.)

## Measurement only: a car's detection of the other car against the other car's self position

Not used by anything. For each car `c` and frame, the detection of the other
car `k` (attributed to the label of the recording car `k`) minus `k`'s own
position, in the shared map frame, against the translation alignment estimates
(replay with alignment on, committed tracker, car 0's world,
`-(t_0 - t_1)`), over frames where all three exist (535 of 748). The offset is
the difference of the two cars' pose errors plus detection noise and the
lidar-to-box-centre offset (0.6 m on V2X-Real; not measured on V2V4Real).

| scenario | frames | median offset (car 0 sees 1 / car 1 sees 0) | median alignment | corr East / North (offset vs alignment) | median distance between the car-averaged offset and the alignment |
|---|---|---|---|---|---|
| `15-20-01_0` | 219 | 1.87 / 1.45 m | 1.50 m | 0.80 / 0.26 | 0.33 m |
| `21-09-35-07_0` | 114 | 0.41 / 0.18 m | 0.59 m | -0.23 / -0.43 | 0.59 m |
| `21-09-35-07_10` | 202 | 1.02 / 0.82 m | 1.55 m | -0.14 / -0.06 | 1.06 m |
| pooled | 535 | 1.17 m | 1.45 m | 0.72 / 0.70 | 0.48 m |

* **Where the pose offset is large, the offset tracks the alignment estimate**
  (`15-20-01_0`: East correlation 0.80, and the average of the two cars' views is
  0.33 m from the alignment's estimate of a 1.5 m offset). **Where it is small
  (under 1 m) there is no relation**: the correlations are near zero or negative
  and the distance between them (0.6 to 1.1 m) is as large as the offsets, which are
  then comparable to detection noise and the mount offset.
* The pooled correlations (0.72 and 0.70) are mostly the difference between the
  large-offset scenario and the others, not agreement within scenarios; they should
  not be quoted as the result. North is weaker than East even in the large-offset
  scenario (0.26). Alignment is available in 538 of 748 frames only.

## Not shown

* That self reports remove the wrong merges that matter: val has few of them
  (77 involve a recording car). V2X-Real is the test.
* Anything about AP in the dataset's frame being improved by self reports: it is
  lower on val, for the evaluation-frame reason above, and would be on any dataset
  whose truth for a recording car comes from another agent's pose frame. Whether
  V2X-Real's labels (in one map frame, 0.29 m median disagreement between agents)
  behave differently is not known.
* The footprint on V2V4Real: a nominal 4.5 x 2.0 x 1.7 m (the median exported
  detection size on val) centred on the lidar position was used; the real offset
  of about 0.7 m was left inside the gate, not corrected.
* The refinements were added after seeing the first val result, and they are
  evaluated on the same frames (they are not tuned: no parameter was changed).

## Update: the self object's position fused from the self report and its assigned reports

Design change (2026-11-10, as asked): a self object's identity still comes from the
self report, but its state is now fused from the self report and the reports
assigned to it, like any cluster (covariance intersection in key order); the no-merge
rule is unchanged. It is behind the same flag (`self_reports`), so the numbers above
(position from the self report alone) are the behaviour of the earlier commit
`a17708f`. A unit test checks that a precise detection pulls the position while the
id, `self_agent`, membership and the neighbour's separation hold. Tests: 160 core
tests, node tests, DST seeds 1 to 3 (all 60 runs) pass.

**The two refinements (same-source exactness, competition) were added after the first
val run** (the one that showed the literal rule taking neighbours); they are on in
the runs below and are the choice for V2X-Real.

V2V4Real val, both trackers, sharing, alignment off, both refinements on
(`selfrep_val_fused_runs.json`; script `scripts/phase_d/selfreport_val_fused.sh`):

| tracker | dataset AP | standard AP | wrong merges | missed | recording-car boxes covered (IoU 0.5) | nearest world box to those boxes, median / p90 |
|---|---|---|---|---|---|---|
| committed, off | 50.23 / 29.80 | 67.58 / 46.20 | 251 | 78 | 97.3% | 0.19 / 0.47 m |
| committed, self position from the self report alone | 38.38 / 18.19 | 50.72 / 25.32 | 243 | 74 | 43.7% (literal rule; refined 46.9% for the selected tracker) | 1.06 / 1.94 m (selected) |
| **committed, self position fused** | 41.88 / 19.44 | 54.15 / 28.78 | 243 | 74 | **59.0%** | 0.96 / 1.58 m |
| selected, off | 67.17 / 42.06 | 70.72 / 48.98 | 202 | 669 | 96.2% | 0.19 / 0.49 m |
| selected, self position from the self report alone | 51.46 / 26.05 | 53.23 / 26.76 | 227 | 639 | 46.9% | 1.06 / 1.94 m |
| **selected, self position fused** | 55.54 / 27.90 | 56.84 / 30.16 | 227 | 639 | **59.0%** | 0.96 / 1.58 m |

Coverage of all other objects is unchanged (74.6% and 70.9%, equal to off). Wrong
assignments stay at 81 of 3,183 and 40 of 2,934. Wrong merges, misses and matched
are the same as with the self-report-only position (the fusion changes positions, not
membership).

**The expectation (coverage back toward 97%) is not met.** Fusing raises coverage
of the recording cars' boxes from 44 to 47 percent to 59 percent and AP by 3.5 to 4.1
points, but not most of the way back. The reason is measured: the fused position lies
a median 0.34 of the way (10th to 90th percentile 0.04 to 0.56) from the self report
to the other car's detection of it (1,236 frames with the detection more than 0.5 m
from the self position). The two have comparable covariances: both carry the declared
pose sigma (1.27 m per car), and the detection's own noise (0.1 to 0.3 m) is not
enough to make it dominate the self report's 0.1 m, so covariance intersection
weights them about equally. The truth for a recording car is the other car's label
in car 0's pose frame, which a covariance-weighted fusion cannot know to prefer.
