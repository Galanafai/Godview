# Ego view and offset filter on V2V4Real val

2026-11-10. V2V4Real val only (3 scenarios, 748 frames, 1,496 car-frames), selected
tracker, sharing, each car's ego view; AP is car 0's. CPU only (the six variants, AP and
coverage take about 4 minutes), no GPU. Test was not touched. Expectations and the
filter's parameter rule were committed before the filter existed (`04bb4f1`,
`docs/results/phase_d_d1_plan.md`). Raw outputs: `docs/results/phase_d/d1/ego_val_runs.json`,
`ego_val_filter_vs_label_offset.txt`; scripts `scripts/phase_d/ego_val.sh`,
`selfreport_coverage.py`, `filter_vs_label_offset.py`.

## What was implemented (commits `e41047c`, `91f1d98`)

* **Ego view** (`NodeConfig::ego_view`, `PerceptionNode::ego_world`): the view of
  agent A is a function of the received reports, A's own sensor-only covariances
  (recorded when it publishes) and A's current pose. A's own reports carry sensor
  covariance only; every other source's reports gain the pose-induced covariance of A's
  pose at the object; alignment holds A's translation at zero (only A's connected
  group is offset); self objects use the same fusion. The shared world and the
  published bytes are identical with the flag on (tested).
* **Offset filter** (`NodeConfig::offset_filter`, `alignment::OffsetBank`): per other agent
  a random-walk translation (q), measured by shared objects (any number per frame, gated
  against the prediction, one to one) and agent-agent links; parameters from the rule
  committed before use (q 0.023 / 0.066 m^2/s, sigma_obj 0.37 / 0.57 m, sigma_link 0.33 m,
  gate 16.27, max age 10 s).
* Tests: 168 core tests (ego view equals the shared world with zero pose sigmas; the flag
  changes nothing in bytes or the shared world; an own precise detection pulls a self
  object to (6.05, 0.02) in the ego view while the shared world keeps the sender's
  (7.0, 0.4); the filter converges with one shared object per frame, follows a 0.15 m/s
  drift, rejects decoys and a far-off link, converges from links alone, drops a stale
  estimate), node tests, DST seeds 1 to 3 (all 60 runs).

## Expectations set before the runs, and the outcome

| | expectation | result |
|---|---|---|
| (i) | ego view with self reports on: recording-car box coverage at least 90% | **73.2% (fails)** |
| (ii) | filtered alignment loses at most 1 AP@0.7 point against alignment off (same self setting) | **-20.4 points without self reports (68.4 / 42.9 to 43.2 / 22.5), -11.5 with (61.1 / 35.1 to 47.3 / 23.5) (fails)** |
| (iii) | an estimate in at least 90% of car-frames that share any object | **98.8% (no self reports), 99.2% (self reports) (holds)** |

Two of three fail, so per the rule set in advance I stopped here; D2 step 2 was not started.

## The six variants (selected tracker, sharing, ego views)

Dataset AP and standard AP are AP@0.5 / AP@0.7 on car 0's ego view; coverage is the share
of ground-truth boxes with a world box at IoU 0.5; the last two columns are the share of
car-frames with an offset estimate (all car-frames / those sharing an object) and the
same run's shared world (dataset AP, recording-car coverage), for reference.

| variant (self reports, alignment) | dataset AP | standard AP | wrong merges | splits | missed | recording-car coverage | other-object coverage | offset estimate | shared world (AP; coverage) |
|---|---|---|---|---|---|---|---|---|---|
| off, off | 68.4 / 42.9 | 72.5 / 50.3 | 202 | 52 | 676 | 96.3% | 72.0% | 0% | 67.2 / 42.1; 96.2% |
| off, per-frame | 54.3 / 30.5 | 58.5 / 36.8 | 183 | 14 | 680 | 79.0% | 66.4% | 62.4% / 62.4% | 56.0 / 16.0; 88.0% |
| off, filtered | 43.2 / 22.5 | 46.4 / 27.4 | 217 | 17 | 863 | 65.6% | 60.9% | 98.8% / 98.8% | 67.2 / 42.1; 96.2% |
| on, off | 61.1 / 35.1 | 63.0 / 39.4 | 225 | 83 | 646 | 73.2% | 72.0% | 0% | 57.3 / 28.6; 64.7% |
| on, per-frame | 52.0 / 28.6 | 55.8 / 34.0 | 198 | 45 | 645 | 73.2% | 66.4% | 62.4% / 62.4% | 48.6 / 11.3; 64.4% |
| on, filtered | 47.3 / 23.5 | 51.5 / 29.2 | 215 | 71 | 640 | 73.4% | 63.5% | 99.2% / 99.2% | 57.3 / 28.6; 64.7% |

(Self reports "on" are with both refinements, the fused self position and the V2V4Real
footprint of the rule: 4.73 x 2.25 x 2.34 m, centre 0.10 m ahead and 0.17 m left of the
lidar. The shared world of a "filtered" run does not use the filter, so it equals the
"alignment off" shared world.) Every car-frame shares an object by the attributed
detections (1,496 of 1,496), so the two estimate fractions coincide for the filter.

* The ego view alone helps: off/off is 68.4 / 42.9 against the shared world's
  67.2 / 42.1 (+1.2 / +0.9), and the recording-car coverage stays 96.3%.
* With self reports the ego view's coverage is 73.2% against 64.7% for the same run's
  shared world and 59.0% for the earlier fused shared-world result.
* The filter has an estimate in 99% of car-frames against 62% for the per-frame estimate.

## Why (i) and (ii) fail: V2V4Real's truth places each object by one car's pose

**Explanation (agreed with Galanafai, who specified the expectations):** V2V4Real's
ground truth places each object by one car's pose (each label is in its annotating
car's lidar frame, and the dataset builds its truth in car 0's frame from the cars'
`lidar_pose`s). Correcting a pose error therefore moves a detection away from truth that
contains that error, and the dataset's AP penalizes it. Expectations (i) and (ii) were
badly specified on that account; they ask the metric to reward something it cannot see.
The evidence is the coverage split below and the filter's agreement with the
cross-car label offset.

Coverage split by whose box it is and which car labeled it
(`coverage_split.json`; the first four rows are the ego view):

| ground-truth boxes | n | off, off | on, off | off, filtered | on, filtered |
|---|---|---|---|---|---|
| the other recording car (car 1) | 772 | 98.7% | **98.7%** | 98.7% | 99.4% |
| the ego (car 0), the other car's label of it | 768 | 93.9% | **47.5%** | 32.3% | 47.4% |
| other objects labeled by both cars | 5,290 | 75.1% | 75.1% | 75.5% | 77.0% |
| other objects labeled by car 0 only | 853 | 72.1% | 72.1% | 72.1% | 72.1% |
| other objects labeled by car 1 only | 1,764 | 62.8% | 62.8% | **11.5%** | **19.1%** |

* **(i):** the ego view does what was expected for the other car: with self reports its
  box is covered 98.7% of the time (82.0% in the shared world). The 73.2% is half of
  that and half the ego's own box (47.5%). The ground truth for the ego car is the
  other car's label of it placed in car 0's frame by the other car's pose; the ego's
  self object sits at its own position in its own frame, and the two poses disagree by
  about a metre. Without self reports the world contains that other car's detection of
  the ego, which is the truth by construction (93.9%).
* **(ii):** the filter moves car 1's reports into car 0's frame. Objects only car 1
  labeled have their ground truth in car 1's (erroneous) frame, so correcting the frame
  moves the detections away from the truth: coverage falls from 62.8% to 11.5%
  (19.1% with self reports). The same applies to the ego's box (32.3% without self
  reports, where it is car 1's detection of the ego that is moved).
* **The filter itself is accurate against the one independent reference we have**, the
  cross-car label offset (median over objects both cars labeled, per frame;
  `ego_val_filter_vs_label_offset.txt`): with agent-agent links, median error 0.28 m
  on a median offset of 1.37 m, 90th percentile 0.96 m, 71% of frames within 0.5 m, with an
  estimate in 684 of the 690 frames that have a reference. The per-frame estimate:
  0.29 m, 75% within 0.5 m, but only 460 of 690 frames. The filter without links
  (shared objects alone): 0.36 m median, 90th percentile 2.76 m, 60% within 0.5 m (it
  initialises from an ambiguous consensus more often).

## Not shown

* That the filter improves anything the dataset's AP can measure: it cannot, on a
  dataset whose truth for half of the objects is in a frame the filter corrects. A
  physical test (the filter's translation against the true relative pose) would need
  an independent pose reference, which V2V4Real does not have.
* The label offset is itself made of annotator noise; the filter's error against it
  is an upper bound on its error only if the labels are unbiased.
* Whether V2X-Real behaves differently: its labels share one map frame across agents
  (0.29 m median disagreement), so a pose-frame error of this kind should not exist in
  its truth. Not tested.
* Expectations (i) and (ii) failed as stated; (iii) held. The failures are explained by the
  truth being placed by one car's pose (above), not by a measured defect of the ego view
  or the filter: the coverage split (the other car 98.7%, the ego's own box 47.5%, car-1-only
  objects 62.8% falling to 11.5% under the filter) and the filter's 0.28 m median agreement
  with the cross-car label offset are the evidence. The numbers alone do not exclude a
  defect that they cannot see; the next step scores each car against its own labels only.
