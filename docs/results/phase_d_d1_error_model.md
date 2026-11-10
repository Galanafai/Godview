# Phase D1 step 2: error model of the PointPillars detector

Measured on 2026-11-10 from the step 1 exports
(`docs/results/phase_d_d1_detections.md`), on **val** (3 scenarios, 748
frames) to choose settings and on **train1** (3 scenarios, 1,806 frames,
without the scenario duplicated in test) as a cross-check. Nothing here reads
test. Script: `scripts/phase_d/error_model.py`; full outputs:
`docs/results/phase_d/d1/error_model_{val,train1_dedup}.json`. CPU only.

## Method

* Each car's detections are matched to that car's own labels, both in the
  car's lidar frame, inside the dataset's evaluation range (x within 100 m,
  y within 40 m), by Hungarian assignment on centre distance with a 3 m gate.
* Error = detection minus label. **The labels are the truth here, and they
  have their own errors** (Phase C found the two cars' annotators disagree on
  sizes by up to about 1 m).
* Each error is split per object (one label ID of one car in one scenario,
  objects with at least 5 matched frames: 151 on val, 74 on train1) into a
  persistent part, the object's mean error, and a per-scan part, the
  deviation from that mean. GodView averages per-scan noise in its tracks but
  not persistent errors, so the persistent part must be declared as
  systematic (`box_center_sys_frac`, `box_size_sys_m`, `box_yaw_sys_rad`) and
  the per-scan part as the detection's own sigma. The systematic standard
  deviation is corrected for the noise left in each object's mean.
* Lidar points per label are counted in the car's own point cloud.

## Position

Total error per horizontal axis (RMS), and its per-scan part along and
across the labeled box:

| range (m) | val n | val total | val per-scan along / across / z | train1 total | train1 per-scan along / across / z |
|---|---|---|---|---|---|
| 0 to 10 | 883 | 0.14 | 0.16 / 0.07 / 0.05 | 0.15 | 0.21 / 0.09 / 0.06 |
| 10 to 20 | 1,875 | 0.27 | 0.31 / 0.12 / 0.08 | 0.16 | 0.20 / 0.10 / 0.07 |
| 20 to 30 | 2,404 | 0.26 | 0.27 / 0.13 / 0.09 | 0.16 | 0.17 / 0.09 / 0.08 |
| 30 to 40 | 2,639 | 0.32 | 0.34 / 0.17 / 0.10 | 0.19 | 0.19 / 0.11 / 0.12 |
| 40 to 50 | 2,229 | 0.32 | 0.31 / 0.18 / 0.13 | 0.23 | 0.22 / 0.13 / 0.17 |
| 50 to 60 | 1,954 | 0.33 | 0.32 / 0.19 / 0.13 | 0.28 | 0.30 / 0.15 / 0.25 |
| 60 to 80 | 1,256 | 0.37 | 0.35 / 0.24 / 0.18 | 0.40 | 0.44 / 0.24 / 0.26 |

* Maximum-likelihood fit of the per-scan horizontal noise per axis,
  `sigma(r) = sqrt(a^2 + (b r)^2)`: val a = 0.200 m, b = 0.0036; train1
  a = 0.125 m, b = 0.0037. At 50 m: 0.27 m (val), 0.22 m (train1).
* The error is about twice as large along the box as across it (val per-scan
  fits: 0.27 against 0.09 m at short range). GodView's input takes one
  isotropic `sigma_m`, so this anisotropy cannot be declared.
* Persistent part per object (val): along 0.33 m, across 0.17 m standard
  deviation; as a fraction of the half-length 0.087 and of the half-width
  0.136 (train1: 0.062 and 0.103). The simulator's `large_objects` uses 0.25,
  which was not measured.
* Height: detections sit 0.11 m low on average on val (0.04 m on train1),
  with a persistent part of 0.19 m. GodView's systematic centre error covers
  length and width only.
* Gate sensitivity (val): RMS per horizontal axis 0.27 m with a 2 m gate
  (13,153 matches), 0.30 m with 3 m (13,240), 0.32 m with 4 m (13,268). A
  few large errors move the RMS by about 10 percent.

## Size

| | val per-scan | val persistent | val mean | train1 per-scan | train1 persistent | train1 mean |
|---|---|---|---|---|---|---|
| length (m), all | 0.37 | 0.97 | -0.37 | 0.48 | 1.29 | -0.22 |
| width (m), all | 0.10 | 0.33 | -0.24 | 0.08 | 0.19 | -0.02 |
| height (m), all | 0.12 | 0.28 | -0.10 | 0.12 | 0.23 | -0.05 |
| length (m), cars only | 0.30 | 0.50 | -0.22 | 0.23 | 0.32 | -0.06 |
| width (m), cars only | 0.09 | 0.29 | -0.22 | 0.08 | 0.12 | 0.00 |
| height (m), cars only | 0.11 | 0.18 | -0.06 | 0.10 | 0.17 | -0.03 |

* The large persistent length error comes from the few large vehicles (7
  objects on val, 9 on train1). The anchors are car-sized (3.9 x 1.6 x
  1.56 m): on val the 186 concrete truck matches are 3.5 m too short on
  average (label length 13.2 m).
* Val car widths come out 0.23 m narrower than the labels, train1 car widths
  0.01 m. Whether the val labels are wider or the detector does worse there
  is not separable from this data.

## Heading

* **Front and back are not known to the detector.** The network has no
  direction classifier and its yaw lies in one 180 degree window (val:
  -40 to 127 degrees). When the label's heading lies inside that window, 0.5
  percent of matches differ by more than 90 degrees; outside it, 97 percent.
  So the 25 percent "flip rate" on val is this window, not random flips:
  every detection should be declared `yaw_ambiguous: true`.
* Error modulo 180 degrees: val RMS 8.5, median 3.0, 95th percentile 17.3
  degrees (train1: 5.9, 2.8, 11.3). Per-scan 7.5 degrees, persistent 5.7
  degrees (train1: 5.3, 2.9).

## Misses and false positives

* Of the labels with at least 5 of the car's own lidar points, 16.0 percent
  are missed on val (12.8 on train1). Miss rate by range (val): 0 below
  10 m, 6 to 9 percent from 10 to 60 m, 37 percent from 60 to 80 m, and 100
  percent beyond 80 m: the detector's own range is x within 70.4 m
  (`cav_lidar_range`), while the evaluation counts labels to 100 m. 52 percent
  of all val misses lie beyond x = 70.4 m.
* By points (val): 88 percent missed with 1 to 4 points, 58 percent with 5
  to 19, 13 percent with 20 to 49, 5 to 7 percent above 50, but 11 percent
  above 300 points; those are concrete trucks (306 of 331).
* False positives (no label of that car within 3 m): 2.1 per car and frame on
  val, 1.7 on train1. Only 6 to 9 percent lie within 4 m of something the
  other car labeled, so very few are objects one car's annotator left out.
* Scores are not calibrated probabilities. Share of detections that match a
  label, by score (val / train1): 0.2 to 0.3: 0.33 / 0.12; 0.3 to 0.4: 0.71 /
  0.41; 0.4 to 0.5: 0.89 / 0.77; 0.5 to 0.6: 0.96 / 0.94; above 0.6: 0.99 or
  more on both.

## Proposed replay settings (from val)

To be committed with the evaluation plan in step 3, before any test run:

| GodView input | value | from |
|---|---|---|
| score threshold | 0.2 | the baseline's own, unchanged |
| `sigma_m` per detection | `sqrt(0.20^2 + (0.0036 r)^2)` m, r the detection's range from its car | val per-scan horizontal fit |
| `box_center_sys_frac` | 0.14 | larger of the val persistent along (0.087) and across (0.136) fractions |
| `sigma_size_m` per detection | 0.37 m | largest val per-scan size error (length) |
| `box_size_sys_m` | **open, see below** | |
| `sigma_yaw_deg` per detection | 7.5 | val per-scan, modulo 180 |
| `box_yaw_sys_deg` | 5.7 | val persistent, modulo 180 |
| `yaw_ambiguous` | true for every detection | the 180 degree window |
| `class_prob` | the detector score | so that the existence score used for AP (maximum of members' scores) is the raw score |
| pose sigmas | Phase C frozen values (1.27 m, 2.0 m, 0.1 degrees) | unchanged |

`box_size_sys_m` is one value for all three dimensions and all objects. The
val measurement gives 0.97 m over all objects (driven by 7 large vehicles)
and 0.50 m over cars only. 0.97 m keeps the declaration true for trucks but
loosens the size gate for every car; 0.50 m understates the trucks, which
may then be split between the two cars. This choice is left to Galanafai.

The detector's mean biases (0.11 m low, boxes 0.2 to 0.4 m short and
narrow, trucks 3.5 m short) are not corrected; the replay uses the detector
as it is.

## Not shown

* How these settings behave in GodView: that is step 4, on test only.
* The labels' own error: the split into detector error and annotation error
  is not possible without independent truth.
* Train1 numbers are on data the detector was most likely trained on, and
  they are better than val in nearly every row, as expected.
