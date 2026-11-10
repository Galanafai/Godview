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
