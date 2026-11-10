# Phase D1: second test run, plan (committed before the run)

**This is test run 2, separately labeled. Test run 1
(`docs/results/phase_d_d1_test.md`, the settings committed in
`docs/results/phase_d_d1_plan.md`) is kept as it is and is not replaced.**
Run 2 uses tracker settings chosen on val after run 1 had been seen (the
ablation, `docs/results/phase_d_d1_ablation.md`). Nothing about run 2 was
chosen on test.

## Declared runs, all alignment off

Data: V2V4Real test, 9 scenarios, 1,993 frames; detections from the step 1
export (SHA-256 `796590db...`, unchanged); replay input already converted by
`scripts/phase_d/convert_detections.py` with the committed settings (pose
1.27 / 2.0 m / 0.1 degrees, anisotropic covariance, 0.37 m size sigma, 7.5
degrees heading sigma); replay options common to all GodView runs:
`--box-size-sys-m 0.50 --box-yaw-sys-deg 5.7 --box-center-sys-frac 0.14
--box-center-sys-z-m 0.22`, confirmation after 3 hits, mean score, filtered
box, 100 ms delay.

| run | what | replay flags added |
|---|---|---|
| 1 (primary) | GodView, selected setting, sharing | `--fresh-only --overlap-iou 0.3 --overlap-keep recent --pub-min-score 0.3` |
| 2 | GodView, selected setting, no sharing | the same plus `--no-sharing` |
| 3 | GodView, selected setting with coasting tracks kept (the persistence trade-off), sharing | `--overlap-iou 0.3 --overlap-keep recent --pub-min-score 0.3` (no `--fresh-only`) |
| 4a | No Fusion, detections with score at least 0.2 (published) | none (car 0's export) |
| 4b | No Fusion, score at least 0.3 | none |
| 4c | Late Fusion, score at least 0.2 (published) | none (both cars, joint rotated NMS 0.15) |
| 4d | Late Fusion, score at least 0.3 | none |

0.3 is the threshold GodView's selected setting publishes at. The baselines'
0.3 versions filter the export's per-car detections before the joint NMS
(the export has score at least 0.2, post-NMS; filtering is equivalent to
running NMS at 0.3, because a box is only suppressed by a higher-scoring
one). Scores are exported rounded to five decimals.

Nothing else is run on test for this plan. If a run must be redone because of
a bug, the first result is kept and the new one is labeled separately.

## Metrics, for every run

1. **The dataset's AP** (AP@0.5, AP@0.7, car 0's lidar frame, the dataset's
   ground truth): as in run 1; true and false positives concatenated in frame
   order, not ranked by score across frames.
2. **A standard AP**: VOC 2010 all-point interpolation, detections of all
   frames pooled and ranked by score, highest-IoU ground truth taken once.
   `scripts/phase_d/ap_eval.py`; validated on val before this plan
   (`scripts/phase_d/validate_standard_ap.py`, `docs/results/phase_d/d1/ap_validation.json`):
   three hand-computed toy cases, scikit-learn's `average_precision_score`
   (step-wise area, rescaled to the ground-truth count) equal to 1e-9, and
   opencood's `voc_ap` on the same curve equal to 1e-9; on val the dataset
   AP of ap_eval equals the library's functions to 1e-9 for every method.
3. Both APs overall and in the ranges 0 to 30, 30 to 50 and 50 to 100 m; as
   a labeled second number each also with every label's first 3 frames
   ignored.
4. **GodView's own metrics** (`scripts/phase_d/aggregate_replay.py`): matched,
   missed, duplicates, false, RMS, NEES, size NEES, fused / both detect, ID
   changes, wrong merges, splits, coasting object-frames, world objects per
   car-frame, **objects gained and false objects introduced by sharing per
   car-frame**.
5. All of it for the 9 scenarios, and without `2022-03-15-09-54-40_0`.

## What the write-up must say

* The tracker settings were chosen on val among a few options, scored on the
  same frames; run 2 is the first test of them.
* The dataset's AP rewards fewer boxes: it does not rank by score across
  frames, so removing boxes (fresh-only, the publish threshold, and the
  baselines' 0.3 threshold) raises it. The standard AP is reported next to it
  for that reason, and the lead, or the lack of one, is stated under both
  definitions, against the baselines at 0.2 and at 0.3.
* Run 3 shows what keeping coasting tracks costs under each AP.
* Run 1's result (`docs/results/phase_d_d1_test.md`) stays the committed-settings result.
