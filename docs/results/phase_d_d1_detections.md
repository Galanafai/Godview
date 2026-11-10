# Phase D1 step 1: per-car detections from the late-fusion baseline's detector

Run on 2026-11-10 on the RunPod A40, same environment and checkpoint as D0
(`docs/results/phase_d_d0.md`).

## What was exported

`scripts/phase_d/export_detections.py` runs the published PointPillars
checkpoint (the one the late-fusion baseline uses) on each car of each frame
and writes, per frame and car:

* `box`: `x y z l w h yaw` in that car's own lidar frame (centre in m, full
  length along the heading, width and height in m, yaw in rad, not wrapped);
* `corners` (8 x 3, same frame) and `score` (sigmoid of the class output);
* `lidar_pose` (4x4, from the frame's YAML) and `T_to_ego`, the transform the
  baseline uses to project that car into the ego car's frame;
* per frame, the dataset's ground truth as the evaluation builds it
  (`gt_corners_ego`), so the dataset's AP can be recomputed without the loader.

Post-processing per car is the baseline's own: **score threshold 0.2** (from
the checkpoint config), rotated NMS at IoU 0.15, then the evaluation range
mask (x within 100 m, y within 40 m). The only change from the late-fusion
baseline is that NMS runs per car in its own frame rather than once over both
cars' projected boxes. Seeds are fixed (0); rerunning val gave identical
counts and AP.

| split | scenarios | frames | detections car 0 / car 1 | GPU time |
|---|---|---|---|---|
| train1 without the duplicate (see below) | 3 | 1,806 | 9,788 / 11,669 | 3 min 25 s |
| val | 3 | 748 | 7,480 / 8,934 | 1 min 58 s |
| test (test1 to test3) | 9 | 1,993 | 29,843 / 33,201 | 5 min 41 s |

The exports (gzipped JSON lines, 4.5 to 16.6 MB) are in
`/workspace/data/v2v4real/d1_detections/`, not in the repository. SHA-256:
train1_dedup `b74eb40b...b334d`, val `ad096734...72fd`, test
`796590db...6fe7` (full hashes in the commit message).

## Checks

* **Decoding matches the library.** On the first 50 frames of every split
  the script's own decoding was compared with the library's `post_process`
  (identity transform): 0 mismatches.
* **The export reproduces the baselines on test.** AP recomputed from the
  exports with the dataset's `eval_utils`:

  | | AP@0.5 export / D0 | AP@0.7 export / D0 |
  |---|---|---|
  | No Fusion (ego car's export) | 39.85 / 39.81 | 22.05 / 22.02 |
  | Late Fusion (both exports projected, joint NMS) | 55.00 / 55.07 | 26.74 / 26.63 |

  The differences are at most 0.11 AP point. The cause is not established;
  candidates are the random point shuffle before voxelization (D0 ran
  unseeded, the export uses seed 0) and, for Late Fusion, NMS per car before
  the joint NMS.
* **Boxes are where the labels are.** On every 10th frame, the share of a
  car's own labels with a detection within 2 m is 0.79 (val), 0.84 (train1),
  0.85 (test); median distance to the nearest detection 0.21 to 0.29 m. Median
  size 4.5 x 2.0 x 1.7 m. (`scripts/phase_d/check_export.py`)

## Findings that matter for the next steps

1. **A test scenario is also in the train download.** `train_01.zip` contains
   `testoutput_CAV_data_2022-03-15-09-54-40_0`, byte-identical to the test1
   scenario of the same name (all 588 files, both cars: 147 frames, 7.4 % of
   the 1,993 test frames). If the checkpoint was trained on `train_01` as
   distributed, it saw these test frames. This cannot be checked from here.
   It applies to the D0 numbers and to the published README numbers as much
   as to anything GodView will report. The scenario is excluded from the
   train part of the error model. Whether `train_02` to `train_08` overlap
   with test is not known (not downloaded).
2. **The detector does better on train than on val or test**, as expected for
   data it was trained on: No Fusion AP@0.7 is 34.8 on train1 against 25.7 on
   val and 22.0 on test (AP@0.5: 41.2, 46.0, 39.9). An error model fitted on
   train would understate the errors on unseen data. The plan for step 2 is
   to fit on val and use train only as a cross-check.
3. **Heading is ambiguous by 180 degrees by construction.** This PointPillars
   head has two anchors (0 and 90 degrees) and no direction classifier;
   exported yaw spans -1.05 to 2.60 rad. Step 2 measures the flip rate, and
   the replay should declare `yaw_ambiguous: true` unless that measurement
   says otherwise.
4. **The dataset's evaluation uses car 0 as the only ego.** The loader takes
   the first car folder as ego in every frame, and both baselines are scored
   against both cars' labels (deduplicated by object ID) in car 0's frame,
   within x of 100 m and y of 40 m. "AP in each ego car's frame" (D1 step 4)
   therefore means car 0's frame unless a second, separately labeled
   evaluation with car 1 as ego is added.
5. **Box column order.** The network regresses `x y z h w l yaw` (config
   order `hwl`), but `box_utils.boxes_to_corners_3d` reorders columns 3 to 5
   of its input in place, so the boxes leaving the library's post-processing
   are `l w h`. Checked against the corners on all val detections.

## Safety of inputs

* All zips were listed before unpacking: no absolute paths, `..` or
  symlinks (`scripts/phase_d/check_zip.py`).
* The checkpoint pickle references only `OrderedDict` and torch tensor
  rebuilds (`scripts/phase_d/check_pickle_globals.py`).
* The data YAML files across train1, val and test hold only numpy tags
  (`ndarray`, `scalar`, `dtype`, `_reconstruct`) plus `tuple` and `binary`.
  The repository reads them with `yaml.Loader`; the export reads them with
  the safe `V2VLoader` from `scripts/v2v4real/v2v4real_io.py` instead.

## GPU time

About 0.25 GPU hours for this step, including one stopped run (fixed a field
name) and the repeated val export. D0 and D1 so far: about 0.4 GPU hours.

## Not shown

* Nothing here involves GodView yet.
* The test sanity check above reads test labels only to confirm the box
  format; no setting is chosen from it.
