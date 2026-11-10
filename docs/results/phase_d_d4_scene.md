# D4: the chosen moment, checks and scene package

Scene `2023-04-03-18-19-32_13_0`, vehicle 1 (folder `1`, agent index 2), pedestrians 43, 58, 30, 46 (ranks 1, 2, 4, 5).
The demo uses dataset annotations as detections (labels plus noise, sigma 0.3 m, 0.2 m, 3 deg), not a trained detector.
Scripts: `scripts/phase_d/d4_scene_analysis.py`, `d4_wm_detail.py`, `extract_scene_cameras.py`, `d4_export_package.py`, `d4_make_rrd.py`.
Raw numbers: `docs/results/phase_d/d4/scene/analysis.json`, `wrong_merges_detail.json`. CPU only for all of this, no GPU used.

## 1. Visibility in vehicle 1's front camera after the interval

Visible means the projected box centre is inside the image and at least 5 of vehicle 1's own lidar points are in the box.

| ped | hidden frames | first visible | delay after interval | own points then | visible until scene end |
|---|---|---|---|---|---|
| 58 | 125-153 (2.9 s) | 154 | 0.1 s | 9 | yes (85 frames) |
| 30 | 129-153 (2.5 s) | 154 | 0.1 s | 9 | yes (74 frames) |
| 46 | 154-177 (2.4 s) | 179 | 0.2 s | 6 | yes (61 frames) |
| 43 | 133-197 (6.5 s) | 198 | 0.1 s | 7 | no (12 frames, then out of range) |

Roadway: 58, 30 and 46 walk across the crosswalk ahead of the stopped vehicle (strip `strip_vehicle1_front.jpg`, 1.4 to 1.6 m/s); a swept-area proxy for the driven road never flags them (0 frames), so the visual strip is the evidence and the proxy is weak (it only knows where tracked vehicles drove). 43 (a scooter rider, 0.66 m/s median) is flagged on the proxy for 14 frames from 198, so it is the one that reaches the roadway; the strips alone do not settle this. In the 190 frame crop of unit -1 camera 2, the projected box of 43 sits below and left of the rider, so its label height or footprint does not fit the image well. Not resolved.

## 2. Roadside unit cameras (frames 110 to 239, share of pedestrian-frames with the projected box inside the image)

| camera | share inside | mean pedestrians per frame | median height px |
|---|---|---|---|
| vehicle 1 front | 89.4% | 3.5 | 130 |
| unit -1 cam2 | 95.1% | 3.7 | 85 |
| unit -1 cam1 | 61.4% | 2.4 | 88 |
| unit -2 cam1 | 40.9% | 1.6 | 150 |
| unit -2 cam2 | 7.7% | 0.3 | 143 |

Unit -1 camera 2 shows them best (all four, near the crosswalk, projections of 58, 30, 46 land on the people in the frame 190 crop). Unit -2 camera 1 is closer and larger but the group leaves the bottom of the image. Strips: `strip_unit_*.jpg`.

## 3. Wrong merges, frames 110 to 239 (an object whose members attribute to two or more labels)

* Agent-involving wrong merges: 0.
* Wrong merges on the four pedestrians: not clean. In vehicle 1's ego view the four are inside a merged object in 95 to 106 of 112 to 130 frames each. The pairs are 30 with 58 (87 to 99 frames per agent) and 46 with 47 (93 to 98), people walking side by side. None merged with a vehicle.
* Vehicles around vehicle 1: 24 wrong-merge events in its ego view (e.g. labels 0 and 9, 3 and 5), 158 over all views.
* Total ego-view wrong-merge object-frames: 2275 over four agents (574, 536, 583, 582).

So the requested verification does not pass for the pedestrians and nearby vehicles. This is the dense-pedestrian merging seen before, not a failure of the new self or ego mechanisms. Not diagnosed further: whether it is the association gate or the label spacing (30 and 58 centres are well under a metre apart) is not measured here. The package records `attributed_labels` and `merged_labels` per ego-view object so merged objects can be shown or avoided.

## 4. Package

Frames 100 to 239 (the last pedestrian is visible from 198, plus 100 frames exceeds the scene end, so the scene end applies). Location `/workspace/data/v2xreal/d4_package/2023-04-03-18-19-32_13_0/` (735 MB; not in the repository) and the Rerun recording `/workspace/data/v2xreal/d4_package/2023-04-03-18-19-32_13_0.rrd` (728 MB, 1.5 GB total).

* `manifest.json`, `calibration.json` (per frame for vehicle cameras, static for roadside units)
* `frames/NNNNNN.json`: all agents' lidar pose (map), odom position, declared pose, detections, detected labels; labels (truth); vehicle 1 ego view (`status` solid when vehicle 1 contributes, else ghost; id, covariance, velocity, class, box, contributing agents, attributed labels)
* `cameras/<folder>/<cam>/NNNNNN.jpeg`: vehicle 1 cams 1 to 4, vehicle 2 cam 1 only (the dataset carries cam1 for it in the extracted set), units -1 and -2 cams 1 and 2
* `points/static_odom.npy` (363,597 points, voxel 0.2 m), `points/moving_odom.npz` (about 33,000 points per frame, voxel 0.08 m)

Everything is in the odom frame (map minus `origin_map`). Not checked: the .rrd was written without opening a viewer, so appearance in the Rerun viewer is unverified.
