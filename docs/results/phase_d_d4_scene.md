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

Mechanism (recorded as the finding of record, decision: keep the scene as is, no fixes, render GodView's real output): the merges are association swaps between neighbouring pedestrians. Same-source exclusivity holds (agent-involving merges 0) and the swaps are caused by wide cross-agent gates.

So the requested verification does not pass for the pedestrians and nearby vehicles. This is the dense-pedestrian merging seen before, not a failure of the new self or ego mechanisms. Not diagnosed further: whether it is the association gate or the label spacing (30 and 58 centres are well under a metre apart) is not measured here. The package records `attributed_labels` and `merged_labels` per ego-view object so merged objects can be shown or avoided.

## 3b. Ghosts near hidden pedestrians in vehicle 1's ego view (report only, `ghost_check.json`)

Distance from each person's label centre (horizontal) to the nearest ghost object, over the frames of that person's hidden interval.

| ped | frames | median m | max m | frames with no ghost within 1 m | stretches over 1 s with none within 1 m |
|---|---|---|---|---|---|
| 58 | 125-153 | 0.14 | 0.40 | 0 | none |
| 30 | 129-153 | 0.17 | 0.40 | 0 | none |
| 46 | 154-177 | 0.21 | 0.52 | 0 | none |
| 43 | 133-197 | 4.80 | 12.83 | 65 of 65 | 133-197 (flagged) |

Ghosts within 2 m of any hidden person against the number of hidden people (73 frames with at least one hidden): mean 0.84 against 1.96; fewer ghosts than people in 67 frames, more in 1. The shortfall is the 58 and 30 merge (one object for two people, they hide together) plus 43, which has no ghost at all. No object of any kind in vehicle 1's ego view lies within 3.6 m of 43's label while it is hidden.

Cause for 43 (checked, not fixed): the detections attributed to gid 43 in the agents' views sit about 12 m from the label position in `truth.json` (frame 140: attributed objects near (15.2, 11.3), label at (3.3, 3.5); frame 197: objects at (9.7, 11.6) and (-20, -12), label at (-0.4, 3.3)). Three agents attribute detections to 43 in all 65 hidden frames, so it is not an invisibility: the identity of gid 43 in the converter output does not describe one place. This is open. It probably also explains item 3.

## 3c. Open: pedestrian 43 offset in unit -1 camera 2

In the frame 190 crop the projected label box of 43 sits below and left of the scooter rider (58, 30 and 46 fit). Unexplained; consistent with the label position problem above but not shown to be the same cause. Roadside-camera boxes stay off in the render. The first-visible frame and own-point counts for 43 in section 1 were computed from the label position, so they inherit this doubt.

## 4. Package

Frames 100 to 239 (the last pedestrian is visible from 198, plus 100 frames exceeds the scene end, so the scene end applies). Location `/workspace/data/v2xreal/d4_package/2023-04-03-18-19-32_13_0/` (735 MB; not in the repository) and the Rerun recording `/workspace/data/v2xreal/d4_package/2023-04-03-18-19-32_13_0.rrd` (728 MB, 1.5 GB total).

* `manifest.json`, `calibration.json` (per frame for vehicle cameras, static for roadside units)
* `frames/NNNNNN.json`: all agents' lidar pose (map), odom position, declared pose, detections, detected labels; labels (truth); vehicle 1 ego view (`status` solid when vehicle 1 contributes, else ghost; id, covariance, velocity, class, box, contributing agents, attributed labels)
* `cameras/<folder>/<cam>/NNNNNN.jpeg`: vehicle 1 cams 1 to 4, vehicle 2 cam 1 only (the dataset carries cam1 for it in the extracted set), units -1 and -2 cams 1 and 2
* `points/static_odom.npy` (363,597 points, voxel 0.2 m), `points/moving_odom.npz` (about 33,000 points per frame, voxel 0.08 m)

Everything is in the odom frame (map minus `origin_map`). Not checked: the .rrd was written without opening a viewer, so appearance in the Rerun viewer is unverified.
