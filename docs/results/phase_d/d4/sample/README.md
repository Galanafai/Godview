# D4 sample package (scene 2023-04-03-18-19-32_13_0, frames 100 to 239)

A small slice of the full package (kept outside the repository, 735 MB) so a renderer can be built. The detections that feed
GodView are dataset annotations with noise (labels plus noise), not a trained detector. Nothing here was tuned for the demo;
`ego_view` is GodView's real output for vehicle 1 (folder `1`, agent index 2).

## Frame of reference
All positions are in the **odom frame**: east, north, up, metres, equal to the dataset map frame minus `manifest.origin_map`
(`[43.749, -165.619, 2.876]`). Angles are degrees. Time: 10 Hz, `frame` is the dataset index, `t_s` counts from frame 100.
`lidar_pose_map` is the only value in the **map** frame (see calibration).

## Files
* `manifest.json`: scene, frame range, agent table (`index`, `folder`, `kind`: indices 0 and 1 are roadside units -2 and -1, 2 and 3 are vehicles 1 and 2), cameras per folder, point-file descriptions. It also describes the full package, which has more files than this sample.
* `frames/NNNNNN.json` for all 140 frames (7.3 MB):
  * `agents[]`: `index`, `folder`, `kind`, `lidar_pose_map` = `[x, y, z, roll, yaw, pitch]` (map frame, metres and degrees, pose of the lidar), `ego_odom` = `[x, y, z]` of the agent in the odom frame.
  * `labels[]`: dataset labels (truth): `gid` (string, the pedestrians are `43`, `58`, `30`, `46`), `pos` = centre `[x, y, z]`, `size` = `[length, width, height]` metres, `yaw_enu_deg`, `class_id` (1 vehicle, 2 pedestrian, 3 truck).
  * `ego_view[]`: vehicle 1's view, one entry per object: `id` ("owner agent:track", the owner is the agent whose track it is), `status` (`solid` when vehicle 1's own sensor is among `contributing_agents`, `ghost` when only other agents contribute), `x, y, z` (centre, odom), `cov_pos` (3x3 position covariance in m^2, odom axes), `vel` (m/s, odom), `p_exist`, `class_id`, `class_prob`, `box` (`l`, `w`, `h` metres, `yaw_deg`, `size_var`, `yaw_var`), `contributing_agents` (agent indices), `self_agent` (agent index if the object is an agent's own vehicle, else null), `members` (number of fused reports), `attributed_labels` (label gids the members were matched to by the evaluation; two or more means a merged object), `merged_labels` (true when two or more).
* `calibration.json`: `1/cam1` is per frame (`NNNNNN` -> `intrinsic` 3x3, `extrinsic_cam_to_lidar` 4x4, `lidar_pose_map`), because the vehicle moves; `-1/cam2` (roadside unit -1, camera 2) is static (`static`, `intrinsic`, `extrinsic_cam_to_lidar`, `lidar_pose_map`). Image size 1920 x 1080. Projection of an odom point `p`: `pm = p + origin_map`; `T = pose_matrix(lidar_pose_map)` (rotation from roll, yaw, pitch as in the OPV2V convention, see `scripts/phase_d/d4_scene_analysis.py` `rot` and `pose_matrix`); `pc = inv(extrinsic_cam_to_lidar) @ inv(T) @ [pm, 1]`, OpenCV axes (x right, y down, z forward); pixel `= K @ pc`, divided by depth.
* `cameras/1/cam1/NNNNNN.jpeg`: vehicle 1 front camera, every 10th frame (100, 110, ... 230).
* `points/static_odom.npy`: 363,597 x 3 float32, odom frame, static background from all four lidars (every 5th frame, points outside label boxes, voxel 0.2 m).

## Known limits
* Pedestrian 43: the label position in `labels` and the detections attributed to gid 43 disagree (see `docs/results/phase_d_d4_scene.md`, open item); do not trust 43's label box in the camera images.
* Merged objects: pedestrian neighbours are merged in the ego view for most frames (`merged_labels`).
* The yaw convention of `ego_view.box.yaw_deg` and `labels.yaw_enu_deg` is assumed equal; checked only on vehicle 1's own object (-92.57 against the lidar yaw -92.56).
* Roadside camera boxes are not to be rendered (project decision).
