# D4: demo candidates (rule applied 2026-11-10)

**The demo uses dataset annotations as detections.** The "detections" are V2X-Real's own labels plus
seeded noise (as in D2), not the output of a detector; what the demo shows is GodView's sharing and ego view
on real footage and real geometry, not detector quality. This statement goes on the video's end card and in
every write-up.

The rule is the one committed before the search (`docs/PHASE_D_HANDOFF.md`, "D4: the rule for the demo
moment", commit `5e2b094`): an object within 50 m of a vehicle, its box centre in the vehicle's front camera image,
none of that vehicle's own lidar points in its box, held in that vehicle's **ego view** only through other agents'
reports (labels plus noise, self reports on with `self_single_per_source`, alignment off, the D2 tracker), as a run of
at least 10 frames (gaps of at most 2 frames in the camera and sharing conditions bridged), after which the object
becomes directly visible (at least 5 own lidar points in at least 3 of 10 consecutive frames within 30 frames).
Ranking: pedestrians and cyclists first, then longest. Script `scripts/phase_d/d4_search.py`; all candidates:
`docs/results/phase_d/d4/candidates.json`. CPU only (conversion, replay and search of 14 test and 6 val
scenarios took about 15 minutes).

**Search space:** 20 scenes (6 val, 14 test), 2 vehicles each (one scene has the two vehicles 256 m apart). **35
candidates** (30 from test, 5 from val); 28 are pedestrians, 7 are cars; no cyclist qualified. They come from 10 scenes
(most in `2023-04-04-14-30-53_47_0`: 10, `2023-04-03-18-19-32_13_0`: 6, `2023-04-04-14-27-53_44_0` and
`2023-04-07-15-05-15_4_1`: 5 each).

## Top 5

Frame numbers are file names; 10 frames per second. Thumbnails: the vehicle's front camera at the start and the end of
the interval, the object's projected centre ringed in red (`docs/results/phase_d/d4/rank<k>.jpg`).

| rank | scene (test) and vehicle | object | length | agents that saw it (frames of the interval) | range at start / end | what the thumbnails show |
|---|---|---|---|---|---|---|
| 1 | `2023-04-03-18-19-32_13_0`, vehicle 1 | pedestrian, label 43 | **65 frames (6.5 s)**, 133 to 197 | unit -2 (65), unit -1 (65), vehicle 2 (65) | 22 m / 21 m | The vehicle is stopped at a red light behind a car; the pedestrian is hidden behind the white utility truck on its left (ring on the truck's side compartment). At the end a person is visible near the crosswalk ahead (not verified to be the same person) |
| 2 | same scene and vehicle | pedestrian, label 58 | 29 frames (2.9 s), 125 to 153 | vehicle 2 (29) | 16 m / 15 m | Behind the same truck, closer, toward its front |
| 3 | `2023-04-04-14-27-53_44_0`, vehicle 1 | pedestrian, label 168 | 28 frames (2.8 s), 71 to 98 | vehicle 2 (28) | 46 m / 37 m | Far ahead among parked cars on a street with a bike lane; small in the image |
| 4 | `2023-04-03-18-19-32_13_0`, vehicle 1 | pedestrian, label 30 | 25 frames (2.5 s), 129 to 153 | units -2 and -1 (25 each) | 16 m / 15 m | Behind the same truck |
| 5 | same scene and vehicle | pedestrian, label 46 | 24 frames (2.4 s), 154 to 177 | units -2 (24), -1 (24), vehicle 2 (24) | 17 m / 17 m | Behind the same truck |

## Things to know before picking

* **Ranks 1, 2, 4 and 5 are one moment**: the same scene and vehicle, overlapping in time (frames 125 to 197), four
  different pedestrians behind the same truck. They could be shown together (several ghosts at once) rather than as
  alternatives; rank 3 is a different scene.
* **Rank 1 is the clearest by the rule**: the longest interval (6.5 s) and the most sources (both roadside units and the
  other vehicle). In that scene the ego vehicle is nearly stationary (stopped at the light), which makes the camera
  overlay easy and the 3D flythrough static; rank 3 has the vehicle moving.
* **Sources are labels plus noise**: "seen by" is which agents' lidar covered the label with at least 5 points (the
  converter's detection rule), not a detector output. A pedestrian behind a truck seen by a roadside unit is real geometry
  from the data; that the unit would have detected it is an assumption of the labels-plus-noise input.
* **Occlusion means zero of the vehicle's own lidar points in the label's box**, a strict test; the camera may still show
  part of the object in some frames (the ring only marks the box centre).
* **Not checked by eye:** that the rank-1 pedestrian is the one who emerges at the end; the frames between the start and
  the end; the other 30 candidates (only the top 5 were rendered).

## Pick (to be recorded)

The pick and its rank are made by Galanafai with the team and will be recorded here. The scene package (the D4 list
in the handoff: poses, detections, the vehicle's ego view per frame with who contributed, labels, the vehicle's front
camera images and calibration, a static point cloud and per-frame moving points) and a Rerun `.rrd` follow the pick.
