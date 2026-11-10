"""Phase D2 step 2: convert one V2X-Real scenario into godview_node input for every agent.

Defaults by rule (docs/PHASE_D_HANDOFF.md, "D2 steps 2 and 3"): fixed made-up Earth anchor
(34.0689 N, 118.4452 W, height 0, map x axis East); pose sigmas per frame, vehicles 1.51 m
horizontal / 0.28 m vertical / 1.8 degrees, roadside units 0.1 / 0.1 / 0.1 degrees
(assumed); classes by the repository's mapping (vehicle 1, pedestrian 2, truck 3).

Detections are labels plus noise (as Phase C): an agent detects a label of its own list when
at least 5 of its own lidar points are inside the box and the box is within 100 m of the
agent; the detection is the label plus seeded Gaussian noise (position 0.3 m, size 0.2 m,
heading 3 degrees), declared as such. Labels are in the exact shared map frame, so no pose
error is injected.

Writes OUT_DIR/agent<k>.jsonl (k = index of the agent in `agents` of truth.json),
truth.json (the union of the agents' labels by id) and attribution.json (the label id of
each detection).

Usage (python -I): convert_v2xreal.py SCENARIO_DIR OUT_DIR [--seed 1]
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml

L = yaml.CSafeLoader
LAT0, LON0, H0 = 34.0689, -118.4452, 0.0
CLASS = {}
for cid, names in ((1, ['LongVehicle', 'Car', 'PoliceCar']),
                   (2, ['Child', 'RoadWorker', 'Pedestrian', 'Scooter', 'ScooterRider', 'Motorcycle', 'MotorcyleRider', 'BicycleRider']),
                   (3, ['Truck', 'Van', 'TrashCan', 'ConcreteTruck', 'Bus'])):
    for n in names:
        CLASS[n] = cid
SIGMA = {'vehicle': (1.51, 0.28, 1.8), 'rsu': (0.1, 0.1, 0.1)}
FOOT_OFFSET = (0.60, 0.12)       # vehicle footprint centre ahead of / left of the lidar (m)
MIN_POINTS, MAX_RANGE = 5, 100.0
DT_NS = 100_000_000
T0_NS = 1_700_000_000_000_000_000
POS_SIGMA, SIZE_SIGMA, YAW_SIGMA = 0.3, 0.2, 3.0


def rot(roll, yaw, pitch):
    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    cr, sr = np.cos(np.radians(roll)), np.sin(np.radians(roll))
    cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    return np.array([[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                     [sp, -cp * sr, cp * cr]])


def pose_matrix(p):
    m = np.eye(4)
    m[:3, :3] = rot(p[3], p[4], p[5])
    m[:3, 3] = p[:3]
    return m


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def points_inside(P, loc, angle, extent, T_inv):
    """Number of lidar points (agent frame) inside a map-frame label box."""
    c = (T_inv @ np.r_[loc, 1.0])[:3]
    R = T_inv[:3, :3] @ rot(*angle)
    # Only points that can be inside (within the box's circumscribed sphere) are transformed.
    r2 = float(np.sum(np.square(extent))) + 1e-6
    P = P[np.sum(np.square(P - c), axis=1) <= r2]
    if not len(P):
        return 0
    q = (P - c) @ R
    return int(np.sum(np.all(np.abs(q) <= np.asarray(extent), axis=1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    sc = a.scenario_dir.rstrip('/')
    name = os.path.basename(sc)
    agents = sorted(os.listdir(sc), key=int)
    kinds = ['vehicle' if int(x) > 0 else 'rsu' for x in agents]
    frames = sorted({n.split('.')[0].split('_')[0] for n in os.listdir(os.path.join(sc, agents[0]))})
    Y = [[yaml.load(open(os.path.join(sc, x, f + '.yaml')), Loader=L) for x in agents] for f in frames]
    T = [[pose_matrix(np.array(y['lidar_pose'], float)) for y in yy] for yy in Y]
    origin = np.mean([t[:3, 3] for tt in T for t in tt], axis=0)
    en = origin[:2]
    lat_o = LAT0 + np.degrees(en[1] / 6378137.0)
    lon_o = LON0 + np.degrees(en[0] / (6378137.0 * np.cos(np.radians(LAT0))))
    h_o = H0 + origin[2]
    rng = np.random.default_rng([a.seed] + [ord(ch) for ch in name])
    os.makedirs(a.out_dir, exist_ok=True)
    lines = [[] for _ in agents]
    attribution, truth_frames = [], []
    n_det = [0] * len(agents)
    for i, f in enumerate(frames):
        t_ns = T0_NS + i * DT_NS
        objs = {}
        det_by = {}
        attr_f = []
        for k, x in enumerate(agents):
            P = np.fromfile(os.path.join(sc, x, f + '.bin'), dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
            Tinv = np.linalg.inv(T[i][k])
            pos_lidar = T[i][k][:3, 3]
            dets, gids = [], []
            for lid, v in (Y[i][k]['vehicles'] or {}).items():
                cid = CLASS.get(v['obj_type'])
                loc = np.array(v['location'], float)
                size = 2.0 * np.array(v['extent'], float)
                yaw = float(v['angle'][1])
                o = objs.setdefault(str(lid), {'gid': str(lid), 'locs': [], 'sizes': [], 'yaw': yaw, 'class_id': cid,
                                              'labeled_by': [], 'detected_by': []})
                o['locs'].append(loc)
                o['sizes'].append(size)
                o['labeled_by'].append(k)
                # Noise is drawn for every label of this agent so the stream does not depend on visibility.
                n = rng.normal(size=3) * POS_SIGMA
                ns = rng.normal(size=3) * SIZE_SIGMA
                ny = rng.normal() * YAW_SIGMA
                if cid is None or np.linalg.norm((loc - pos_lidar)[:2]) > MAX_RANGE:
                    continue
                if points_inside(P, loc, v['angle'], v['extent'], Tinv) < MIN_POINTS:
                    continue
                p = loc - origin + n
                dets.append({'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]), 'sigma_m': POS_SIGMA,
                             'class_id': cid, 'class_prob': 0.9,
                             'box': {'length_m': float(max(size[0] + ns[0], 0.1)), 'width_m': float(max(size[1] + ns[1], 0.1)),
                                     'height_m': float(max(size[2] + ns[2], 0.1)), 'yaw_deg': float(wrap(yaw + ny)),
                                     'sigma_size_m': SIZE_SIGMA, 'sigma_yaw_deg': YAW_SIGMA, 'yaw_ambiguous': False}})
                gids.append(str(lid))
                o['detected_by'].append(k)
            sh, sv, sy = SIGMA[kinds[k]]
            pose = {'lat_deg': float(lat_o), 'lon_deg': float(lon_o), 'height_m': float(h_o), 'yaw_deg': 0.0,
                    'sigma_h_m': sh, 'sigma_v_m': sv, 'sigma_yaw_deg': sy}
            yaw_l = float(wrap(np.degrees(np.arctan2(T[i][k][1, 0], T[i][k][0, 0]))))
            lines[k].append(json.dumps({'t_unix_ns': t_ns, 'pose': pose, 'detections': dets,
                                        'agent_xyz': (pos_lidar - origin).tolist(), 'agent_yaw_deg': yaw_l,
                                        'agent_kind': kinds[k]}))
            attr_f.append(gids)
            n_det[k] += len(dets)
        attribution.append(attr_f)
        out_objs = []
        for gid, o in sorted(objs.items()):
            out_objs.append({'gid': gid, 'pos': (np.mean(o['locs'], axis=0) - origin).tolist(),
                             'size': np.mean(o['sizes'], axis=0).tolist(), 'yaw_enu_deg': float(wrap(o['yaw'])),
                             'class_id': o['class_id'], 'labeled_by': o['labeled_by'], 'detected_by': o['detected_by'],
                             'agent': None})
        # Which label is the body of a vehicle agent: the nearest label of a vehicle class within 2.5 m
        # of its footprint centre (lidar position plus the footprint offset).
        for k, kind in enumerate(kinds):
            if kind != 'vehicle':
                continue
            yaw_r = np.radians(np.degrees(np.arctan2(T[i][k][1, 0], T[i][k][0, 0])))
            centre = T[i][k][:2, 3] + np.array([np.cos(yaw_r) * FOOT_OFFSET[0] - np.sin(yaw_r) * FOOT_OFFSET[1],
                                                np.sin(yaw_r) * FOOT_OFFSET[0] + np.cos(yaw_r) * FOOT_OFFSET[1]])
            best = None
            for o in out_objs:
                if o['class_id'] == 1:
                    d = np.linalg.norm(np.array(o['pos'][:2]) + origin[:2] - centre)
                    if d < 2.5 and (best is None or d < best[0]):
                        best = (d, o)
            if best:
                best[1]['agent'] = k
        truth_frames.append({'frame': f, 't_ns': t_ns, 'ego_odom': [(t[:3, 3] - origin).tolist() for t in T[i]], 'objects': out_objs})
    for k in range(len(agents)):
        with open(os.path.join(a.out_dir, 'agent%d.jsonl' % k), 'w') as fh:
            fh.write('\n'.join(lines[k]) + '\n')
    with open(os.path.join(a.out_dir, 'attribution.json'), 'w') as fh:
        json.dump({'frames': attribution}, fh)
    truth = {'scenario': name, 'anchor': {'lat_deg': float(lat_o), 'lon_deg': float(lon_o), 'height_m': float(h_o), 'yaw_deg': 0.0},
             'made_up_anchor': {'lat_deg': LAT0, 'lon_deg': LON0, 'height_m': H0}, 'origin_map': origin.tolist(), 'dt_ns': DT_NS,
             'agents': [{'folder': x, 'kind': kinds[k], 'index': k} for k, x in enumerate(agents)],
             'declared': {'pose_sigma_vehicle': SIGMA['vehicle'], 'pose_sigma_rsu': SIGMA['rsu'], 'det_sigma_m': POS_SIGMA,
                          'size_sigma_m': SIZE_SIGMA, 'yaw_sigma_deg': YAW_SIGMA, 'min_points': MIN_POINTS, 'max_range_m': MAX_RANGE,
                          'footprint_offset_forward_left_m': FOOT_OFFSET, 'seed': a.seed},
             'frames': truth_frames}
    with open(os.path.join(a.out_dir, 'truth.json'), 'w') as fh:
        json.dump(truth, fh)
    print('%s: %d frames, agents %s, detections %s' % (name, len(frames), agents, n_det))


if __name__ == '__main__':
    main()
