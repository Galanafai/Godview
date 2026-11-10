"""Phase D4: search V2X-Real scenes for the demo moment, by the committed rule.

Rule (docs/PHASE_D_HANDOFF.md, "D4: the rule for the demo moment"): an object-vehicle pair
qualifies at a frame when the object (a shared-truth label, not a recording vehicle's body) is
within 50 m of the vehicle, its box centre projects inside the vehicle's front camera image
(cam1, 1920 x 1080, depth over 3 m), none of the vehicle's own lidar points lie in its box, and
the vehicle's ego view holds a world object attributed to that label whose members are only
other agents' reports while at least one other agent detected it this frame. An interval is a
maximal run of qualifying frames, bridging gaps of at most 2 frames in the camera and sharing
conditions (not in occlusion or range); it counts if, within 30 frames after it ends, the vehicle has
at least 5 own lidar points in the box in at least 3 of 10 consecutive frames while the object is
within 50 m; minimum 10 frames. Ranking: pedestrians and cyclists (Pedestrian, BicycleRider,
ScooterRider, Scooter) first, then longest, then scenario, vehicle, label id.

Inputs per scenario: the raw split directory (yaml, vehicle lidar `.bin`), the D2 converter's output
(truth.json) and the `v2x-replay` dump (ego views; self reports on with self_single_per_source,
alignment off). Writes all candidates as json.

Usage (python -I): d4_search.py OUT.json SCENARIO_NAME RAW_SCENARIO_DIR CONV_DIR DUMP.jsonl [...repeat]
"""
import json
import os
import sys

import numpy as np
import yaml

L = yaml.CSafeLoader
PREFERRED = {'Pedestrian', 'BicycleRider', 'ScooterRider', 'Scooter'}
MAX_RANGE, MIN_DEPTH, GAP, MIN_LEN, AFTER, WIN, WIN_NEED, VIS_PTS = 50.0, 3.0, 2, 10, 30, 10, 3, 5
W, H = 1920, 1080


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


def count_points(P, c, R, half):
    r2 = float(np.sum(np.square(half))) + 1e-6
    Q = P[np.sum(np.square(P - c), axis=1) <= r2]
    if not len(Q):
        return 0
    q = (Q - c) @ R
    return int(np.sum(np.all(np.abs(q) <= half, axis=1)))


def intervals(q, base):
    """Maximal runs of q, merged across gaps of at most GAP frames where `base` still holds."""
    n = len(q)
    idx = [i for i in range(n) if q[i]]
    runs = []
    for i in idx:
        if runs and i - runs[-1][1] - 1 <= GAP and all(base[k] for k in range(runs[-1][1] + 1, i)):
            runs[-1][1] = i
        else:
            runs.append([i, i])
    return [tuple(r) for r in runs]


def search(scen, raw, conv, dump):
    T = json.load(open(os.path.join(conv, 'truth.json')))
    agents = T['agents']
    frames = T['frames']
    kinds = {a['index']: a['kind'] for a in agents}
    folder = {a['index']: a['folder'] for a in agents}
    lines = {}
    for l in open(dump):
        if l.strip():
            e = json.loads(l)
            lines[e['frame']] = e
    origin = np.array(T['origin_map'])
    # object class names from any agent's yaml (first sight)
    names = {}
    ypose = {}
    ycam = {}
    for k, a in enumerate(agents):
        for i, fr in enumerate(frames):
            y = yaml.load(open(os.path.join(raw, a['folder'], fr['frame'] + '.yaml')), Loader=L)
            for gid, v in (y['vehicles'] or {}).items():
                names.setdefault(str(gid), v['obj_type'])
            if a['kind'] == 'vehicle':
                ypose[(k, i)] = pose_matrix(np.array(y['lidar_pose'], float))
                ycam[(k, i)] = (np.array(y['cam1']['extrinsic'], float), np.array(y['cam1']['intrinsic'], float))
    out = []
    for v, a in enumerate(agents):
        if a['kind'] != 'vehicle':
            continue
        n = len(frames)
        gids = sorted({o['gid'] for fr in frames for o in fr['objects'] if o['agent'] is None and o['class_id'] is not None})
        pts = {}
        info = {g: {'dist': [None] * n, 'proj': [False] * n, 'npts': [None] * n, 'shared': [False] * n, 'seen_by': [None] * n,
                    'uv': [None] * n} for g in gids}
        for i, fr in enumerate(frames):
            P = np.fromfile(os.path.join(raw, a['folder'], fr['frame'] + '.bin'), dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
            T_l = ypose[(v, i)]
            inv = np.linalg.inv(T_l)
            E, K = ycam[(v, i)]
            Einv = np.linalg.inv(E)
            ego_view = lines[i]['agents'][v]['ego'] if i in lines else []
            by_label = {}
            for o in ego_view:
                for g in o['labels']:
                    by_label.setdefault(g, []).append(o)
            for o in fr['objects']:
                g = o['gid']
                if g not in info:
                    continue
                pos = np.array(o['pos']) + origin
                d = float(np.hypot(*(pos[:2] - T_l[:2, 3])))
                info[g]['dist'][i] = d
                if d > MAX_RANGE:
                    continue
                cl = (inv @ np.r_[pos, 1.0])[:3]
                pc = (Einv @ np.r_[cl, 1.0])[:3]
                if pc[2] > MIN_DEPTH:
                    u, vv = K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2]
                    info[g]['proj'][i] = bool(0 <= u < W and 0 <= vv < H)
                    info[g]['uv'][i] = [float(u), float(vv)]
                half = np.array(o['size']) / 2.0
                R = inv[:3, :3] @ rot(0, o['yaw_enu_deg'], 0)
                info[g]['npts'][i] = count_points(P, cl, R, half)
                cands = by_label.get(g, [])
                only_others = all(v not in x['sources'] for x in cands)
                others_detected = [k for k in o['detected_by'] if k != v]
                info[g]['shared'][i] = bool(cands) and only_others and bool(others_detected)
                info[g]['seen_by'][i] = others_detected
        for g in gids:
            d = info[g]
            q = [d['dist'][i] is not None and d['dist'][i] <= MAX_RANGE and d['proj'][i] and d['npts'][i] == 0 and d['shared'][i] for i in range(n)]
            base = [d['dist'][i] is not None and d['dist'][i] <= MAX_RANGE and d['npts'][i] == 0 for i in range(n)]
            for s, e in intervals(q, base):
                length = e - s + 1
                if length < MIN_LEN:
                    continue
                # afterwards directly visible
                vis = False
                for w0 in range(e + 1, min(n, e + 1 + AFTER) - WIN + 1):
                    c = sum(1 for k in range(w0, w0 + WIN) if d['dist'][k] is not None and d['dist'][k] <= MAX_RANGE
                            and d['npts'][k] is not None and d['npts'][k] >= VIS_PTS)
                    if c >= WIN_NEED:
                        vis = True
                        break
                if not vis:
                    continue
                seen = {}
                for k in range(s, e + 1):
                    for a_ in (d['seen_by'][k] or []):
                        seen[folder[a_]] = seen.get(folder[a_], 0) + 1
                out.append({'scenario': scen, 'vehicle': a['folder'], 'vehicle_index': v, 'gid': g, 'class_name': names.get(g, '?'),
                            'preferred': names.get(g) in PREFERRED, 'start_frame': s, 'end_frame': e, 'length_frames': length,
                            'length_s': length / 10.0, 'start_file': frames[s]['frame'], 'end_file': frames[e]['frame'],
                            'seen_by_agents_frames': seen, 'range_m_start_end': [d['dist'][s], d['dist'][e]],
                            'uv_start': d['uv'][s], 'uv_end': d['uv'][e]})
    return out


def main():
    out_path = sys.argv[1]
    args = sys.argv[2:]
    allc = []
    for k in range(0, len(args), 4):
        scen, raw, conv, dump = args[k:k + 4]
        c = search(scen, raw, conv, dump)
        print(scen, len(c), 'candidates', flush=True)
        allc += c
    allc.sort(key=lambda c: (not c['preferred'], -c['length_frames'], c['scenario'], c['vehicle'], c['gid']))
    json.dump(allc, open(out_path, 'w'), indent=1)
    print(len(allc), 'candidates in total')


if __name__ == '__main__':
    main()
