"""Phase D1: convert one scenario of the PointPillars export into replay input.

Writes, like scripts/v2v4real/convert.py (Phase C), car0.jsonl and car1.jsonl
(godview_node input) and truth.json (the labels, built the same way as in
Phase C), but the detections are the detector's own boxes, and each
detection is attributed to a label:

* Each car's detections are matched to that car's own labels, both in the
  car's lidar frame, by Hungarian assignment on horizontal centre distance
  with a --gate (3 m) cutoff, the matching of the error model
  (scripts/phase_d/error_model.py), here against all of the car's labels.
  The matched label's global id (car 0 label k is "0:k"; a car 1 label linked
  by ass_id is the car 0 id, otherwise "1:j") is written per detection to
  attribution.json. An unmatched detection has null.
* An object's detected_by (used by the replay's "expected" rule) is the set
  of cars with a detection matched to one of its labels in that frame.

Per detection, in the odometry frame:
* position: box centre (x, y, z) through lidar_pose;
* covariance: along / across the detection's box and vertical,
  sigma_k(r) = sqrt(a_k^2 + (b_k r)^2), r the detection's horizontal range
  from its own lidar (--along, --across, --vert give a,b);
* box: length, width, height, heading through lidar_pose, --sigma-size,
  --sigma-yaw-deg, yaw_ambiguous true;
* class_id 1, class_prob = the detector score.

The pose is the same as Phase C: --sigma-h, --sigma-v, --sigma-yaw-deg.

Usage (python -I, needs numpy, scipy, PyYAML):
  python -I convert_detections.py EXPORT.jsonl.gz SCENARIO_DIR OUT_DIR \
      --sigma-h 1.27 --sigma-v 2.0 --sigma-yaw-deg 0.1
"""
import argparse
import gzip
import json
import os
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'v2v4real'))
sys.path.insert(0, HERE)
from gps_fit import fit_map_to_enu, yaw_deg  # noqa: E402
from v2v4real_io import load_yaml  # noqa: E402
from error_model import read_pcd_xyz  # noqa: E402

DT_NS = 100_000_000
T0_NS = 1_700_000_000_000_000_000
CAV_MATCH_M = 4.0


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def points_in_box(P, loc, yaw_d, half):
    a = np.radians(yaw_d)
    c, s = np.cos(a), np.sin(a)
    q = P - np.asarray(loc)
    x = c * q[:, 0] + s * q[:, 1]
    y = -s * q[:, 0] + c * q[:, 1]
    return int(np.sum((np.abs(x) <= half[0]) & (np.abs(y) <= half[1]) & (np.abs(q[:, 2]) <= half[2])))


def sig(ab, r):
    return float(np.sqrt(ab[0] ** 2 + (ab[1] * r) ** 2))


def match(dets, labs, gate):
    """Hungarian on horizontal centre distance; returns {det index: label index}."""
    if not len(dets) or not len(labs):
        return {}
    L = np.array([l[0] for l in labs])
    D = np.linalg.norm(dets[:, None, :2] - L[None, :, :2], axis=2)
    cost = np.where(D <= gate, D, 1e6)
    r, k = linear_sum_assignment(cost)
    return {int(i): int(j) for i, j in zip(r, k) if cost[i, j] < 1e6}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('export')
    ap.add_argument('scenario_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--sigma-h', type=float, required=True)
    ap.add_argument('--sigma-v', type=float, required=True)
    ap.add_argument('--sigma-yaw-deg', type=float, required=True)
    ap.add_argument('--along', type=float, nargs=2, required=True, metavar=('A', 'B'))
    ap.add_argument('--across', type=float, nargs=2, required=True, metavar=('A', 'B'))
    ap.add_argument('--vert', type=float, nargs=2, required=True, metavar=('A', 'B'))
    ap.add_argument('--sigma-size', type=float, required=True)
    ap.add_argument('--det-sigma-yaw-deg', type=float, required=True)
    ap.add_argument('--gate', type=float, default=3.0)
    args = ap.parse_args()

    sc = args.scenario_dir.rstrip('/')
    name = os.path.basename(sc)
    recs = [json.loads(l) for l in gzip.open(args.export, 'rt')]
    recs = sorted((r for r in recs if r['scenario'] == name), key=lambda r: r['timestamp'])
    assert recs, 'no export records for %s' % name
    frames = [r['timestamp'] for r in recs]
    yaml_frames = sorted(os.path.basename(p)[:-5] for p in os.listdir(sc + '/0') if p.endswith('.yaml'))
    Y = [[load_yaml(f'{sc}/{c}/{f}.yaml') for c in (0, 1)] for f in frames]
    T = [[np.array(y['lidar_pose'], dtype=float) for y in yy] for yy in Y]
    for r, tt in zip(recs, T):
        ids = [c['cav_id'] for c in r['cars']]
        assert ids == ['0', '1'], ids
        for c, t in zip(r['cars'], tt):
            assert np.allclose(np.array(c['lidar_pose']), t, atol=1e-4), 'export pose differs from yaml'

    if all('gps' in y for yy in Y for y in yy):
        fit = fit_map_to_enu([t for tt in T for t in tt], [y['gps'] for yy in Y for y in yy])
    else:
        # As Phase C: val has no gps; a made-up anchor shared by both cars and the truth.
        fit = {'rot_deg': 0.0, 'lat0': 39.98, 'lon0': -83.02, 'h0': 0.0, 'rms_h': None,
               'max_h': None, 'rms_v': None, 'heading_offset': lambda k: None, 'no_gps': True}
    rot = np.radians(fit['rot_deg'])
    R = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
    origin = np.mean([t[:3, 3] for tt in T for t in tt], axis=0)
    en = R @ origin[:2]
    anchor = {'lat_deg': float(fit['lat0'] + np.degrees(en[1] / 6378137.0)),
              'lon_deg': float(fit['lon0'] + np.degrees(en[0] / (6378137.0 * np.cos(np.radians(fit['lat0']))))),
              'height_m': float(fit['h0'] + origin[2]), 'yaw_deg': float(fit['rot_deg'])}

    link = {}
    for yy in Y:
        for j, v in (yy[1]['vehicles'] or {}).items():
            if v['ass_id'] != -1:
                link.setdefault(j, set()).add(v['ass_id'])
    multi = {j: sorted(ks) for j, ks in link.items() if len(ks) > 1}
    gid_of = [lambda k: '0:%s' % k, lambda j: '0:%s' % min(link[j]) if j in link else '1:%s' % j]

    os.makedirs(args.out_dir, exist_ok=True)
    car_lines = [[], []]
    attribution = []
    truth_frames = []
    n_det = [0, 0]
    n_attr = [0, 0]
    for i, (f, rec) in enumerate(zip(frames, recs)):
        t_ns = T0_NS + i * DT_NS
        ego_odom = [T[i][c][:3, 3] - origin for c in (0, 1)]
        objs = {}
        own = [[], []]      # per car: (loc, gid) of its labels, in its lidar frame
        for c in (0, 1):
            P = read_pcd_xyz(f'{sc}/{c}/{f}.pcd')
            for vid, v in (Y[i][c]['vehicles'] or {}).items():
                L = np.array(v['location'], dtype=float)
                E = np.array(v['extent'], dtype=float)
                g = gid_of[c](vid)
                own[c].append((L, g))
                o = objs.setdefault(g, {'gid': g, 'labels': {}})
                o['labels'][c] = {'pos_map': (T[i][c] @ np.r_[L, 1.0])[:3], 'size': 2.0 * E,
                                  'yaw_map': yaw_deg(T[i][c]) + v['angle'][1],
                                  'points': points_in_box(P, L, v['angle'][1], E), 'type': v['obj_type']}
        attr_f = []
        detected = {}
        for c in (0, 1):
            car = rec['cars'][c]
            dets = np.asarray(car['box'], float).reshape(-1, 7)
            scores = np.asarray(car['score'], float).reshape(-1)
            m = match(dets, own[c], args.gate)
            yaw_car = yaw_deg(T[i][c])
            out = []
            gids = []
            for k, d in enumerate(dets):
                p = (T[i][c] @ np.r_[d[:3], 1.0])[:3] - origin
                rng_m = float(np.hypot(d[0], d[1]))
                out.append({
                    'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                    'sigma_m': sig(args.vert, rng_m),
                    'sigma_along_m': sig(args.along, rng_m),
                    'sigma_across_m': sig(args.across, rng_m),
                    'sigma_z_m': sig(args.vert, rng_m),
                    'class_id': 1, 'class_prob': float(scores[k]),
                    'box': {'length_m': float(d[3]), 'width_m': float(d[4]), 'height_m': float(d[5]),
                            'yaw_deg': float(wrap(yaw_car + np.degrees(d[6]))),
                            'sigma_size_m': args.sigma_size, 'sigma_yaw_deg': args.det_sigma_yaw_deg,
                            'yaw_ambiguous': True},
                })
                g = own[c][m[k]][1] if k in m else None
                gids.append(g)
                if g is not None:
                    detected.setdefault(g, set()).add(c)
            n_det[c] += len(out)
            n_attr[c] += len(m)
            attr_f.append(gids)
            pose = dict(anchor, sigma_h_m=args.sigma_h, sigma_v_m=args.sigma_v, sigma_yaw_deg=args.sigma_yaw_deg)
            car_lines[c].append(json.dumps({'t_unix_ns': t_ns, 'pose': pose, 'detections': out,
                                            'agent_xyz': ego_odom[c].tolist(),
                                            # The car's heading in the odometry frame (the map frame's
                                            # orientation): the self report's footprint heading.
                                            'agent_yaw_deg': float(wrap(yaw_deg(T[i][c])))}))
        attribution.append(attr_f)

        out_objs = []
        for g, o in sorted(objs.items()):
            labs = o['labels']
            pos = np.mean([l['pos_map'] for l in labs.values()], axis=0)
            size = np.mean([l['size'] for l in labs.values()], axis=0)
            best = max(labs.values(), key=lambda l: l['points'])
            cav = None
            for c in (0, 1):
                if np.linalg.norm((pos - origin)[:2] - ego_odom[c][:2]) < CAV_MATCH_M:
                    cav = c
            out_objs.append({
                'gid': g, 'pos': (pos - origin).tolist(), 'size': size.tolist(),
                'yaw_enu_deg': float(wrap(best['yaw_map'] + fit['rot_deg'])),
                'labeled_by': sorted(labs), 'detected_by': sorted(detected.get(g, ())),
                'points': [labs[c]['points'] if c in labs else 0 for c in (0, 1)],
                'type': best['type'], 'cav': cav,
                # Each car's own label centre (odometry frame), to identify
                # the dataset's ground-truth boxes in the AP evaluation.
                'per_car_pos': {str(c): (l['pos_map'] - origin).tolist() for c, l in labs.items()},
            })
        truth_frames.append({'frame': f, 't_ns': t_ns, 'ego_odom': [e.tolist() for e in ego_odom],
                             'objects': out_objs})

    for c in (0, 1):
        with open(os.path.join(args.out_dir, f'car{c}.jsonl'), 'w') as fh:
            fh.write('\n'.join(car_lines[c]) + '\n')
    with open(os.path.join(args.out_dir, 'attribution.json'), 'w') as fh:
        json.dump({'gate_m': args.gate, 'frames': attribution}, fh)
    truth = {
        'scenario': name, 'anchor': anchor, 'origin_map': origin.tolist(), 'dt_ns': DT_NS,
        'detections': 'PointPillars export ' + os.path.basename(args.export),
        'frames_missing_from_export': sorted(set(yaml_frames) - set(frames)),
        'gps_fit': {'rot_deg': fit['rot_deg'], 'rms_h_m': fit['rms_h'],
                    'no_gps_made_up_anchor': fit.get('no_gps', False)},
        'declared': {'pose_sigma_h_m': args.sigma_h, 'pose_sigma_v_m': args.sigma_v,
                     'pose_sigma_yaw_deg': args.sigma_yaw_deg, 'along_a_b': args.along,
                     'across_a_b': args.across, 'vert_a_b': args.vert, 'size_sigma_m': args.sigma_size,
                     'yaw_sigma_deg': args.det_sigma_yaw_deg, 'attribution_gate_m': args.gate},
        'links_to_several_car0_ids': {str(k): v for k, v in multi.items()},
        'frames': truth_frames,
    }
    with open(os.path.join(args.out_dir, 'truth.json'), 'w') as fh:
        json.dump(truth, fh)
    print('%s: %d frames (%d yaml frames not in export), detections car0 %d car1 %d, attributed %d / %d, multi-links %s'
          % (name, len(frames), len(truth['frames_missing_from_export']), n_det[0], n_det[1],
             n_attr[0], n_attr[1], multi))


if __name__ == '__main__':
    main()
