"""Convert one V2V4Real scenario into godview_node input (JSON lines) per car,
plus a truth file for the replay scorer.

Version 1 detections: a car "detects" a labeled box if the box is within
--max-range of the car and contains at least --min-points of the car's own
lidar points (so visibility and occlusion come from the real data). The
detection is the label plus seeded Gaussian noise. This is not a real
detector; it only stands in for one.

Frames (see docs/PHASE_C_HANDOFF.md, Findings):
* Labels are in each car's lidar frame; lidar_pose maps them to the
  scenario's map frame. extent is half size; yaw is angle[1] in degrees.
* The odometry frame given to GodView is the map frame shifted so its origin
  is the mean position of both cars over the scenario (shared by both cars).
  It is anchored on the Earth by a rigid fit of lidar_pose to the gps field
  (gps_fit.py), so the anchor is the same for both cars and for the truth.
* Truth: each object's position is the mean of the labels of the cars that
  labeled it (in the map frame); size is the mean size; heading is taken from
  the car with more lidar points on the object. Car 1 labels are linked to
  car 0 labels by ass_id.

Usage:
  python -I convert.py SCENARIO_DIR OUT_DIR --sigma-h 1.27 --sigma-v 2.0 \
      --sigma-yaw-deg 0.1 [--seed 1]
"""

import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gps_fit import fit_map_to_enu, yaw_deg  # noqa: E402
from v2v4real_io import load_pcd_xyz, load_yaml  # noqa: E402

DT_NS = 100_000_000  # 10 Hz, assumed from the V2V4Real paper (no timestamps in the files)
T0_NS = 1_700_000_000_000_000_000  # arbitrary fixed start time
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--sigma-h', type=float, required=True, help='pose horizontal sigma per axis (m)')
    ap.add_argument('--sigma-v', type=float, required=True, help='pose vertical sigma (m)')
    ap.add_argument('--sigma-yaw-deg', type=float, required=True, help='pose yaw sigma (deg)')
    ap.add_argument('--det-sigma', type=float, default=0.3, help='added and declared position noise (m)')
    ap.add_argument('--size-sigma', type=float, default=0.2, help='added and declared size noise (m)')
    ap.add_argument('--yaw-sigma-deg', type=float, default=3.0, help='added and declared box heading noise (deg)')
    ap.add_argument('--min-points', type=int, default=5)
    ap.add_argument('--max-range', type=float, default=120.0)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()

    sc = args.scenario_dir.rstrip('/')
    name = os.path.basename(sc)
    frames = sorted(os.path.basename(p)[:-5] for p in glob.glob(sc + '/0/*.yaml'))
    Y = [[load_yaml(f'{sc}/{c}/{f}.yaml') for c in (0, 1)] for f in frames]
    T = [[np.array(y['lidar_pose'], dtype=float) for y in yy] for yy in Y]

    fit = fit_map_to_enu([t for tt in T for t in tt], [y['gps'] for yy in Y for y in yy])
    rot = np.radians(fit['rot_deg'])
    R = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
    origin = np.mean([t[:3, 3] for tt in T for t in tt], axis=0)
    # Geodetic point of the odometry origin: map origin shifted by R * origin_xy (ENU).
    en = R @ origin[:2]
    lat_o = fit['lat0'] + np.degrees(en[1] / 6378137.0)
    lon_o = fit['lon0'] + np.degrees(en[0] / (6378137.0 * np.cos(np.radians(fit['lat0']))))
    h_o = fit['h0'] + origin[2]
    anchor = {'lat_deg': float(lat_o), 'lon_deg': float(lon_o), 'height_m': float(h_o),
              'yaw_deg': float(fit['rot_deg'])}

    # Stable global ids: car 0 label k -> "0:k"; car 1 label j -> "0:k" if it is ever
    # linked to k by ass_id in this scenario, else "1:j".
    link = {}
    for yy in Y:
        for j, v in (yy[1]['vehicles'] or {}).items():
            if v['ass_id'] != -1:
                link.setdefault(j, set()).add(v['ass_id'])
    multi = {j: sorted(ks) for j, ks in link.items() if len(ks) > 1}
    gid_of = [lambda k: '0:%s' % k, lambda j: '0:%s' % min(link[j]) if j in link else '1:%s' % j]

    # Seeded from the scenario name's characters (str hash is salted per process).
    rng = np.random.default_rng([args.seed] + [ord(ch) for ch in name])

    os.makedirs(args.out_dir, exist_ok=True)
    car_lines = [[], []]
    truth_frames = []
    class_disagree = 0
    for i, f in enumerate(frames):
        t_ns = T0_NS + i * DT_NS
        P = [load_pcd_xyz(f'{sc}/{c}/{f}.pcd') for c in (0, 1)]
        ego_odom = [T[i][c][:3, 3] - origin for c in (0, 1)]
        objs = {}
        for c in (0, 1):
            for vid, v in (Y[i][c]['vehicles'] or {}).items():
                L = np.array(v['location'], dtype=float)
                E = np.array(v['extent'], dtype=float)
                pm = (T[i][c] @ np.r_[L, 1.0])[:3]
                rng_m = float(np.hypot(L[0], L[1]))
                n_pts = points_in_box(P[c], L, v['angle'][1], E)
                g = gid_of[c](vid)
                o = objs.setdefault(g, {'gid': g, 'labels': {}})
                o['labels'][c] = {
                    'id': vid, 'pos_map': pm, 'size': 2.0 * E, 'yaw_map': yaw_deg(T[i][c]) + v['angle'][1],
                    'points': n_pts, 'range': rng_m, 'type': v['obj_type'],
                    'detected': bool(n_pts >= args.min_points and rng_m <= args.max_range),
                }
        out_objs = []
        offsets = []
        for g, o in sorted(objs.items()):
            labs = o['labels']
            pos = np.mean([l['pos_map'] for l in labs.values()], axis=0)
            size = np.mean([l['size'] for l in labs.values()], axis=0)
            best = max(labs.values(), key=lambda l: l['points'])
            if len(labs) == 2:
                offsets.append(labs[0]['pos_map'] - labs[1]['pos_map'])
                if labs[0]['type'] != labs[1]['type']:
                    class_disagree += 1
            # Is this object one of the two recording cars?
            cav = None
            for c in (0, 1):
                if np.linalg.norm((pos - origin)[:2] - ego_odom[c][:2]) < CAV_MATCH_M:
                    cav = c
            out_objs.append({
                'gid': g,
                'pos': (pos - origin).tolist(),
                'size': size.tolist(),
                'yaw_enu_deg': float(wrap(best['yaw_map'] + fit['rot_deg'])),
                'labeled_by': sorted(labs),
                'detected_by': sorted(c for c, l in labs.items() if l['detected']),
                'points': [labs[c]['points'] if c in labs else 0 for c in (0, 1)],
                'cav': cav,
                # Each car's own label (for the systematic size and heading rule).
                'per_car': {str(c): {'size': l['size'].tolist(),
                                     'yaw_enu_deg': float(wrap(l['yaw_map'] + fit['rot_deg'])),
                                     'points': l['points']} for c, l in labs.items()},
            })
        # Per-frame relative offset of linked objects, rotated into ENU.
        off = None
        if offsets:
            d = np.array(offsets)
            d_enu = np.c_[d[:, :2] @ R.T, d[:, 2]]
            off = {'n': len(d), 'mean_enu': d_enu.mean(0).tolist(), 'pairs_enu': d_enu.tolist(),
                   'rms_h': float(np.sqrt(np.mean(np.sum(d_enu[:, :2] ** 2, axis=1))))}
        truth_frames.append({'frame': f, 't_ns': t_ns, 'ego_odom': [e.tolist() for e in ego_odom],
                             'objects': out_objs, 'offset': off})

        # Detections, per car, in the odometry frame.
        for c in (0, 1):
            dets = []
            for g, o in sorted(objs.items()):
                l = o['labels'].get(c)
                # Draw noise for every label of this car so the stream does not depend on visibility.
                n = rng.normal(size=3) * args.det_sigma
                ns = rng.normal(size=3) * args.size_sigma
                ny = rng.normal() * args.yaw_sigma_deg
                if l is None or not l['detected']:
                    continue
                p = l['pos_map'] - origin + n
                dets.append({
                    'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                    'sigma_m': args.det_sigma, 'class_id': 1, 'class_prob': 0.9,
                    'box': {'length_m': float(max(l['size'][0] + ns[0], 0.1)),
                            'width_m': float(max(l['size'][1] + ns[1], 0.1)),
                            'height_m': float(max(l['size'][2] + ns[2], 0.1)),
                            'yaw_deg': float(wrap(l['yaw_map'] + ny)),
                            'sigma_size_m': args.size_sigma, 'sigma_yaw_deg': args.yaw_sigma_deg,
                            'yaw_ambiguous': False},
                })
            pose = dict(anchor, sigma_h_m=args.sigma_h, sigma_v_m=args.sigma_v, sigma_yaw_deg=args.sigma_yaw_deg)
            car_lines[c].append(json.dumps({'t_unix_ns': t_ns, 'pose': pose, 'detections': dets,
                                            'agent_xyz': ego_odom[c].tolist()}))

    for c in (0, 1):
        with open(os.path.join(args.out_dir, f'car{c}.jsonl'), 'w') as fh:
            fh.write('\n'.join(car_lines[c]) + '\n')
    truth = {
        'scenario': name, 'anchor': anchor, 'origin_map': origin.tolist(), 'dt_ns': DT_NS,
        'min_points': args.min_points, 'max_range': args.max_range,
        'gps_fit': {'rot_deg': fit['rot_deg'], 'rms_h_m': fit['rms_h'], 'max_h_m': fit['max_h'],
                    'rms_v_m': fit['rms_v'], 'heading_offset_deg': fit['heading_offset'](2)},
        'declared': {'pose_sigma_h_m': args.sigma_h, 'pose_sigma_v_m': args.sigma_v,
                     'pose_sigma_yaw_deg': args.sigma_yaw_deg, 'det_sigma_m': args.det_sigma,
                     'size_sigma_m': args.size_sigma, 'yaw_sigma_deg': args.yaw_sigma_deg, 'seed': args.seed},
        'links_to_several_car0_ids': {str(k): v for k, v in multi.items()},
        'linked_pairs_class_disagree': class_disagree,
        'frames': truth_frames,
    }
    with open(os.path.join(args.out_dir, 'truth.json'), 'w') as fh:
        json.dump(truth, fh)
    nd = [sum(len(json.loads(x)['detections']) for x in car_lines[c]) for c in (0, 1)]
    print('%s: %d frames, detections car0 %d car1 %d, gps fit rms %.2f m, class disagreements %d, multi-links %s'
          % (name, len(frames), nd[0], nd[1], fit['rms_h'], class_disagree, multi))


if __name__ == '__main__':
    main()
