"""Phase D2 step 1: format survey of a V2X-Real split (OPV2V layout).

Measures, instead of assuming:
* agents per scenario, frames per agent, file types, point cloud layout;
* the pose convention (6-vector [x, y, z, roll, yaw, pitch], degrees, the
  OpenCOOD `x_to_world` matrix) and whether `true_ego_pose` differs from
  `lidar_pose`; static or moving agents;
* the frame of the labels: the number of the agent's own lidar points inside
  each label's box under two hypotheses (labels in the world frame with
  `extent` as half size; labels already in the lidar frame);
* label ids: shared across agents or per agent; whether agents appear as labels;
* class names; per-agent visibility;
* frame rate, from the agents' displacement against `ego_speed`;
* cameras: image size, the meaning of `extrinsic` (tested both ways by how
  many labeled objects in front of the camera project inside the image).

Usage (python -I): survey_v2xreal.py SPLIT_DIR OUT.json
"""
import json
import os
import struct
import sys
from collections import Counter, defaultdict

import numpy as np
import yaml


def x_to_world(p):
    x, y, z, roll, yaw, pitch = p
    c_y, s_y = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    c_r, s_r = np.cos(np.radians(roll)), np.sin(np.radians(roll))
    c_p, s_p = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    m = np.eye(4)
    m[0, 0] = c_p * c_y
    m[0, 1] = c_y * s_p * s_r - s_y * c_r
    m[0, 2] = -c_y * s_p * c_r - s_y * s_r
    m[1, 0] = s_y * c_p
    m[1, 1] = s_y * s_p * s_r + c_y * c_r
    m[1, 2] = -s_y * s_p * c_r + c_y * s_r
    m[2, 0] = s_p
    m[2, 1] = -c_p * s_r
    m[2, 2] = c_p * c_r
    m[:3, 3] = [x, y, z]
    return m


def jpeg_size(path):
    with open(path, 'rb') as f:
        d = f.read(200000)
    i = 2
    while i < len(d):
        if d[i] != 0xFF:
            i += 1
            continue
        mk = d[i + 1]
        if mk in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack('>HH', d[i + 5:i + 9])
            return w, h
        i += 2 + struct.unpack('>H', d[i + 2:i + 4])[0]
    return None


def points_in_boxes(P, T_lidar, labels, hyp):
    inv = np.linalg.inv(T_lidar)
    out = []
    for lid, v in labels.items():
        loc = np.array(v['location'], float)
        R_box = x_to_world([0, 0, 0] + [v['angle'][0], v['angle'][1], v['angle'][2]])[:3, :3]
        if hyp == 'world':
            c = (inv @ np.r_[loc, 1.0])[:3]
            R = inv[:3, :3] @ R_box
        else:
            c, R = loc, R_box
        q = (P[:, :3] - c) @ R            # points in the box frame
        h = np.array(v['extent'], float)
        out.append(int(np.sum(np.all(np.abs(q) <= h, axis=1))))
    return out


def main():
    root, out_path = sys.argv[1:3]
    res = {'scenarios': {}}
    classes, ids_shared, ids_total = Counter(), Counter(), Counter()
    inside = {'world': [], 'lidar': []}
    static, speed_rows, proj = {}, [], {'cam_to_lidar': [0, 0], 'lidar_to_cam': [0, 0]}
    img_sizes, pts, derived = Counter(), defaultdict(list), Counter()
    for scen in sorted(os.listdir(root)):
        sd = os.path.join(root, scen)
        agents = sorted(os.listdir(sd), key=int)
        info = {'agents': {}}
        data = {}
        for a in agents:
            names = os.listdir(os.path.join(sd, a))
            frames = sorted({n.split('.')[0].split('_')[0] for n in names})
            info['agents'][a] = {'frames': len(frames), 'files': dict(Counter(
                'bin' if n.endswith('.bin') else 'yaml' if n.endswith('.yaml') else 'cam' + n.split('_cam')[1][0] for n in names)),
                'infra': None}
            data[a] = frames
        info['same_frame_ids_across_agents'] = len({tuple(f) for f in data.values()}) == 1
        # Per frame (every 5th) checks
        for fi in data[agents[0]][::5]:
            ys = {a: yaml.safe_load(open(os.path.join(sd, a, fi + '.yaml'))) for a in agents}
            for a in agents:
                info['agents'][a]['infra'] = ys[a]['infra']
            # ids
            keysets = {a: set(ys[a]['vehicles'].keys()) for a in agents}
            union = set().union(*keysets.values())
            for k in union:
                ids_total['labels'] += 1
                ids_shared['seen_by_all_agents'] += all(k in ks for ks in keysets.values())
            for a in agents:
                for k, v in ys[a]['vehicles'].items():
                    classes[v['obj_type']] += 1
            # same id, same place?
            for k in union:
                locs = [np.array(ys[a]['vehicles'][k]['location']) for a in agents if k in ys[a]['vehicles']]
                if len(locs) > 1:
                    derived['same id, max pairwise location difference (m): ' + ('< 0.01' if max(np.linalg.norm(l - locs[0]) for l in locs) < 0.01 else '>= 0.01')] += 1
            # lidar points in boxes (sampled frames only, first two agents of each kind)
            for a in agents:
                T = x_to_world(ys[a]['lidar_pose'])
                P = np.fromfile(os.path.join(sd, a, fi + '.bin'), dtype=np.float32).reshape(-1, 4)
                pts[('infra' if ys[a]['infra'] else 'vehicle', a if ys[a]['infra'] else 'v')].append(len(P))
                if int(fi) % 10 == 0:
                    lab = {k: v for k, v in ys[a]['vehicles'].items()
                           if np.linalg.norm((np.linalg.inv(T) @ np.r_[v['location'], 1.0])[:2]) < 80}
                    if lab:
                        inside['world'] += points_in_boxes(P, T, lab, 'world')
                        inside['lidar'] += points_in_boxes(P, T, lab, 'lidar')
                # cameras
                if int(fi) % 10 == 0:
                    for cam in [k for k in ys[a] if k.startswith('cam')]:
                        c = ys[a][cam]
                        E, K = np.array(c['extrinsic']), np.array(c['intrinsic'])
                        img = os.path.join(sd, a, '%s_%s.jpeg' % (fi, cam))
                        sz = jpeg_size(img)
                        img_sizes[(a if ys[a]['infra'] else 'vehicle', cam, sz)] += 1
                        Tinv = np.linalg.inv(T)
                        for v in ys[a]['vehicles'].values():
                            pl = (Tinv @ np.r_[v['location'], 1.0])
                            for name, M in (('cam_to_lidar', np.linalg.inv(E)), ('lidar_to_cam', E)):
                                pc = (M @ pl)[:3]
                                if pc[2] > 3 and np.linalg.norm(pl[:2]) < 40 and abs(pc[0]) < pc[2] * 0.3:
                                    # roughly in front of this camera under this hypothesis
                                    u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
                                    vv = K[1, 1] * pc[1] / pc[2] + K[1, 2]
                                    proj[name][0] += 1
                                    proj[name][1] += (0 <= u < (sz[0] if sz else 1920) and 0 <= vv < (sz[1] if sz else 1080))
        # poses: static or moving; speed
        poses = {}
        for a in agents:
            poses[a] = np.array([yaml.safe_load(open(os.path.join(sd, a, f + '.yaml')))['lidar_pose'] for f in data[a]])
            tp = np.array([yaml.safe_load(open(os.path.join(sd, a, f + '.yaml')))['true_ego_pose'] for f in data[a][:20]])
            derived['true_ego_pose equals lidar_pose in first 20 frames' + (': yes' if np.allclose(tp, poses[a][:20]) else ': no')] += 1
            info['agents'][a]['pose_xyz_std_m'] = [round(float(x), 3) for x in poses[a][:, :3].std(0)]
            info['agents'][a]['pose_xy_path_length_m'] = round(float(np.linalg.norm(np.diff(poses[a][:, :2], axis=0), axis=1).sum()), 1)
            if int(a) > 0:
                sp = [yaml.safe_load(open(os.path.join(sd, a, f + '.yaml')))['ego_speed'] for f in data[a]]
                step = np.r_[np.nan, np.linalg.norm(np.diff(poses[a][:, :2], axis=0), axis=1)]
                for s, d in zip(sp, step):
                    if s and s > 5 and d == d:
                        speed_rows.append((s, d))
        res['scenarios'][scen] = info
    r = np.array(speed_rows) if speed_rows else np.zeros((0, 2))
    res['frame_rate'] = {}
    if len(r):
        # dt = displacement / speed, speed in km/h (OPV2V) -> m/s
        dt = r[:, 1] / (r[:, 0] / 3.6)
        res['frame_rate'] = {'n': len(r), 'dt_s_median_if_ego_speed_kmh': float(np.median(dt)),
                             'dt_s_p10_p90': [float(np.percentile(dt, 10)), float(np.percentile(dt, 90))]}
    res['classes'] = dict(classes)
    res['labels_seen_by_all_agents_share'] = ids_shared['seen_by_all_agents'] / max(1, ids_total['labels'])
    res['derived_counts'] = dict(derived)
    for h in inside:
        v = np.array(inside[h])
        res['points_in_label_box_hypothesis_' + h] = {'labels': len(v), 'median_points': float(np.median(v)) if len(v) else None,
                                                      'share_with_5_or_more': float(np.mean(v >= 5)) if len(v) else None}
    res['camera_projection_test'] = {k: {'objects_in_front': v[0], 'inside_image': int(v[1])} for k, v in proj.items()}
    res['image_sizes'] = {'%s %s %s' % k: v for k, v in img_sizes.items()}
    res['points_per_frame'] = {'%s %s' % k: {'min': int(min(v)), 'median': int(np.median(v)), 'max': int(max(v))} for k, v in pts.items()}
    json.dump(res, open(out_path, 'w'), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != 'scenarios'}, indent=1))
    for s, i in res['scenarios'].items():
        print(s, {a: (x['frames'], 'infra' if x['infra'] else 'veh', x['pose_xy_path_length_m']) for a, x in i['agents'].items()})


if __name__ == '__main__':
    main()
