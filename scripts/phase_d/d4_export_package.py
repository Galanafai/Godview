"""Phase D4: export the scene package for the demo (frames 100 to 239 of scene 2023-04-03-18-19-32_13_0).

Everything is in the shared odom frame (map frame minus origin_map, east north up, metres), the frame of
truth.json, the detections and the ego view. Detections are dataset annotations with noise (labels plus noise),
not a trained detector.

Usage (python -I, rerun venv): d4_export_package.py RAW_SCENE_DIR CONV_DIR DUMP.jsonl OUT_DIR
"""
import json
import os
import shutil
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d4_scene_analysis import rot, pose_matrix  # noqa: E402

L = yaml.CSafeLoader
F0, F1 = 100, 239
PEDS = ['43', '58', '30', '46']


def voxel(P, v):
    if len(P) == 0:
        return P
    k = np.floor(P / v).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[np.sort(idx)]


def in_boxes(P, objs, origin, grow=0.15):
    m = np.zeros(len(P), bool)
    for o in objs:
        c = np.array(o['pos']) + origin
        R = rot(0, o['yaw_enu_deg'], 0)
        q = (P - c) @ R
        m |= np.all(np.abs(q) <= np.array(o['size']) / 2 + grow, axis=1)
    return m


def main():
    raw, conv, dump, out = sys.argv[1:5]
    T = json.load(open(os.path.join(conv, 'truth.json')))
    att = json.load(open(os.path.join(conv, 'attribution.json')))['frames']
    origin = np.array(T['origin_map'])
    agents = T['agents']
    v1 = [a['index'] for a in agents if a['folder'] == '1'][0]
    det = {a['index']: [json.loads(l) for l in open(os.path.join(conv, f'agent{a["index"]}.jsonl'))] for a in agents}
    dumps = {}
    for l in open(dump):
        d = json.loads(l)
        dumps[d['frame']] = d
    os.makedirs(out, exist_ok=True)
    for sub in ('cameras', 'frames', 'points'):
        os.makedirs(os.path.join(out, sub), exist_ok=True)

    cams = {}          # folder -> list of cam names
    for a in agents:
        names = sorted({f.split('_')[1].split('.')[0] for f in os.listdir(os.path.join(raw, a['folder'])) if f.startswith('%06d_' % F1)})
        cams[a['folder']] = names
    calib = {}
    static_chunks, moving = [], {}
    for i in range(F0, F1 + 1):
        fr = '%06d' % i
        rec = {'frame': i, 't_s': (i - F0) * T['dt_ns'] / 1e9, 'agents': [], 'truth': T['frames'][i]['objects'], 'ego_view': None}
        objs = T['frames'][i]['objects']
        mv = []
        for a in agents:
            y = yaml.load(open(os.path.join(raw, a['folder'], fr + '.yaml')), Loader=L)
            lp = np.array(y['lidar_pose'], float)
            Tm = pose_matrix(lp)
            P = np.fromfile(os.path.join(raw, a['folder'], fr + '.bin'), dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
            Pm = (Tm[:3, :3] @ P.T).T + Tm[:3, 3] - origin
            Pm = Pm[np.linalg.norm(P, axis=1) < 80]
            inb = in_boxes(Pm + origin, objs, origin)
            mv.append(voxel(Pm[inb], 0.08))
            if i % 5 == 0:
                static_chunks.append(Pm[~inb])
            ad = det[a['index']][i]
            rec['agents'].append({
                'index': a['index'], 'folder': a['folder'], 'kind': a['kind'],
                'lidar_pose_map': lp.tolist(), 'ego_odom': T['frames'][i]['ego_odom'][a['index']],
                'pose_declared': ad['pose'], 'detections': ad['detections'], 'detected_labels': att[i][a['index']]})
            for c in cams[a['folder']]:
                key = f'{a["folder"]}/{c}'
                cy = y[c]
                if a['kind'] == 'vehicle':
                    calib.setdefault(key, {})[fr] = {'intrinsic': cy['intrinsic'], 'extrinsic_cam_to_lidar': cy['extrinsic'], 'lidar_pose_map': lp.tolist()}
                elif key not in calib:
                    calib[key] = {'static': True, 'intrinsic': cy['intrinsic'], 'extrinsic_cam_to_lidar': cy['extrinsic'], 'lidar_pose_map': lp.tolist()}
                d = os.path.join(out, 'cameras', a['folder'], c)
                os.makedirs(d, exist_ok=True)
                shutil.copyfile(os.path.join(raw, a['folder'], f'{fr}_{c}.jpeg'), os.path.join(d, fr + '.jpeg'))
        dv = dumps[i]['agents'][v1]['ego']
        rec['ego_view'] = [{
            'id': o['id'], 'status': 'solid' if v1 in o['sources'] else 'ghost',
            'x': o['x'], 'y': o['y'], 'z': o['z'], 'cov_pos': o['cov_pos'], 'vel': o['vel'], 'p_exist': o['p_exist'],
            'class_id': o['class_id'], 'class_prob': o['class_prob'], 'box': o['box'],
            'contributing_agents': o['sources'], 'self_agent': o['self_agent'], 'members': o['members'],
            'attributed_labels': o['labels'], 'merged_labels': len(o['labels']) >= 2} for o in dv]
        json.dump(rec, open(os.path.join(out, 'frames', fr + '.json'), 'w'))
        moving[fr] = voxel(np.vstack(mv), 0.08).astype(np.float32)
    static = voxel(np.vstack(static_chunks), 0.2).astype(np.float32)
    np.save(os.path.join(out, 'points', 'static_odom.npy'), static)
    np.savez_compressed(os.path.join(out, 'points', 'moving_odom.npz'), **moving)
    json.dump(calib, open(os.path.join(out, 'calibration.json'), 'w'))
    json.dump({
        'scene': T['scenario'], 'frames': [F0, F1], 'rate_hz': 10, 'frame_note': 'frame numbers are dataset frame indices; t_s counts from frame 100',
        'frame_of_reference': 'odom: east north up metres, map frame minus origin_map', 'origin_map': T['origin_map'],
        'agents': agents, 'vehicle_of_interest': {'folder': '1', 'index': v1}, 'pedestrian_labels': PEDS,
        'cameras': cams, 'detections': 'dataset annotations with noise (labels plus noise), not a trained detector; sigma 0.3 m, size 0.2 m, yaw 3 deg, class_prob 0.9, at least 5 own lidar points, within 100 m',
        'ego_view': 'GodView snapshot_ego of vehicle 1 per frame; status solid when vehicle 1 itself contributes (its own sensor is among contributing_agents), ghost when only other agents do',
        'points': {'static_odom.npy': 'N x 3 float32, every 5th frame of all four lidars, points outside label boxes, voxel 0.2 m',
                   'moving_odom.npz': 'per frame key (6 digit frame) M x 3 float32, points inside label boxes (+0.15 m) of all four lidars, voxel 0.08 m'},
        'settings': 'selected tracker, self reports, single per source, ego view, offset filter off, no anchors, no injected pose error'},
        open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print('static', static.shape, 'moving mean', np.mean([len(v) for v in moving.values()]))


if __name__ == '__main__':
    main()
