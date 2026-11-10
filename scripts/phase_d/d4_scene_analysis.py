"""Phase D4: analysis and camera strips for the chosen moment.

Scene 2023-04-03-18-19-32_13_0, vehicle 1, pedestrians 43, 58, 30, 46 (ranks 1, 2, 4, 5).

* Per pedestrian: when they become visible in vehicle 1's front camera after their interval (projected
  box centre inside the image, own lidar points present: at least 5 in the box, within 50 m), and whether they
  enter the roadway (a proxy: the area swept by moving vehicle boxes and the two recording vehicles) and how they move.
* Strips every 0.5 s (frames 110 to the end) of vehicle 1's front camera and of each roadside-unit camera, with the
  four pedestrians' 3D boxes projected: ghost style (translucent fill, dashed outline) while hidden from the
  agent's own sensor (fewer than 5 of its own lidar points in the box), solid once it has them.
* Camera coverage by the roadside-unit cameras.
* The wrong-merge check in frames 110 to the end.

Usage (python -I, cv2): d4_scene_analysis.py RAW_SCENE_DIR CONV_DIR DUMP.jsonl CANDIDATES.json OUT_DIR
"""
import json
import os
import sys

import cv2
import numpy as np
import yaml

L = yaml.CSafeLoader
PEDS = ['43', '58', '30', '46']
COLORS = {'43': (60, 60, 255), '58': (0, 200, 255), '30': (255, 120, 40), '46': (60, 220, 60)}   # BGR
FIRST = 110
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


def corners_map(o, origin):
    c = np.array(o['pos']) + origin
    half = np.array(o['size']) / 2
    R = rot(0, o['yaw_enu_deg'], 0)
    return np.array([c + R @ (half * np.array(s)) for s in
                     [(1, 1, 1), (1, -1, 1), (-1, -1, 1), (-1, 1, 1), (1, 1, -1), (1, -1, -1), (-1, -1, -1), (-1, 1, -1)]])


def project(corners, T_agent, E, K):
    inv = np.linalg.inv(T_agent)
    Einv = np.linalg.inv(E)
    out = []
    for c in corners:
        pc = (Einv @ inv @ np.r_[c, 1.0])[:3]
        out.append((K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2], pc[2]))
    return np.array(out)


def count_points(P, c, R, half):
    r2 = float(np.sum(np.square(half))) + 1e-6
    Q = P[np.sum(np.square(P - c), axis=1) <= r2]
    return int(np.sum(np.all(np.abs((Q - c) @ R) <= half, axis=1))) if len(Q) else 0


EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def dashed(img, p, q, color, th):
    d = np.hypot(q[0] - p[0], q[1] - p[1])
    n = int(d // 14)
    for k in range(0, n, 2):
        a = (p[0] + (q[0] - p[0]) * k / max(n, 1), p[1] + (q[1] - p[1]) * k / max(n, 1))
        b = (p[0] + (q[0] - p[0]) * min(k + 1, n) / max(n, 1), p[1] + (q[1] - p[1]) * min(k + 1, n) / max(n, 1))
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), color, th, cv2.LINE_AA)


def draw(img, uvz, color, solid, label):
    if np.any(uvz[:, 2] <= 0.5):
        return
    pts = uvz[:, :2]
    if not solid:
        hull = cv2.convexHull(pts.astype(np.int32))
        ov = img.copy()
        cv2.fillConvexPoly(ov, hull, color)
        cv2.addWeighted(ov, 0.30, img, 0.70, 0, img)
    for a, b in EDGES:
        if solid:
            cv2.line(img, tuple(pts[a].astype(int)), tuple(pts[b].astype(int)), color, 4, cv2.LINE_AA)
        else:
            dashed(img, pts[a], pts[b], color, 3)
    top = pts[np.argmin(pts[:, 1])]
    cv2.putText(img, label, (int(top[0]) - 20, int(top[1]) - 12), cv2.FONT_HERSHEY_SIMPLEX, 1.3, color, 3, cv2.LINE_AA)


def main():
    raw, conv, dump, cand, out = sys.argv[1:6]
    os.makedirs(out, exist_ok=True)
    T = json.load(open(os.path.join(conv, 'truth.json')))
    frames, agents = T['frames'], T['agents']
    origin = np.array(T['origin_map'])
    n = len(frames)
    cands = {c['gid']: c for c in json.load(open(cand)) if c['scenario'] == T['scenario'] and c['vehicle'] == '1' and c['gid'] in PEDS}
    dumps = {}
    for l in open(dump):
        if l.strip():
            e = json.loads(l)
            dumps[e['frame']] = e
    v1 = [a['index'] for a in agents if a['folder'] == '1'][0]
    folder = {a['index']: a['folder'] for a in agents}
    # raw yaml: poses and cameras
    ypose, ycam = {}, {}
    for k, a in enumerate(agents):
        for i in range(FIRST - 10, n):
            y = yaml.load(open(os.path.join(raw, a['folder'], frames[i]['frame'] + '.yaml')), Loader=L)
            ypose[(k, i)] = pose_matrix(np.array(y['lidar_pose'], float))
            ycam[(k, i)] = {c: (np.array(y[c]['extrinsic'], float), np.array(y[c]['intrinsic'], float)) for c in y if c.startswith('cam')}
    objs = [{o['gid']: o for o in fr['objects']} for fr in frames]
    # own lidar points of vehicle 1 in each pedestrian's box, and visibility
    npts = {g: [None] * n for g in PEDS}
    for i in range(FIRST - 10, n):
        P = np.fromfile(os.path.join(raw, '1', frames[i]['frame'] + '.bin'), dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
        inv = np.linalg.inv(ypose[(v1, i)])
        for g in PEDS:
            o = objs[i].get(g)
            if o is None:
                continue
            c = (inv @ np.r_[np.array(o['pos']) + origin, 1.0])[:3]
            npts[g][i] = count_points(P, c, inv[:3, :3] @ rot(0, o['yaw_enu_deg'], 0), np.array(o['size']) / 2)
    res = {'scene': T['scenario'], 'pedestrians': {}}
    # roadway proxy: cells swept by moving vehicle-class boxes and the recording vehicles
    cell, driven = 1.0, set()
    prev = {}
    for i in range(n):
        for gid, o in objs[i].items():
            if o['class_id'] in (1, 3) and o['agent'] is None:
                p = np.array(o['pos'][:2])
                if gid in prev and np.linalg.norm(p - prev[gid]) * 10 >= 1.0:
                    R = rot(0, o['yaw_enu_deg'], 0)[:2, :2]
                    for u in np.arange(-o['size'][0] / 2, o['size'][0] / 2 + 0.01, 0.5):
                        for w_ in np.arange(-o['size'][1] / 2, o['size'][1] / 2 + 0.01, 0.5):
                            q = p + R @ np.array([u, w_])
                            driven.add((int(np.floor(q[0] / cell)), int(np.floor(q[1] / cell))))
                prev[gid] = p
        for k in (2, 3):
            e = np.array(frames[i]['ego_odom'][k][:2])
            for u in np.arange(-2.2, 2.3, 0.5):
                for w_ in np.arange(-1.0, 1.1, 0.5):
                    pass
            driven.add((int(np.floor(e[0] / cell)), int(np.floor(e[1] / cell))))
    def on_driven(p):
        return (int(np.floor(p[0] / cell)), int(np.floor(p[1] / cell))) in driven
    cam1_E, cam1_K = None, None
    for g in PEDS:
        c = cands[g]
        s_, e_ = c['start_frame'], c['end_frame']
        vis_frames = []
        for i in range(e_ + 1, n):
            o = objs[i].get(g)
            if o is None or npts[g][i] is None:
                continue
            Tm = ypose[(v1, i)]
            E, K = ycam[(v1, i)]['cam1']
            uvz = project(corners_map(o, origin), Tm, E, K)
            ctr = project(np.array([np.array(o['pos']) + origin]), Tm, E, K)[0]
            d = float(np.hypot(*((np.array(o['pos']) + origin)[:2] - Tm[:2, 3])))
            if 0 <= ctr[0] < W and 0 <= ctr[1] < H and ctr[2] > 3 and npts[g][i] >= 5 and d <= 50:
                vis_frames.append(i)
        first = vis_frames[0] if vis_frames else None
        # trajectory in vehicle-1 frame at key frames
        def rel(i):
            o = objs[i].get(g)
            if o is None:
                return None
            Tm = ypose[(v1, i)]
            p = np.linalg.inv(Tm) @ np.r_[np.array(o['pos']) + origin, 1.0]
            return [round(float(p[0]), 1), round(float(p[1]), 1)]
        traj = {i: objs[i][g] for i in range(s_ - 5, n) if g in objs[i]}
        pos = {i: np.array(o['pos'][:2]) for i, o in traj.items()}
        ids = sorted(pos)
        speed = [float(np.linalg.norm(pos[b] - pos[a]) / ((b - a) / 10)) for a, b in zip(ids[:-5:5], ids[5::5])]
        drv = [i for i in ids if on_driven(pos[i])]
        res['pedestrians'][g] = {
            'rank': c['rank'], 'interval_frames': [s_, e_], 'interval_s': (e_ - s_ + 1) / 10,
            'first_visible_after_interval': first, 'delay_after_end_frames': (first - e_) if first is not None else None,
            'delay_after_end_s': ((first - e_) / 10) if first is not None else None,
            'visible_frames_after_interval': len(vis_frames), 'last_scene_frame': n - 1,
            'visible_until_scene_end': bool(vis_frames) and vis_frames[-1] == n - 1,
            'position_in_vehicle1_frame_m_forward_left': {'interval_start': rel(s_), 'interval_end': rel(e_), 'first_visible': rel(first) if first is not None else None, 'scene_end': rel(n - 1)},
            'median_speed_m_s': float(np.median(speed)) if speed else None,
            'frames_on_driven_area_proxy': len(drv), 'first_frame_on_driven_area_proxy': drv[0] if drv else None,
            'own_points_at_first_visible': npts[g][first] if first is not None else None,
        }
    # strips
    sel = list(range(FIRST, n, 5)) + ([n - 1] if (n - 1 - FIRST) % 5 else [])
    cams = [(v1, 'cam1', 'vehicle1_front'), (0, 'cam1', 'unit_-2_cam1'), (0, 'cam2', 'unit_-2_cam2'), (1, 'cam1', 'unit_-1_cam1'), (1, 'cam2', 'unit_-1_cam2')]
    cover = {}
    for k, cam, name in cams:
        tiles = []
        stats = {'inside': 0, 'tall_enough': 0, 'pairs': 0, 'heights': [], 'simult': []}
        for i in range(FIRST, n):
            n_in = 0
            for g in PEDS:
                o = objs[i].get(g)
                if o is None:
                    continue
                E, K = ycam[(k, i)][cam]
                uvz = project(corners_map(o, origin), ypose[(k, i)], E, K)
                stats['pairs'] += 1
                ctr = project(np.array([np.array(o['pos']) + origin]), ypose[(k, i)], E, K)[0]
                if ctr[2] > 1 and 0 <= ctr[0] < W and 0 <= ctr[1] < H and np.all(uvz[:, 2] > 0.5):
                    h_px = float(uvz[:, 1].max() - uvz[:, 1].min())
                    stats['inside'] += 1
                    n_in += 1
                    stats['heights'].append(h_px)
                    stats['tall_enough'] += h_px >= 40
            stats['simult'].append(n_in)
        cover[name] = {'share_of_pedestrian_frames_inside_image': stats['inside'] / max(1, stats['pairs']),
                       'share_inside_and_at_least_40_px_tall': stats['tall_enough'] / max(1, stats['pairs']),
                       'median_height_px_when_inside': float(np.median(stats['heights'])) if stats['heights'] else None,
                       'mean_pedestrians_in_image_per_frame': float(np.mean(stats['simult']))}
        for i in sel:
            img = cv2.imread(os.path.join(raw, folder[k], frames[i]['frame'] + '_%s.jpeg' % cam))
            if img is None:
                img = np.zeros((H, W, 3), np.uint8)
            for g in PEDS:
                o = objs[i].get(g)
                if o is None:
                    continue
                E, K = ycam[(k, i)][cam]
                uvz = project(corners_map(o, origin), ypose[(k, i)], E, K)
                # solid once the agent's own sensor has points in the box
                own = npts[g][i] if k == v1 else None
                if k != v1:
                    P = np.fromfile(os.path.join(raw, folder[k], frames[i]['frame'] + '.bin'), dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
                    inv = np.linalg.inv(ypose[(k, i)])
                    c = (inv @ np.r_[np.array(o['pos']) + origin, 1.0])[:3]
                    own = count_points(P, c, inv[:3, :3] @ rot(0, o['yaw_enu_deg'], 0), np.array(o['size']) / 2)
                draw(img, uvz, COLORS[g], bool(own is not None and own >= 5), g)
            img = cv2.resize(img, (480, 270), interpolation=cv2.INTER_AREA)
            cv2.rectangle(img, (0, 0), (480, 22), (0, 0, 0), -1)
            cv2.putText(img, '%s  frame %d  t=%.1fs' % (name, i, i / 10), (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(img)
        cols = 4
        while len(tiles) % cols:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [cv2.hconcat(tiles[r:r + cols]) for r in range(0, len(tiles), cols)]
        cv2.imwrite(os.path.join(out, 'strip_%s.jpg' % name), cv2.vconcat(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
    res['camera_coverage_frames_%d_to_end' % FIRST] = cover
    # wrong merges, frames FIRST to the end
    bodies_all = {gid for fr in frames for gid, o in objs[0].items()}  # placeholder, replaced below
    wm = {'agent_involving': [], 'involving_the_four': [], 'involving_vehicles_around_vehicle1': [], 'all': 0}
    for i in range(FIRST, n):
        bodies = {o['gid'] for o in frames[i]['objects'] if o['agent'] is not None}
        v1pos = np.array(frames[i]['ego_odom'][v1][:2])
        around = {o['gid'] for o in frames[i]['objects'] if o['class_id'] in (1, 3) and o['agent'] is None
                  and np.linalg.norm(np.array(o['pos'][:2]) - v1pos) <= 30}
        for ag in dumps[i]['agents']:
            for view in ('ego', 'shared'):
                for o in ag[view]:
                    labs = set(o['labels'])
                    if len(labs) < 2:
                        continue
                    if view == 'ego':
                        wm['all'] += 1
                    rec = {'frame': i, 'agent': folder[ag['index']], 'view': view, 'labels': sorted(labs)}
                    if labs & bodies:
                        wm['agent_involving'].append(rec)
                    if labs & set(PEDS):
                        wm['involving_the_four'].append(rec)
                    if labs & around:
                        wm['involving_vehicles_around_vehicle1'].append(rec)
    res['wrong_merges_frames_%d_to_end' % FIRST] = {'ego_view_wrong_merges_all': wm['all'], **{k: len(v) for k, v in wm.items() if k != 'all'},
                                                   'examples': {k: v[:5] for k, v in wm.items() if k != 'all'}}
    json.dump(res, open(os.path.join(out, 'analysis.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != 'pedestrians'}, indent=1))
    for g, r in res['pedestrians'].items():
        print(g, json.dumps(r))


if __name__ == '__main__':
    main()
