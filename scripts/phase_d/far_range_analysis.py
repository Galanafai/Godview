"""Phase D1 (explain only): why GodView trails Late Fusion at 30 m and beyond.

For every ground-truth box (car 0's frame, the dataset's) with centroid 30 to
100 m from car 0, whether each method has a box with bird's-eye IoU >= 0.5
("covered"), and for the boxes R1 does not cover, the first reason that
applies:
* wrong merge: an R1 world object within 3 m of the box centre whose members
  are attributed to two or more labels;
* localization: an R1 object within 3 m of the box centre but IoU < 0.5;
* not detected: no detection attributed to this object by car 0 in this frame
  or by car 1 in the previous frame (the report car 0 holds under the 100 ms
  delay);
* below threshold: detected, but every such detection scores below 0.3 (the
  publish threshold);
* fresh-only: detected, no R1 box, but R3 (same setting, coasting kept) covers it;
* no track: detected, neither R1 nor R3 covers it (confirmation after 3 hits,
  gating, overlap pruning).
Also boxes per method in the range and how many of them cover a ground-truth
box (IoU >= 0.5), for the precision side.

Usage (python -I): far_range_analysis.py OPENCOOD EXPORT INPUTS R1_DUMPS R3_DUMPS OUT.json
"""
import gzip
import json
import os
import sys
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ap_eval as E  # noqa: E402

BANDS = {'30_50': (30, 50), '50_100': (50, 100)}


def main():
    opencood, export, inputs, d_r1, d_r3, out = sys.argv[1:7]
    sys.path.insert(0, opencood)
    import torch
    from opencood.utils import box_utils
    recs = [json.loads(l) for l in gzip.open(export, 'rt')]
    scen = sorted({r['scenario'] for r in recs})
    truth = {s: json.load(open(os.path.join(inputs, s, 'truth.json'))) for s in scen}
    attr = {s: json.load(open(os.path.join(inputs, s, 'attribution.json')))['frames'] for s in scen}
    fidx = {s: {f['frame']: i for i, f in enumerate(truth[s]['frames'])} for s in scen}
    recs_by = {(r['scenario'], fidx[r['scenario']][r['timestamp']]): r for r in recs}
    dumps = {n: {s: [json.loads(l) for l in open(os.path.join(d, s + '.dump.jsonl')) if l.strip()] for s in scen}
             for n, d in (('R1', d_r1), ('R3', d_r3))}
    cover = {b: defaultdict(Counter) for b in BANDS}     # band -> method -> counter
    reasons = {b: {'R1 uncovered': Counter(), 'LF@0.3 covers, R1 does not': Counter()} for b in BANDS}
    boxes = {b: defaultdict(Counter) for b in BANDS}
    speed_cover = defaultdict(lambda: defaultdict(Counter))   # speed bin -> method -> counter
    loc_cases = []
    for r in recs:
        s, i = r['scenario'], fidx[r['scenario']][r['timestamp']]
        pose0 = np.array(r['cars'][0]['lidar_pose'])
        inv0 = np.linalg.inv(pose0)
        origin = np.array(truth[s]['origin_map'])
        gt = np.array(r['gt_corners_ego'], float).reshape(-1, 8, 3)
        if not len(gt):
            continue
        gp = E.polys(gt)
        gd = E.dists(gp)
        far = [j for j in range(len(gt)) if gd[j] > 30]
        if not far:
            continue
        # ground-truth identities
        cand = [(o['gid'], np.array(v)) for o in truth[s]['frames'][i]['objects'] for v in o['per_car_pos'].values()]
        gids = []
        for c in gt.mean(axis=1):
            pm = pose0[:3, :3] @ c + pose0[:3, 3] - origin
            d = [np.linalg.norm(pm - v) for _, v in cand]
            k = int(np.argmin(d)) if d else -1
            gids.append(cand[k][0] if k >= 0 and d[k] <= 0.2 else None)
        # methods' boxes in car 0's frame
        M = {}
        for t in (0.2, 0.3):
            pc, ps = [], []
            for car in r['cars']:
                c = np.array(car['corners'], float).reshape(-1, 8, 3)
                sc = np.array(car['score'], float)
                keep = sc >= t
                T = np.array(car['T_to_ego'])
                if keep.any():
                    pc.append(c[keep] @ T[:3, :3].T + T[:3, 3])
                    ps.append(sc[keep])
            if pc:
                tc, ts = torch.tensor(np.vstack(pc)).float(), torch.tensor(np.concatenate(ps)).float()
                kp = box_utils.nms_rotated(tc, ts, 0.15)
                tc, ts = tc[kp], ts[kp]
                m = box_utils.get_mask_for_boxes_within_range_torch(tc)
                M['LF@%s' % t] = (tc[m].numpy().astype(float), None)
            else:
                M['LF@%s' % t] = (np.zeros((0, 8, 3)), None)
        for n in ('R1', 'R3'):
            fr = dumps[n][s][i]
            assert fr['frame'] == i
            objs = fr['worlds'][0]
            corners, labs = [], []
            for o in objs:
                b = o.get('box')
                if b is None:
                    continue
                a = np.radians(b['yaw_deg'])
                R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
                c = (E.TEMPLATE * [b['l'], b['w'], b['h']]) @ R.T + [o['x'], o['y'], o['z']] + origin
                corners.append(c @ inv0[:3, :3].T + inv0[:3, 3])
                labs.append((o.get('labels') or [], len(o.get('sources') or [])))
            corners = np.array(corners).reshape(-1, 8, 3)
            mk = E.in_range_mask(corners)
            M[n] = (corners[mk], [l for l, k in zip(labs, mk) if k])
        ious, cent = {}, {}
        for n, (cb, _) in M.items():
            dp = E.polys(cb)
            ious[n] = E.iou_matrix(dp, [gp[j] for j in far]) if len(cb) else np.zeros((0, len(far)))
            cent[n] = np.array([[p.centroid.x, p.centroid.y] for p in dp]).reshape(-1, 2)
            dd = E.dists(dp) if len(dp) else np.zeros(0)
            for band, (lo, hi) in BANDS.items():
                inb = (dd > lo) & (dd < hi)
                # boxes in the band and how many cover any ground-truth box (all ground truth)
                full = E.iou_matrix([dp[k] for k in np.where(inb)[0]], gp) if inb.any() else np.zeros((0, len(gp)))
                boxes[band][n]['boxes'] += int(inb.sum())
                boxes[band][n]['cover_a_gt'] += int((full.max(axis=1) >= 0.5).sum()) if full.size else 0
        att0 = attr[s][i][0]
        att1 = attr[s][i - 1][1] if i > 0 else []
        sc0 = r['cars'][0]['score']
        for jj, j in enumerate(far):
            band = '30_50' if gd[j] < 50 else '50_100'
            cov = {n: bool(len(ious[n]) and ious[n][:, jj].max() >= 0.5) for n in M}
            cover[band]['GT']['n'] += 1
            for n in M:
                cover[band][n]['covered'] += cov[n]
            g = gids[j]
            gc = np.array([gp[j].centroid.x, gp[j].centroid.y])
            # Object speed from the label's position in the previous frame (odometry frame).
            spd = None
            if g and i > 0:
                po = {o['gid']: o['pos'] for o in truth[s]['frames'][i - 1]['objects']}
                pn = {o['gid']: o['pos'] for o in truth[s]['frames'][i]['objects']}
                if g in po and g in pn:
                    spd = float(np.linalg.norm(np.array(pn[g])[:2] - np.array(po[g])[:2]) / 0.1)
            sb = 'unknown' if spd is None else ('under 3 m/s' if spd < 3 else '3 to 10 m/s' if spd < 10 else 'over 10 m/s')
            speed_cover[sb]['GT']['n'] += 1
            for n in ('LF@0.2', 'LF@0.3', 'R1', 'R3'):
                speed_cover[sb][n]['covered'] += cov[n]
            for key, cond in (('R1 uncovered', not cov['R1']),
                              ('LF@0.3 covers, R1 does not', cov['LF@0.3'] and not cov['R1'])):
                if not cond:
                    continue
                near = [k for k in range(len(M['R1'][0])) if np.linalg.norm(cent['R1'][k] - gc) < 3.0]
                if near:
                    wm = any(len(M['R1'][1][k][0]) >= 2 for k in near)
                    reason = 'wrong merge' if wm else 'localization (box near, IoU < 0.5)'
                    if not wm and key == 'R1 uncovered':
                        kk = min(near, key=lambda k: np.linalg.norm(cent['R1'][k] - gc))
                        c3, c0 = gt[j][3, :2], gt[j][0, :2]
                        u = (c0 - c3) / np.linalg.norm(c0 - c3)
                        off = cent['R1'][kk] - gc
                        from shapely.affinity import translate
                        shifted = translate(E.polys(M['R1'][0][kk:kk + 1])[0], -off[0], -off[1])
                        un = shifted.union(gp[j]).area
                        shape_iou = shifted.intersection(gp[j]).area / un if un > 0 else 0.0
                        loc_cases.append({'along': float(off @ u), 'across': float(off @ np.array([-u[1], u[0]])),
                                          'shape_iou': float(shape_iou), 'speed': sb, 'band': band, 'sources': M['R1'][1][kk][1]})
                else:
                    d0 = [k for k, x in enumerate(att0) if x == g] if g else []
                    d1 = [k for k, x in enumerate(att1) if x == g] if g else []
                    scores = [sc0[k] for k in d0]
                    if i > 0:
                        scores += [recs_by[(s, i - 1)]['cars'][1]['score'][k] for k in d1]
                    if not scores:
                        reason = 'not detected'
                    elif max(scores) < 0.3:
                        reason = 'below 0.3 threshold'
                    elif cov['R3']:
                        reason = 'fresh-only (coasting kept it)'
                    else:
                        reason = 'no track (confirmation / gating / pruning)'
                reasons[band][key][reason] += 1
    la = np.array([abs(c['along']) for c in loc_cases]) if loc_cases else np.zeros(1)
    lc = np.array([abs(c['across']) for c in loc_cases]) if loc_cases else np.zeros(1)
    res = {'localization_cases': {
               'n': len(loc_cases), 'median_abs_offset_along_m': float(np.median(la)),
               'median_abs_offset_across_m': float(np.median(lc)),
               'share_shape_alone_iou_ge_0.5': float(np.mean([c['shape_iou'] >= 0.5 for c in loc_cases])),
               'by_speed': dict(Counter(c['speed'] for c in loc_cases)),
               'by_number_of_source_cars_in_the_object': dict(Counter(c['sources'] for c in loc_cases))},
           'far_ground_truth_by_speed': {sb: {'n': v['GT']['n'], **{n: v[n]['covered'] for n in ('LF@0.2', 'LF@0.3', 'R1', 'R3')}}
                                         for sb, v in speed_cover.items()},
           'ground_truth_boxes_by_band': {b: cover[b]['GT']['n'] for b in BANDS},
           'covered_by_method': {b: {n: dict(c) for n, c in cover[b].items() if n != 'GT'} for b in BANDS},
           'reasons': {b: {k: dict(v) for k, v in reasons[b].items()} for b in BANDS},
           'boxes_in_band': {b: {n: dict(c) for n, c in boxes[b].items()} for b in BANDS}}
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))



if __name__ == '__main__':
    main()
