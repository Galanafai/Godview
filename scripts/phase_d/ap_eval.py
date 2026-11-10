"""Phase D1: two APs for GodView's fused world and the baselines.

Both are computed from the same bird's-eye IoU (first four corners, as the
dataset does), in car 0's lidar frame against `gt_corners_ego` of the export.

* **dataset AP** mirrors V2V4Real's evaluation (opencood/utils/eval_utils.py):
  per frame, detections sorted by score are matched greedily to the ground
  truth by IoU and a match removes that ground-truth box (the next detection
  may take another box); true and false positives are concatenated in frame
  order, NOT sorted by score across frames; VOC 2010 area. The library's own
  functions are run alongside and must agree (`--check-library`).
* **standard AP** (VOC 2010, all-point interpolation): detections of all
  frames pooled and ranked by score; per frame a detection is true if the
  ground-truth box with the highest IoU has IoU >= threshold and is not yet
  matched, else false (no fall-back to a second box, as in VOC). Ties in score
  keep frame order (stable sort).

Ranges are by the distance of a box centroid from the ego car (as the
dataset). Second number "excl first 3": each ground-truth label's first 3
frames are ignored (not ground truth; a detection matched to one is neither
true nor false). Labels are identified by matching ground-truth centres to
the per-car label centres of truth.json (0.2 m).

Methods: nofusion@T and late@T (the export's detections with score >= T, T =
0.2 is the published threshold; Late Fusion projects both cars, joint rotated
NMS 0.15 and range mask), and GodView worlds from `v2v4real-replay --dump`
(score: class_prob).

Usage (python -I):
  python -I ap_eval.py --opencood V2V4REAL_REPO --export EXPORT.jsonl.gz \
      --inputs REPLAY_IN_DIR --out OUT.json [--baselines 0.2,0.3] \
      [--dumps NAME=DUMP_DIR ...] [--skip-scenario NAME] [--check-library]
DUMP_DIR/<scenario>.dump.jsonl; REPLAY_IN_DIR/<scenario>/truth.json.
"""
import argparse
import gzip
import json
import os
import sys

import numpy as np
from shapely.geometry import Polygon

RANGES = {'all': (-np.inf, np.inf), '0_30': (0, 30), '30_50': (30, 50), '50_100': (50, 100)}
THRESHOLDS = (0.5, 0.7)
TEMPLATE = np.array([[1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, -1],
                     [1, -1, 1], [1, 1, 1], [-1, 1, 1], [-1, -1, 1]], float) / 2
GT_RANGE = (100.0, 40.0)
FIRST = 3


def polys(corners):
    return [Polygon([(b[i, 0], b[i, 1]) for i in range(4)]) for b in corners]


def iou_matrix(dp, gp):
    m = np.zeros((len(dp), len(gp)))
    for i, d in enumerate(dp):
        for j, g in enumerate(gp):
            u = d.union(g).area
            m[i, j] = d.intersection(g).area / u if u > 0 else 0.0
    return m


def dists(polygons):
    return np.array([np.hypot(p.centroid.x, p.centroid.y) for p in polygons])


def in_range_mask(corners):
    if not len(corners):
        return np.zeros(0, bool)
    xy = corners[:, :, :2]
    return np.all((np.abs(xy[:, :, 0]) <= GT_RANGE[0]) & (np.abs(xy[:, :, 1]) <= GT_RANGE[1]), axis=1)


def new_dataset_stat():
    return {'tp': [], 'fp': [], 'gt': 0}


def new_standard_stat():
    return {'score': [], 'tp': [], 'gt': 0}


def dataset_match(ious, scores, ddist, gdist, ignore, th, lo, hi, stat):
    """The library's caluclate_tp_fp on a precomputed IoU matrix."""
    di = [i for i in range(len(scores)) if lo < ddist[i] < hi]
    avail = [j for j in range(len(gdist)) if lo < gdist[j] < hi]
    stat['gt'] += sum(1 for j in avail if not ignore[j])
    if not di:
        return
    sc = np.array([scores[i] for i in di])
    for k in np.argsort(-sc):
        row = ious[di[k], avail] if avail else np.zeros(0)
        if not avail or np.max(row) < th:
            stat['fp'].append(1)
            stat['tp'].append(0)
            continue
        j = int(np.argmax(row))
        if not ignore[avail[j]]:
            stat['fp'].append(0)
            stat['tp'].append(1)
        avail.pop(j)


def standard_match(ious, scores, ddist, gdist, ignore, th, lo, hi, stat):
    """VOC matching: highest-IoU ground truth, taken once, no fall-back."""
    di = [i for i in range(len(scores)) if lo < ddist[i] < hi]
    gi = [j for j in range(len(gdist)) if lo < gdist[j] < hi]
    stat['gt'] += sum(1 for j in gi if not ignore[j])
    taken = set()
    order = sorted(di, key=lambda i: -scores[i])
    for i in order:
        if not gi:
            stat['score'].append(scores[i])
            stat['tp'].append(0)
            continue
        row = ious[i, gi]
        j = gi[int(np.argmax(row))]
        if ious[i, j] >= th:
            if ignore[j]:
                continue
            if j not in taken:
                taken.add(j)
                stat['score'].append(scores[i])
                stat['tp'].append(1)
                continue
        stat['score'].append(scores[i])
        stat['tp'].append(0)


def voc_area(tp, fp, n_gt):
    """VOC 2010 area from cumulative counts in the given order."""
    if not n_gt or not len(tp):
        return 0.0
    tp_c, fp_c = np.cumsum(tp), np.cumsum(fp)
    rec = [0.0] + (tp_c / n_gt).tolist() + [1.0]
    pre = [0.0] + (tp_c / (tp_c + fp_c)).tolist() + [0.0]
    for i in range(len(pre) - 2, -1, -1):
        pre[i] = max(pre[i], pre[i + 1])
    return float(sum((rec[i] - rec[i - 1]) * pre[i] for i in range(1, len(rec)) if rec[i] != rec[i - 1]))


def dataset_ap(stat):
    return voc_area(stat['tp'], stat['fp'], stat['gt'])


def standard_ap(stat):
    if not len(stat['tp']):
        return 0.0
    s = np.array(stat['score'])
    order = np.argsort(-s, kind='stable')
    tp = np.array(stat['tp'])[order]
    return voc_area(tp, 1 - tp, stat['gt'])


def world_boxes(objs, origin, inv_pose, score_key='class_prob', exclude_self=False):
    corners, scores = [], []
    for o in objs:
        if exclude_self and o.get('self'):
            continue
        b = o.get('box')
        if b is None:
            continue
        a = np.radians(b['yaw_deg'])
        R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        c = (TEMPLATE * [b['l'], b['w'], b['h']]) @ R.T + [o['x'], o['y'], o['z']] + origin
        corners.append(c @ inv_pose[:3, :3].T + inv_pose[:3, 3])
        scores.append(o[score_key])
    corners = np.array(corners).reshape(-1, 8, 3)
    m = in_range_mask(corners)
    return corners[m], np.array(scores)[m]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--opencood', required=True)
    p.add_argument('--export', required=True)
    p.add_argument('--inputs', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--dumps', action='append', default=[], help='NAME=DIR')
    p.add_argument('--baselines', default='', help='comma list of score thresholds for No Fusion and Late Fusion')
    p.add_argument('--skip-scenario', action='append', default=[])
    p.add_argument('--check-library', action='store_true')
    p.add_argument('--exclude-self', action='append', default=[], help='dump NAMEs scored without the self objects (explanation only)')
    a = p.parse_args()
    sys.path.insert(0, a.opencood)
    import torch
    from opencood.utils import box_utils, eval_utils

    recs = [json.loads(l) for l in gzip.open(a.export, 'rt')]
    recs = [r for r in recs if r['scenario'] not in a.skip_scenario]
    scen = sorted({r['scenario'] for r in recs})
    truth = {s: json.load(open(os.path.join(a.inputs, s, 'truth.json'))) for s in scen}
    fidx = {s: {f['frame']: i for i, f in enumerate(truth[s]['frames'])} for s in scen}
    dumps = {}
    for d in a.dumps:
        name, path = d.split('=', 1)
        dumps[name] = {s: [json.loads(l) for l in open(os.path.join(path, s + '.dump.jsonl')) if l.strip()]
                       for s in scen}
    bths = [float(x) for x in a.baselines.split(',') if x]

    # Identify ground-truth boxes, then each label's first frames per scenario.
    gt_ids, unidentified, n_gt = {}, 0, 0
    for r in recs:
        s, i = r['scenario'], fidx[r['scenario']][r['timestamp']]
        pose0 = np.array(r['cars'][0]['lidar_pose'])
        origin = np.array(truth[s]['origin_map'])
        gt = np.array(r['gt_corners_ego'], float).reshape(-1, 8, 3)
        cand = [(o['gid'], np.array(v)) for o in truth[s]['frames'][i]['objects'] for v in o['per_car_pos'].values()]
        ids = []
        for c in gt.mean(axis=1):
            pm = pose0[:3, :3] @ c + pose0[:3, 3] - origin
            d = [np.linalg.norm(pm - v) for _, v in cand]
            k = int(np.argmin(d)) if d else -1
            ids.append(cand[k][0] if k >= 0 and d[k] <= 0.2 else None)
        n_gt += len(ids)
        unidentified += sum(x is None for x in ids)
        gt_ids[(s, i)] = ids
    seen, ignore = {}, {}
    for (s, i) in sorted(gt_ids):
        ig = []
        for g in gt_ids[(s, i)]:
            n = seen.get((s, g), 0) if g is not None else FIRST
            ig.append(n < FIRST)
            if g is not None:
                seen[(s, g)] = n + 1
        ignore[(s, i)] = ig

    methods = [f'{b}@{t}' for t in bths for b in ('nofusion', 'late')] + list(dumps)
    stats = {m: {v: {rg: {th: {'dataset': new_dataset_stat(), 'standard': new_standard_stat()}
                          for th in THRESHOLDS} for rg in RANGES} for v in ('all_frames', 'excl_first_3')}
             for m in methods}
    lib = {m: {th: {'tp': [], 'fp': [], 'gt': 0} for th in THRESHOLDS} for m in methods}
    n_boxes = {m: 0 for m in methods}
    for r in recs:
        s, i = r['scenario'], fidx[r['scenario']][r['timestamp']]
        assert r['cars'][0]['cav_id'] == '0' and r['cars'][0]['is_ego']
        gt = np.array(r['gt_corners_ego'], float).reshape(-1, 8, 3)
        gp = polys(gt)
        gdist = dists(gp)
        dets = {}
        for t in bths:
            c0 = np.array(r['cars'][0]['corners'], float).reshape(-1, 8, 3)
            s0 = np.array(r['cars'][0]['score'], float)
            k0 = s0 >= t
            dets[f'nofusion@{t}'] = (c0[k0], s0[k0])
            pc, ps = [], []
            for car in r['cars']:
                c = np.array(car['corners'], float).reshape(-1, 8, 3)
                sc_ = np.array(car['score'], float)
                keep = sc_ >= t
                T = np.array(car['T_to_ego'])
                if keep.any():
                    pc.append(c[keep] @ T[:3, :3].T + T[:3, 3])
                    ps.append(sc_[keep])
            if pc:
                tc, ts = torch.tensor(np.vstack(pc)).float(), torch.tensor(np.concatenate(ps)).float()
                kp = box_utils.nms_rotated(tc, ts, 0.15)
                tc, ts = tc[kp], ts[kp]
                m = box_utils.get_mask_for_boxes_within_range_torch(tc)
                dets[f'late@{t}'] = (tc[m].numpy().astype(float), ts[m].numpy().astype(float))
            else:
                dets[f'late@{t}'] = (np.zeros((0, 8, 3)), np.zeros(0))
        inv0 = np.linalg.inv(np.array(r['cars'][0]['lidar_pose']))
        origin = np.array(truth[s]['origin_map'])
        for n in dumps:
            fr = dumps[n][s][i]
            assert fr['frame'] == i
            dets[n] = world_boxes(fr['worlds'][0], origin, inv0, exclude_self=n in a.exclude_self)
        none = [False] * len(gt)
        for mth, (dc, dsc) in dets.items():
            n_boxes[mth] += len(dc)
            dp = polys(dc)
            ious = iou_matrix(dp, gp)
            ddist = dists(dp)
            for th in THRESHOLDS:
                for rg, (lo, hi) in RANGES.items():
                    for v, ig in (('all_frames', none), ('excl_first_3', ignore[(s, i)])):
                        st = stats[mth][v][rg][th]
                        dataset_match(ious, dsc, ddist, gdist, ig, th, lo, hi, st['dataset'])
                        standard_match(ious, dsc, ddist, gdist, ig, th, lo, hi, st['standard'])
                if a.check_library:
                    eval_utils.caluclate_tp_fp(torch.tensor(dc) if len(dc) else None,
                                               torch.tensor(dsc) if len(dc) else None,
                                               torch.tensor(gt), lib[mth], th)

    out = {'export': os.path.basename(a.export), 'frames': len(recs), 'scenarios': scen,
           'skipped': a.skip_scenario, 'gt_boxes': n_gt, 'gt_boxes_unidentified': unidentified,
           'gt_boxes_ignored_in_excl_first_3': int(sum(sum(1 for g, ig in zip(gt_ids[k], ignore[k])
                                                           if ig and g is not None) for k in ignore)),
           'boxes': n_boxes, 'ap': {}, 'positives': {}}
    for mth in methods:
        out['ap'][mth] = {}
        for v in ('all_frames', 'excl_first_3'):
            out['ap'][mth][v] = {rg: {d: {'ap_50': (dataset_ap if d == 'dataset' else standard_ap)(stats[mth][v][rg][0.5][d]),
                                          'ap_70': (dataset_ap if d == 'dataset' else standard_ap)(stats[mth][v][rg][0.7][d])}
                                      for d in ('dataset', 'standard')} for rg in RANGES}
        st5 = stats[mth]['all_frames']['all'][0.5]['standard']
        out['positives'][mth] = {'gt': st5['gt'], 'boxes_in_range': len(st5['tp']), 'tp_at_0.5': int(sum(st5['tp']))}
        if a.check_library:
            for th, key in ((0.5, 'ap_50'), (0.7, 'ap_70')):
                l, _, _ = eval_utils.calculate_ap(lib[mth], th)
                mine = out['ap'][mth]['all_frames']['all']['dataset'][key]
                assert abs(mine - l) < 1e-9, (mth, key, mine, l)
    json.dump(out, open(a.out, 'w'), indent=1)
    for mth in methods:
        A = out['ap'][mth]['all_frames']['all']
        print('%-26s dataset %.2f / %.2f   standard %.2f / %.2f   boxes %d' % (
            mth, 100 * A['dataset']['ap_50'], 100 * A['dataset']['ap_70'],
            100 * A['standard']['ap_50'], 100 * A['standard']['ap_70'], n_boxes[mth]))
    print('gt boxes %d, unidentified %d' % (n_gt, unidentified))


if __name__ == '__main__':
    main()
