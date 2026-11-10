"""Phase D1: the dataset's AP for GodView's fused world, next to the baselines.

Mirrors the V2V4Real evaluation (opencood/utils/eval_utils.py):
* per frame, detections sorted by score are matched greedily to the ground
  truth by bird's-eye IoU of the first four corners; a match removes the
  ground-truth box;
* true and false positives are concatenated **in frame order** (the library
  does not sort them globally by score, so the score only ranks detections
  within a frame) and AP is the VOC 2010 area;
* ranges by the distance of a box's centroid from the ego car.
The overall numbers without exclusions are also computed with the library's
own functions and must agree with this implementation (asserted).

Methods, all in car 0's lidar frame against `gt_corners_ego` of the export:
* nofusion: car 0's exported detections;
* late: both cars' exported detections, car 1 projected with T_to_ego, the
  library's rotated NMS (0.15) and range mask;
* godview (with --dumps): car 0's world from `v2v4real-replay --dump`. Centre
  through the shared anchor (exact) and car 0's lidar_pose, size and heading
  from the fused box, the library's range mask. Score: class_prob (primary)
  or p_exist.

Second number, labeled "excluding first 3 frames": each ground-truth label's
first 3 frames in the ground truth of its scenario are ignored (not counted
as ground truth; a detection matched to one is neither true nor false
positive). Labels are identified by matching each ground-truth box centre to
the per-car label centres in truth.json (0.2 m).

Usage (python -I):
  python -I ap_eval.py --opencood V2V4REAL_REPO --export EXPORT.jsonl.gz \
      --inputs REPLAY_IN_DIR --out OUT.json [--dumps NAME=DUMP_DIR ...] \
      [--skip-scenario NAME]
REPLAY_IN_DIR/<scenario>/truth.json; DUMP_DIR/<scenario>.dump.jsonl.
"""
import argparse
import gzip
import json
import os
import sys

import numpy as np
from shapely.geometry import Polygon

RANGES = {'all': (-np.inf, np.inf), '0_30': (0, 30), '30_50': (30, 50), '50_100': (50, 100)}
TEMPLATE = np.array([[1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, -1],
                     [1, -1, 1], [1, 1, 1], [-1, 1, 1], [-1, -1, 1]], float) / 2
GT_RANGE = (100.0, 40.0)
FIRST = 3


def polys(corners):
    return [Polygon([(b[i, 0], b[i, 1]) for i in range(4)]) for b in corners]


def iou(a, b):
    u = a.union(b).area
    return a.intersection(b).area / u if u > 0 else 0.0


def in_range_mask(corners):
    if not len(corners):
        return np.zeros(0, bool)
    xy = corners[:, :, :2]
    return np.all((np.abs(xy[:, :, 0]) <= GT_RANGE[0]) & (np.abs(xy[:, :, 1]) <= GT_RANGE[1]), axis=1)


def tp_fp(det, score, gt, ignore, thresh, lo, hi, stat):
    """The library's caluclate_tp_fp, plus ignored ground-truth boxes."""
    dp = [(p, s) for p, s in zip(polys(det), score) if lo < np.hypot(p.centroid.x, p.centroid.y) < hi]
    gp = [(p, ig) for p, ig in zip(polys(gt), ignore) if lo < np.hypot(p.centroid.x, p.centroid.y) < hi]
    stat['gt'] += sum(1 for _, ig in gp if not ig)
    for k in np.argsort(-np.array([s for _, s in dp])) if dp else []:
        ious = [iou(dp[k][0], g) for g, _ in gp]
        if not gp or max(ious) < thresh:
            stat['fp'].append(1)
            stat['tp'].append(0)
            continue
        j = int(np.argmax(ious))
        if not gp[j][1]:
            stat['fp'].append(0)
            stat['tp'].append(1)
        gp.pop(j)


def ap(stat):
    tp, fp = np.cumsum(stat['tp']), np.cumsum(stat['fp'])
    if not stat['gt'] or not len(tp):
        return 0.0
    rec = [0.0] + (tp / stat['gt']).tolist() + [1.0]
    pre = [0.0] + (tp / (tp + fp)).tolist() + [0.0]
    for i in range(len(pre) - 2, -1, -1):
        pre[i] = max(pre[i], pre[i + 1])
    return float(sum((rec[i] - rec[i - 1]) * pre[i] for i in range(1, len(rec)) if rec[i] != rec[i - 1]))


def world_boxes(objs, origin, inv_pose, score_key):
    corners, scores = [], []
    for o in objs:
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
    p.add_argument('--skip-scenario', action='append', default=[])
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
    seen = {}
    ignore = {}
    for (s, i) in sorted(gt_ids):
        ig = []
        for g in gt_ids[(s, i)]:
            n = seen.get((s, g), 0) if g is not None else FIRST
            ig.append(n < FIRST)
            if g is not None:
                seen[(s, g)] = n + 1
        ignore[(s, i)] = ig

    methods = ['nofusion', 'late'] + ['%s:%s' % (n, k) for n in dumps for k in ('class_prob', 'p_exist')]
    new = lambda: {'tp': [], 'fp': [], 'gt': 0}  # noqa: E731
    stats = {m: {v: {rg: {th: new() for th in (0.5, 0.7)} for rg in RANGES} for v in ('all_frames', 'excl_first_3')}
             for m in methods}
    lib = {m: {th: new() for th in (0.5, 0.7)} for m in methods}
    n_boxes = {m: 0 for m in methods}
    for r in recs:
        s, i = r['scenario'], fidx[r['scenario']][r['timestamp']]
        assert r['cars'][0]['cav_id'] == '0' and r['cars'][0]['is_ego']
        gt = np.array(r['gt_corners_ego'], float).reshape(-1, 8, 3)
        dets = {}
        c0 = np.array(r['cars'][0]['corners'], float).reshape(-1, 8, 3)
        dets['nofusion'] = (c0, np.array(r['cars'][0]['score'], float))
        pc, ps = [], []
        for car in r['cars']:
            c = np.array(car['corners'], float).reshape(-1, 8, 3)
            T = np.array(car['T_to_ego'])
            if len(c):
                pc.append(c @ T[:3, :3].T + T[:3, 3])
                ps.append(np.array(car['score'], float))
        if pc:
            tc, ts = torch.tensor(np.vstack(pc)).float(), torch.tensor(np.concatenate(ps)).float()
            keep = box_utils.nms_rotated(tc, ts, 0.15)
            tc, ts = tc[keep], ts[keep]
            m = box_utils.get_mask_for_boxes_within_range_torch(tc)
            dets['late'] = (tc[m].numpy().astype(float), ts[m].numpy().astype(float))
        else:
            dets['late'] = (np.zeros((0, 8, 3)), np.zeros(0))
        inv0 = np.linalg.inv(np.array(r['cars'][0]['lidar_pose']))
        origin = np.array(truth[s]['origin_map'])
        for n in dumps:
            fr = dumps[n][s][i]
            assert fr['frame'] == i
            for k in ('class_prob', 'p_exist'):
                dets['%s:%s' % (n, k)] = world_boxes(fr['worlds'][0], origin, inv0, k)
        none = [False] * len(gt)
        for mth, (dc, dsc) in dets.items():
            n_boxes[mth] += len(dc)
            for th in (0.5, 0.7):
                for rg, (lo, hi) in RANGES.items():
                    tp_fp(dc, dsc, gt, none, th, lo, hi, stats[mth]['all_frames'][rg][th])
                    tp_fp(dc, dsc, gt, ignore[(s, i)], th, lo, hi, stats[mth]['excl_first_3'][rg][th])
                eval_utils.caluclate_tp_fp(torch.tensor(dc) if len(dc) else None,
                                           torch.tensor(dsc) if len(dc) else None,
                                           torch.tensor(gt), lib[mth], th)

    out = {'export': os.path.basename(a.export), 'frames': len(recs), 'scenarios': scen,
           'skipped': a.skip_scenario, 'gt_boxes': n_gt, 'gt_boxes_unidentified': unidentified,
           'gt_boxes_ignored_in_excl_first_3': int(sum(sum(v) for v in ignore.values())) - 0,
           'boxes': n_boxes, 'ap': {}, 'library_ap_all_frames': {}}
    out['gt_boxes_ignored_in_excl_first_3'] = int(sum(sum(1 for g, ig in zip(gt_ids[k], ignore[k]) if ig and g is not None)
                                                      for k in ignore))
    for mth in methods:
        out['ap'][mth] = {v: {rg: {'ap_50': ap(stats[mth][v][rg][0.5]), 'ap_70': ap(stats[mth][v][rg][0.7])}
                              for rg in RANGES} for v in stats[mth]}
        l5, _, _ = eval_utils.calculate_ap(lib[mth], 0.5)
        l7, _, _ = eval_utils.calculate_ap(lib[mth], 0.7)
        out['library_ap_all_frames'][mth] = {'ap_50': l5, 'ap_70': l7}
        mine = out['ap'][mth]['all_frames']['all']
        assert abs(mine['ap_50'] - l5) < 1e-9 and abs(mine['ap_70'] - l7) < 1e-9, (mth, mine, l5, l7)
    json.dump(out, open(a.out, 'w'), indent=1)
    for mth in methods:
        A = out['ap'][mth]
        print('%-28s all frames %.2f / %.2f   excl first 3 %.2f / %.2f   boxes %d' % (
            mth, 100 * A['all_frames']['all']['ap_50'], 100 * A['all_frames']['all']['ap_70'],
            100 * A['excl_first_3']['all']['ap_50'], 100 * A['excl_first_3']['all']['ap_70'], n_boxes[mth]))
    print('gt boxes %d, unidentified %d, ignored %d' % (n_gt, unidentified, out['gt_boxes_ignored_in_excl_first_3']))


if __name__ == '__main__':
    main()
