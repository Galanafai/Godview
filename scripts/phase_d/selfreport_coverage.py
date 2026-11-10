"""Phase D self reports (explanation only): which ground-truth boxes are covered.

For car 0's frame and the dataset's ground truth, the share of ground-truth
boxes covered (bird's-eye IoU >= 0.5) by a box of the world, split into the
recording cars (a ground-truth box within 4 m of a car's own position in the
odometry frame, `cav` in truth.json) and all others; plus, for the two recording
cars' boxes, the median centre offset between the world's nearest box and the
ground-truth box.

Usage (python -I): selfreport_coverage.py OPENCOOD EXPORT INPUTS OUT.json NAME=DUMP_DIR ...
"""
import gzip
import json
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ap_eval as E  # noqa: E402


def main():
    opencood, export, inputs, out = sys.argv[1:5]
    specs = [a.split('=', 1) for a in sys.argv[5:]]
    recs = [json.loads(l) for l in gzip.open(export, 'rt')]
    scen = sorted({r['scenario'] for r in recs})
    truth = {s: json.load(open(os.path.join(inputs, s, 'truth.json'))) for s in scen}
    fidx = {s: {f['frame']: i for i, f in enumerate(truth[s]['frames'])} for s in scen}
    dumps = {n: {s: [json.loads(l) for l in open(os.path.join(d, s + '.dump.jsonl')) if l.strip()] for s in scen}
             for n, d in specs}
    cov = defaultdict(lambda: defaultdict(int))
    off = defaultdict(list)
    for r in recs:
        s, i = r['scenario'], fidx[r['scenario']][r['timestamp']]
        pose0 = np.array(r['cars'][0]['lidar_pose'])
        inv0 = np.linalg.inv(pose0)
        origin = np.array(truth[s]['origin_map'])
        gt = np.array(r['gt_corners_ego'], float).reshape(-1, 8, 3)
        gp = E.polys(gt)
        cand = [(o['gid'], o['cav'], np.array(v)) for o in truth[s]['frames'][i]['objects'] for v in o['per_car_pos'].values()]
        kind = []
        for c in gt.mean(axis=1):
            pm = pose0[:3, :3] @ c + pose0[:3, 3] - origin
            d = [np.linalg.norm(pm - v) for _, _, v in cand]
            k = int(np.argmin(d)) if d else -1
            kind.append('recording car' if (k >= 0 and d[k] <= 0.2 and cand[k][1] is not None) else 'other')
        for n in dumps:
            cb, _ = E.world_boxes(dumps[n][s][i]['worlds'][0], origin, inv0)
            dp = E.polys(cb)
            ious = E.iou_matrix(dp, gp) if len(dp) else np.zeros((0, len(gp)))
            for j in range(len(gt)):
                cov[(n, kind[j])]['gt'] += 1
                cov[(n, kind[j])]['covered'] += bool(len(dp) and ious[:, j].max() >= 0.5)
                if kind[j] == 'recording car' and len(dp):
                    cents = np.array([[p.centroid.x, p.centroid.y] for p in dp])
                    off[n].append(float(np.min(np.linalg.norm(cents - np.array([gp[j].centroid.x, gp[j].centroid.y]), axis=1))))
    res = {'coverage': {'%s | %s' % k: {'gt_boxes': v['gt'], 'covered': v['covered'], 'share': v['covered'] / v['gt']} for k, v in cov.items()},
           'recording_car_boxes_nearest_world_box_offset_m_median_p90': {n: [float(np.median(v)), float(np.percentile(v, 90))] for n, v in off.items()}}
    json.dump(res, open(out, 'w'), indent=1)
    for k, v in res['coverage'].items():
        print('%-48s %5d boxes, covered %.3f' % (k, v['gt_boxes'], v['share']))
    for n, v in res['recording_car_boxes_nearest_world_box_offset_m_median_p90'].items():
        print('%-28s recording-car boxes: nearest world box offset median %.2f p90 %.2f m' % (n, v[0], v[1]))


if __name__ == '__main__':
    main()
