"""Phase D2 step 3: score the V2X-Real replay (labels plus noise).

Scoring rules as committed in docs/PHASE_D_HANDOFF.md ("D2 steps 2 and 3"): truth is the
shared map-frame labels (union by id); each agent's own box is excluded from its own view's
scoring (a vehicle's body label is removed from its truth, and from its boxes its self
object and any object attributed to that label); truth and boxes within 70 m (horizontal)
of the agent; the shared world is scored once per agent with the same truth, region and
exclusion; boxes match only labels of their class. AP at IoU 0.3 and 0.5 (bird's-eye),
dataset-style and standard, class-pooled and per class, for the vehicle views pooled, the
roadside-unit views pooled and each agent; wrong merges (ego views: objects whose members
are attributed to two or more labels) and wrong merges involving an agent body; the share
of vehicle-view frames with an offset estimate. Per scenario, in aggregate, and in
aggregate without `2023-03-17-16-03-02_11_1`.

Usage (python3, needs numpy, shapely): v2x_eval.py INPUTS_DIR OUT.json NAME=DUMP_DIR ...
DUMP_DIR holds <scenario>.dump.jsonl written by `v2x-replay`.
"""
import json
import os
import sys
from collections import defaultdict

import numpy as np
from shapely.geometry import Polygon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ap_eval as E  # noqa: E402

RANGE = 70.0
TH = (0.3, 0.5)
NO_ANCHOR_SCEN = '2023-03-17-16-03-02_11_1'


def rect(x, y, l, w, yaw_deg):
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    pts = [(l / 2, w / 2), (-l / 2, w / 2), (-l / 2, -w / 2), (l / 2, -w / 2)]
    return Polygon([(x + c * u - s * v, y + s * u + c * v) for u, v in pts])


def fast_iou(bp, gp):
    """IoU matrix computed only for pairs whose centres are closer than the sum of the boxes'
    half-diagonals (any other pair has IoU 0); equal to ap_eval.iou_matrix otherwise."""
    m = np.zeros((len(bp), len(gp)))
    if not bp or not gp:
        return m
    bc = np.array([[p.centroid.x, p.centroid.y] for p in bp])
    gc = np.array([[p.centroid.x, p.centroid.y] for p in gp])
    br = np.array([np.hypot(p.bounds[2] - p.bounds[0], p.bounds[3] - p.bounds[1]) / 2 for p in bp])
    gr = np.array([np.hypot(p.bounds[2] - p.bounds[0], p.bounds[3] - p.bounds[1]) / 2 for p in gp])
    d = np.linalg.norm(bc[:, None, :] - gc[None, :, :], axis=2)
    for i, j in zip(*np.where(d <= br[:, None] + gr[None, :])):
        u = bp[i].union(gp[j]).area
        m[i, j] = bp[i].intersection(gp[j]).area / u if u > 0 else 0.0
    return m


def merge_dataset(into, tmp):
    into['tp'] += tmp['tp']
    into['fp'] += tmp['fp']
    into['gt'] += tmp['gt']


def merge_standard(into, tmp):
    into['score'] += tmp['score']
    into['tp'] += tmp['tp']
    into['gt'] += tmp['gt']


def main():
    inputs, out = sys.argv[1:3]
    specs = [a.split('=', 1) for a in sys.argv[3:]]
    scens = sorted(os.listdir(inputs))
    truth = {s: json.load(open(os.path.join(inputs, s, 'truth.json'))) for s in scens}
    from multiprocessing import Pool
    with Pool(len(specs)) as pool:
        res = dict(pool.map(eval_variant, [(name, d, scens, truth) for name, d in specs]))
    json.dump(res, open(out, 'w'), indent=1)
    print_summary(res)


def eval_variant(arg):
    name, d, scens, truth = arg
    stats = defaultdict(lambda: {'dataset': E.new_dataset_stat(), 'standard': E.new_standard_stat()})
    wm = defaultdict(int)
    est = defaultdict(int)
    for s in scens:
        T = truth[s]
        kinds = [a['kind'] for a in T['agents']]
        groups_of = {'vehicle': [], 'rsu': []}
        for k, kd in enumerate(kinds):
            groups_of[kd].append(k)
        lines = [json.loads(l) for l in open(os.path.join(d, s + '.dump.jsonl')) if l.strip()]
        scope = [s, 'all'] + ([] if s == NO_ANCHOR_SCEN else ['without_11_1'])
        for fr in lines:
            i = fr['frame']
            tf = T['frames'][i]
            objs = [o for o in tf['objects'] if o['class_id'] is not None]
            bodies = {o['gid'] for o in tf['objects'] if o['agent'] is not None}
            polys_gt = {o['gid']: rect(o['pos'][0], o['pos'][1], o['size'][0], o['size'][1], o['yaw_enu_deg']) for o in objs}
            for ag in fr['agents']:
                a = ag['index']
                kind = kinds[a]
                pos_a = np.array(tf['ego_odom'][a][:2])
                own_body = {o['gid'] for o in tf['objects'] if o['agent'] == a}
                gt = [o for o in objs if o['gid'] not in own_body and np.linalg.norm(np.array(o['pos'][:2]) - pos_a) <= RANGE]
                gp = [polys_gt[o['gid']] for o in gt]
                gd = np.array([np.linalg.norm(np.array(o['pos'][:2]) - pos_a) for o in gt])
                for view in ('ego', 'shared'):
                    boxes = []
                    for o in ag[view]:
                        b = o['box']
                        if b is None or np.hypot(o['x'] - pos_a[0], o['y'] - pos_a[1]) > RANGE:
                            continue
                        if o['self_agent'] == a or (own_body & set(o['labels'])):
                            continue
                        boxes.append(o)
                    if view == 'ego':
                        for o in ag['ego']:
                            labs = set(o['labels'])
                            if len(labs) >= 2:
                                for sc in scope:
                                    wm[(sc, 'wrong_merges')] += 1
                                    wm[(sc, 'wrong_merges_involving_an_agent')] += bool(labs & bodies)
                                    wm[(sc, kind, 'wrong_merges')] += 1
                                    wm[(sc, kind, 'wrong_merges_involving_an_agent')] += bool(labs & bodies)
                        if kind == 'vehicle':
                            others = [k for k in range(len(kinds)) if k != a]
                            shares = any(a in o['detected_by'] and any(k in o['detected_by'] for k in others)
                                         for o in tf['objects'] if o['gid'] not in own_body)
                            has = any(x['src'] in others for x in ag['alignment'])
                            for sc in scope:
                                est[(sc, 'vehicle_frames_sharing')] += shares
                                est[(sc, 'vehicle_frames_sharing_with_estimate')] += (shares and has)
                    bp = [rect(o['x'], o['y'], o['box']['l'], o['box']['w'], o['box']['yaw_deg']) for o in boxes]
                    bd = np.array([np.hypot(o['x'] - pos_a[0], o['y'] - pos_a[1]) for o in boxes])
                    sc_ = np.array([o['class_prob'] for o in boxes])
                    for cls in (1, 2, 3):
                        bi = [j for j, o in enumerate(boxes) if o['class_id'] == cls]
                        gi = [j for j, o in enumerate(gt) if o['class_id'] == cls]
                        ious = fast_iou([bp[j] for j in bi], [gp[j] for j in gi])
                        for th in TH:
                            td, ts = E.new_dataset_stat(), E.new_standard_stat()
                            none = [False] * len(gi)
                            E.dataset_match(ious, sc_[bi] if bi else np.zeros(0), bd[bi] if bi else np.zeros(0), gd[gi] if gi else np.zeros(0), none, th, -np.inf, np.inf, td)
                            E.standard_match(ious, sc_[bi] if bi else np.zeros(0), bd[bi] if bi else np.zeros(0), gd[gi] if gi else np.zeros(0), none, th, -np.inf, np.inf, ts)
                            for sc in scope:
                                for grp in (kind, 'agent%d' % a):
                                    for c_ in (cls, 'all'):
                                        k_ = (sc, view, grp, c_, th)
                                        merge_dataset(stats[k_]['dataset'], td)
                                        merge_standard(stats[k_]['standard'], ts)
    r = {'ap': {}, 'wrong_merges': {'|'.join(map(str, k)): v for k, v in wm.items()},
         'estimates': {'|'.join(map(str, k)): v for k, v in est.items()}}
    for k, v in stats.items():
        r['ap']['|'.join(map(str, k))] = {'dataset': E.dataset_ap(v['dataset']), 'standard': E.standard_ap(v['standard']),
                                          'gt': v['dataset']['gt'], 'boxes': len(v['dataset']['tp'])}

    return name, r


def print_summary(res):
    for name, r in res.items():
        def g(sc, view, grp, cls, th):
            v = r['ap'].get('|'.join(map(str, (sc, view, grp, cls, th))))
            return ('%.1f' % (100 * v['dataset'])) if v else 'n/a'
        print('%-22s vehicle views ego %s shared %s | rsu views ego %s shared %s (class-pooled dataset AP@0.5, all scenarios) | wrong merges %d (involving an agent %d)' % (
            name, g('all', 'ego', 'vehicle', 'all', 0.5), g('all', 'shared', 'vehicle', 'all', 0.5), g('all', 'ego', 'rsu', 'all', 0.5),
            g('all', 'shared', 'rsu', 'all', 0.5), r['wrong_merges'].get('all|wrong_merges', 0), r['wrong_merges'].get('all|wrong_merges_involving_an_agent', 0)))

if __name__ == '__main__':
    main()
