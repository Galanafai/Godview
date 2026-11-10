"""Phase D (report only): each car's ego view scored against that car's own labels.

Metric as stated in `docs/results/phase_d_d1_plan.md`, "Own-frame truth check on
V2V4Real val": truth = the car's own labels (its lidar frame, within x 100 / y 40 m),
the car's own box excluded from the boxes (self object, and objects attributed to the
label of its body; for baselines the detections attributed to it, before NMS); AP
(dataset-style and standard) at IoU 0.5 / 0.7 with both cars pooled; and, of the
labels the car's own detector did not detect, the share covered at IoU 0.5.

Usage (python -I): own_frame_eval.py OPENCOOD EXPORT RAW_VAL_DIR INPUTS OUT.json NAME=DUMP_DIR ...
"""
import gzip
import json
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'v2v4real'))
import ap_eval as E  # noqa: E402
import v2v4real_io as vio  # noqa: E402


def label_corners(v):
    a = np.radians(v['angle'][1])
    R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    return (E.TEMPLATE * [2 * e for e in v['extent']]) @ R.T + np.array(v['location'], float)


def main():
    opencood, export, raw, inputs, out = sys.argv[1:6]
    specs = [a.split('=', 1) for a in sys.argv[6:]]
    sys.path.insert(0, opencood)
    import torch
    from opencood.utils import box_utils
    recs = [json.loads(l) for l in gzip.open(export, 'rt')]
    scen = sorted({r['scenario'] for r in recs})
    truth = {s: json.load(open(os.path.join(inputs, s, 'truth.json'))) for s in scen}
    attr = {s: json.load(open(os.path.join(inputs, s, 'attribution.json')))['frames'] for s in scen}
    fidx = {s: {f['frame']: i for i, f in enumerate(truth[s]['frames'])} for s in scen}
    dumps = {n: {s: {json.loads(l)['frame']: json.loads(l) for l in open(os.path.join(d, s + '.dump.jsonl')) if l.strip()}
                 for s in scen} for n, d in specs}
    methods = [n for n, _ in specs] + ['nofusion', 'late']
    TH = (0.5, 0.7)
    st = {m: {th: {'dataset': E.new_dataset_stat(), 'standard': E.new_standard_stat()} for th in TH} for m in methods}
    sharing = {m: {'labels': 0, 'covered': 0} for m in methods}
    n_excluded = defaultdict(int)
    for r in sorted(recs, key=lambda r: (r['scenario'], r['timestamp'])):
        s, i = r['scenario'], fidx[r['scenario']][r['timestamp']]
        T = truth[s]
        origin = np.array(T['origin_map'])
        objs_by_gid = {o['gid']: o for o in T['frames'][i]['objects']}
        poses = [np.array(c['lidar_pose']) for c in r['cars']]
        for c in (0, 1):
            o_car = 1 - c
            labs = vio.load_yaml(os.path.join(raw, s, str(c), r['timestamp'] + '.yaml'))['vehicles'] or {}
            corners, gids, undetected = [], [], []
            for v in labs.values():
                cc = label_corners(v)
                if not E.in_range_mask(cc[None])[0]:
                    continue
                pm = poses[c][:3, :3] @ np.array(v['location']) + poses[c][:3, 3] - origin
                g = None
                for o in T['frames'][i]['objects']:
                    p = o['per_car_pos'].get(str(c))
                    if p is not None and np.linalg.norm(np.array(p) - pm) < 0.2:
                        g = o['gid']
                        break
                corners.append(cc)
                gids.append(g)
                undetected.append(g is not None and c not in objs_by_gid[g]['detected_by'])
            gt = np.array(corners).reshape(-1, 8, 3)
            gp = E.polys(gt)
            gd = E.dists(gp)
            none = [False] * len(gt)
            own = {g for g, o in objs_by_gid.items() if o['cav'] == c}
            inv_c = np.linalg.inv(poses[c])
            dets = {}
            for n, _ in specs:
                fr = dumps[n][s][i]
                keep = []
                for o in fr['worlds'][c]:
                    if o.get('self') and o['id'].startswith('%d:' % c):
                        n_excluded[n] += 1
                        continue
                    if own & set(o.get('labels') or []):
                        n_excluded[n] += 1
                        continue
                    keep.append(o)
                dets[n] = E.world_boxes(keep, origin, inv_c)
            # baselines
            def car_boxes(k, drop_own):
                cs = np.array(r['cars'][k]['corners'], float).reshape(-1, 8, 3)
                sc = np.array(r['cars'][k]['score'], float)
                if drop_own:
                    a = attr[s][i][k]
                    m = np.array([x not in own for x in a], bool) if len(a) == len(sc) else np.ones(len(sc), bool)
                    cs, sc = cs[m], sc[m]
                return cs, sc
            nf = car_boxes(c, False)
            nfm = E.in_range_mask(nf[0])
            dets['nofusion'] = (nf[0][nfm], nf[1][nfm])
            pc, ps = [nf[0]], [nf[1]]
            co, so = car_boxes(o_car, True)
            if len(co):
                T_oc = inv_c @ poses[o_car]
                pc.append(co @ T_oc[:3, :3].T + T_oc[:3, 3])
                ps.append(so)
            pc, ps = np.vstack([x for x in pc if len(x)]), np.concatenate([x for x in ps if len(x)])
            if len(pc):
                tc, ts = torch.tensor(pc).float(), torch.tensor(ps).float()
                kp = box_utils.nms_rotated(tc, ts, 0.15)
                tc, ts = tc[kp], ts[kp]
                mk = box_utils.get_mask_for_boxes_within_range_torch(tc)
                dets['late'] = (tc[mk].numpy().astype(float), ts[mk].numpy().astype(float))
            else:
                dets['late'] = (np.zeros((0, 8, 3)), np.zeros(0))
            for m in methods:
                dc, dsc = dets[m]
                dp = E.polys(dc)
                ious = E.iou_matrix(dp, gp) if len(dp) else np.zeros((0, len(gp)))
                ddist = E.dists(dp) if len(dp) else np.zeros(0)
                for th in TH:
                    E.dataset_match(ious, dsc, ddist, gd, none, th, -np.inf, np.inf, st[m][th]['dataset'])
                    E.standard_match(ious, dsc, ddist, gd, none, th, -np.inf, np.inf, st[m][th]['standard'])
                for j, u in enumerate(undetected):
                    if u:
                        sharing[m]['labels'] += 1
                        sharing[m]['covered'] += bool(len(dp) and ious[:, j].max() >= 0.5)
    res = {'definition': 'docs/results/phase_d_d1_plan.md, Own-frame truth check', 'ap': {}, 'sharing': {}, 'boxes_excluded_as_own': dict(n_excluded)}
    for m in methods:
        res['ap'][m] = {'ap_50_70': {d: [E.dataset_ap(st[m][th][d]) if d == 'dataset' else E.standard_ap(st[m][th][d]) for th in TH] for d in ('dataset', 'standard')}}
        res['sharing'][m] = {**sharing[m], 'share': sharing[m]['covered'] / max(1, sharing[m]['labels'])}
        a = res['ap'][m]['ap_50_70']
        print('%-18s dataset %.2f / %.2f  standard %.2f / %.2f   labels the car did not detect: %d, covered %.1f%%' % (
            m, 100 * a['dataset'][0], 100 * a['dataset'][1], 100 * a['standard'][0], 100 * a['standard'][1],
            sharing[m]['labels'], 100 * res['sharing'][m]['share']))
    json.dump(res, open(out, 'w'), indent=1)


if __name__ == '__main__':
    main()
