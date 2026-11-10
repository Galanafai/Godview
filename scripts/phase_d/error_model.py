"""Phase D1 step 2: error model of the PointPillars detector, from its export.

Matches each car's exported detections to that car's own labels (both in the
car's lidar frame) and measures, for train and val only:

* position error against range, in the car frame and along / across the
  labeled box;
* size error (length, width, height);
* yaw error, with front/back flips (|error| > 90 degrees) counted separately;
* misses (labels with at least --min-points of the car's own lidar points and
  no detection) and false positives (detections with no label), the latter
  split by whether the other car labeled something there;
* score calibration: the share of detections that match a label, by score.

Each error is split into a persistent part per object (the mean over the
object's matched frames: what GodView must declare as systematic) and a
per-scan part (the deviation from that mean: the detection noise).

Truth is the labels, which have their own errors (see PHASE_C_HANDOFF.md).

Usage (python -I, needs numpy, scipy, PyYAML):
  python -I error_model.py EXPORT.jsonl.gz DATA_DIR OUT.json [--gate 3.0]
"""
import argparse
import gzip
import json
import os
import sys
from collections import defaultdict
from multiprocessing import Pool

import numpy as np
from scipy.optimize import linear_sum_assignment, minimize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'v2v4real'))
import v2v4real_io as io  # noqa: E402

RANGE_X, RANGE_Y = 100.0, 40.0          # the dataset's evaluation range
RANGE_BINS = [0, 10, 20, 30, 40, 50, 60, 80, 100]
SCORE_BINS = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0]
POINT_BINS = [0, 1, 5, 20, 50, 100, 300, 100000]
MIN_FRAMES_PER_OBJECT = 5


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        fields = None
        while True:
            line = f.readline()
            if not line:
                raise ValueError('no DATA line in %s' % path)
            parts = line.split()
            if parts and parts[0] == b'FIELDS':
                fields = [p.decode() for p in parts[1:]]
            if parts and parts[0] == b'DATA':
                if parts[1] != b'ascii':
                    raise ValueError('only ascii pcd: %s' % path)
                break
        arr = np.array(f.read().split(), dtype=np.float64).reshape(-1, len(fields))
    return arr[:, [fields.index('x'), fields.index('y'), fields.index('z')]]


def points_in_boxes(P, labels):
    out = []
    for l in labels:
        a = np.radians(l['yaw'])
        c, s = np.cos(a), np.sin(a)
        q = P - l['loc']
        x = c * q[:, 0] + s * q[:, 1]
        y = -s * q[:, 0] + c * q[:, 1]
        h = l['half']
        out.append(int(np.sum((np.abs(x) <= h[0]) & (np.abs(y) <= h[1]) & (np.abs(q[:, 2]) <= h[2]))))
    return out


def load_labels(path):
    y = io.load_yaml(path)
    labs = []
    for lid, v in (y.get('vehicles') or {}).items():
        labs.append({'id': int(lid), 'loc': np.asarray(v['location'], float),
                     'yaw': float(v['angle'][1]), 'half': np.asarray(v['extent'], float),
                     'type': str(v.get('obj_type', '?'))})
    return labs, np.asarray(y['lidar_pose'], float)


def process_frame(args):
    """One frame: both cars. Returns matched pairs, misses, false positives."""
    rec, root, gate = args
    cars = {}
    for c in rec['cars']:
        base = os.path.join(root, rec['scenario'], c['cav_id'], rec['timestamp'])
        labs, pose = load_labels(base + '.yaml')
        P = read_pcd_xyz(base + '.pcd')
        pts = points_in_boxes(P, labs)
        for l, n in zip(labs, pts):
            l['points'] = n
        cars[c['cav_id']] = (c, labs, pose)
    out = {'pairs': [], 'misses': [], 'fps': [], 'n_dets': 0, 'n_labels': 0}
    for cid, (c, labs, pose) in cars.items():
        labs = [l for l in labs if abs(l['loc'][0]) <= RANGE_X and abs(l['loc'][1]) <= RANGE_Y]
        dets = np.asarray(c['box'], float).reshape(-1, 7)
        scores = np.asarray(c['score'], float)
        out['n_dets'] += len(dets)
        out['n_labels'] += len(labs)
        L = np.array([l['loc'] for l in labs]).reshape(-1, 3)
        if len(dets) and len(labs):
            D = np.linalg.norm(dets[:, None, :2] - L[None, :, :2], axis=2)
            cost = np.where(D <= gate, D, 1e6)
            r, k = linear_sum_assignment(cost)
            ok = cost[r, k] < 1e6
            r, k = r[ok], k[ok]
        else:
            r = k = np.zeros(0, int)
        md, ml = set(r.tolist()), set(k.tolist())
        key = rec['scenario'] + '/' + cid
        for i, j in zip(r, k):
            d, l = dets[i], labs[j]
            e = d[:3] - l['loc']
            a = np.radians(l['yaw'])
            along = np.cos(a) * e[0] + np.sin(a) * e[1]
            across = -np.sin(a) * e[0] + np.cos(a) * e[1]
            dyaw = float(wrap_deg(np.degrees(d[6]) - l['yaw']))
            out['pairs'].append({
                'obj': key + '/' + str(l['id']), 'type': l['type'], 'score': float(scores[i]),
                'range': float(np.hypot(*l['loc'][:2])), 'points': l['points'],
                'ex': float(e[0]), 'ey': float(e[1]), 'ez': float(e[2]),
                'along': float(along), 'across': float(across),
                'lab_size': (2 * l['half']).tolist(),
                'dsize': (d[3:6] - 2 * l['half']).tolist(),
                'dyaw': dyaw, 'flip': abs(dyaw) > 90.0,
                'det_yaw_deg': float(np.degrees(d[6])), 'lab_yaw_deg': float(wrap_deg(l['yaw'])),
                'dyaw_mod180': float(wrap_deg(2 * dyaw) / 2.0),
            })
        for j, l in enumerate(labs):
            if j not in ml:
                out['misses'].append({'range': float(np.hypot(*l['loc'][:2])),
                                      'x': float(l['loc'][0]), 'y': float(l['loc'][1]),
                                      'points': l['points'], 'type': l['type']})
        # Other car's labels in this car's frame (pose offset of 1 to 4 m is
        # known from Phase C, so a 4 m gate).
        others = []
        for oid, (_, olabs, opose) in cars.items():
            if oid == cid:
                continue
            T = np.linalg.inv(pose) @ opose
            for l in olabs:
                others.append((T @ np.r_[l['loc'], 1.0])[:3])
            others.append(T[:3, 3])                       # the other car itself
        others = np.array(others).reshape(-1, 3)
        for i in range(len(dets)):
            if i in md:
                continue
            near_other = bool(len(others) and
                              np.min(np.linalg.norm(others[:, :2] - dets[i, :2], axis=1)) <= 4.0)
            out['fps'].append({'score': float(scores[i]), 'range': float(np.hypot(*dets[i, :2])),
                               'near_other_car_label': near_other})
    return out


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else None


def pct(x, q):
    return float(np.percentile(np.abs(x), q)) if len(x) else None


def binned(values, keys, bins):
    rows = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (keys >= lo) & (keys < hi)
        rows.append((lo, hi, m))
    return rows


def decompose(pairs, field, fn=lambda p, f: p[f]):
    """Per-object mean (systematic) and within-object deviation (noise)."""
    by = defaultdict(list)
    for p in pairs:
        by[p['obj']].append(fn(p, field))
    means, resid, ns = [], [], []
    for v in by.values():
        if len(v) < MIN_FRAMES_PER_OBJECT:
            continue
        v = np.asarray(v, float)
        means.append(v.mean())
        resid.extend(v - v.mean())
        ns.append(len(v))
    if not means:
        return None
    means, resid, ns = np.asarray(means), np.asarray(resid), np.asarray(ns)
    noise_sd = float(np.sqrt(np.sum(resid ** 2) / max(len(resid) - len(means), 1)))
    # variance of object means minus the part the per-scan noise explains
    sys_var = float(np.mean(means ** 2) - np.mean(noise_sd ** 2 / ns))
    return {'objects': int(len(means)), 'frames': int(ns.sum()),
            'noise_sd': noise_sd,
            'sys_rms_of_means': float(np.sqrt(np.mean(means ** 2))),
            'sys_sd_noise_corrected': float(np.sqrt(max(sys_var, 0.0))),
            'mean_of_means': float(means.mean()),
            'abs_means_p50_p90_max': [float(np.percentile(np.abs(means), 50)),
                                      float(np.percentile(np.abs(means), 90)),
                                      float(np.abs(means).max())]}


def summarize(res, gate):
    pairs = [p for r in res for p in r['pairs']]
    misses = [m for r in res for m in r['misses']]
    fps = [f for r in res for f in r['fps']]
    n_dets = sum(r['n_dets'] for r in res)
    n_labels = sum(r['n_labels'] for r in res)
    S = {'gate_m': gate, 'frames': len(res), 'detections': n_dets, 'labels_in_range': n_labels,
         'matched': len(pairs), 'missed_labels': len(misses), 'false_positives': len(fps)}

    rng = np.array([p['range'] for p in pairs])
    ex, ey, ez = (np.array([p[k] for p in pairs]) for k in ('ex', 'ey', 'ez'))
    al, ac = np.array([p['along'] for p in pairs]), np.array([p['across'] for p in pairs])
    eh = np.hypot(ex, ey)

    rows = []
    for lo, hi, m in binned(None, rng, RANGE_BINS):
        if m.sum() == 0:
            continue
        rows.append({'range_m': [lo, hi], 'n': int(m.sum()),
                     'rms_x': rms(ex[m]), 'rms_y': rms(ey[m]), 'rms_z': rms(ez[m]),
                     'rms_h_per_axis': rms(np.r_[ex[m], ey[m]]),
                     'rms_along': rms(al[m]), 'rms_across': rms(ac[m]),
                     'mean_along': float(al[m].mean()), 'mean_across': float(ac[m].mean()),
                     'mean_z': float(ez[m].mean()),
                     'h_err_p50': pct(eh[m], 50), 'h_err_p95': pct(eh[m], 95)})
    S['position_by_range'] = rows

    # sigma(r) = sqrt(a^2 + (b r)^2) fitted to per-scan horizontal per-axis errors
    def fit(err, r):
        """Gaussian maximum likelihood of sigma(r) = sqrt(a^2 + (b r)^2)."""
        def nll(th):
            v = th[0] ** 2 + (th[1] * r) ** 2
            return float(np.sum(np.log(v) + err ** 2 / v))
        best = min((minimize(nll, [a0, b0], method='L-BFGS-B', bounds=[(1e-3, 5), (0, 1)])
                    for a0 in (0.1, 0.3) for b0 in (0.002, 0.01)), key=lambda o: o.fun)
        return [float(best.x[0]), float(best.x[1])]
    S['sigma_h_fit_a_b'] = fit(np.r_[ex, ey], np.r_[rng, rng])
    S['sigma_z_fit_a_b'] = fit(ez, rng)

    # Per-scan part only: residuals from each object's mean error, along and
    # across the labeled box (the systematic part is declared separately).
    by = defaultdict(list)
    for p in pairs:
        by[p['obj']].append(p)
    ra, rc, rz, rr = [], [], [], []
    for v in by.values():
        if len(v) < MIN_FRAMES_PER_OBJECT:
            continue
        a = np.array([p['along'] for p in v]); c = np.array([p['across'] for p in v])
        z = np.array([p['ez'] for p in v]); r = np.array([p['range'] for p in v])
        ra.extend(a - a.mean()); rc.extend(c - c.mean()); rz.extend(z - z.mean()); rr.extend(r)
    ra, rc, rz, rr = map(np.asarray, (ra, rc, rz, rr))
    S['noise_fit_a_b'] = {'horizontal_per_axis': fit(np.r_[ra, rc], np.r_[rr, rr]),
                          'along': fit(ra, rr), 'across': fit(rc, rr), 'z': fit(rz, rr)}
    S['noise_by_range'] = [{'range_m': [lo, hi], 'n': int(m.sum()), 'rms_along': rms(ra[m]),
                            'rms_across': rms(rc[m]), 'rms_z': rms(rz[m])}
                           for lo, hi, m in binned(None, rr, RANGE_BINS) if m.sum()]

    S['decomposition'] = {
        'along_m': decompose(pairs, 'along'),
        'across_m': decompose(pairs, 'across'),
        'z_m': decompose(pairs, 'ez'),
        'along_frac_of_half_length': decompose(pairs, 'along', lambda p, f: p[f] / (p['lab_size'][0] / 2)),
        'across_frac_of_half_width': decompose(pairs, 'across', lambda p, f: p[f] / (p['lab_size'][1] / 2)),
        'length_m': decompose(pairs, 0, lambda p, i: p['dsize'][i]),
        'width_m': decompose(pairs, 1, lambda p, i: p['dsize'][i]),
        'height_m': decompose(pairs, 2, lambda p, i: p['dsize'][i]),
        'yaw_mod180_deg': decompose([p for p in pairs], 'dyaw_mod180'),
    }

    cars = [p for p in pairs if p['type'] == 'Car']
    S['decomposition_cars_only'] = {
        'along_frac_of_half_length': decompose(cars, 'along', lambda p, f: p[f] / (p['lab_size'][0] / 2)),
        'across_frac_of_half_width': decompose(cars, 'across', lambda p, f: p[f] / (p['lab_size'][1] / 2)),
        'length_m': decompose(cars, 0, lambda p, i: p['dsize'][i]),
        'width_m': decompose(cars, 1, lambda p, i: p['dsize'][i]),
        'height_m': decompose(cars, 2, lambda p, i: p['dsize'][i]),
    }
    noncars = [p for p in pairs if p['type'] != 'Car']
    S['decomposition_non_cars'] = {
        'along_frac_of_half_length': decompose(noncars, 'along', lambda p, f: p[f] / (p['lab_size'][0] / 2)),
        'length_m': decompose(noncars, 0, lambda p, i: p['dsize'][i]),
        'width_m': decompose(noncars, 1, lambda p, i: p['dsize'][i]),
        'height_m': decompose(noncars, 2, lambda p, i: p['dsize'][i]),
    }

    # Is a "flip" the 180 degree window of the network's yaw output?
    dyw = np.array([p['det_yaw_deg'] for p in pairs])
    lyw = np.array([p['lab_yaw_deg'] for p in pairs])
    lo_w, hi_w = float(np.percentile(dyw, 0.5)), float(np.percentile(dyw, 99.5))
    fl = np.array([p['flip'] for p in pairs])
    inside = np.array([any(lo_w <= v + k <= hi_w for k in (-360, 0, 360)) for v in lyw])
    S['yaw_window'] = {'det_yaw_p0_5_p99_5_deg': [lo_w, hi_w],
                       'flip_rate_label_inside_window': float(fl[inside].mean()) if inside.any() else None,
                       'flip_rate_label_outside_window': float(fl[~inside].mean()) if (~inside).any() else None,
                       'n_inside': int(inside.sum()), 'n_outside': int((~inside).sum())}

    big = [m for m in misses if m['points'] >= 300]
    S['misses_ge300_points'] = {
        'n': len(big),
        'by_type': {t: sum(1 for m in big if m['type'] == t) for t in set(m['type'] for m in big)},
        'range_p10_p50_p90': [float(np.percentile([m['range'] for m in big], q)) for q in (10, 50, 90)] if big else None,
        'share_abs_x_gt_70_4': float(np.mean([abs(m['x']) > 70.4 for m in big])) if big else None}
    allm = np.array([abs(m['x']) > 70.4 for m in misses])
    S['misses_outside_detector_x_range_share'] = float(allm.mean()) if len(allm) else None

    ds = np.array([p['dsize'] for p in pairs])
    by_type = defaultdict(list)
    for p in pairs:
        by_type[p['type']].append(p)
    S['size'] = {'mean_dl_dw_dh': ds.mean(0).tolist(), 'rms_dl_dw_dh': np.sqrt((ds ** 2).mean(0)).tolist(),
                 'by_type': {t: {'n': len(v), 'mean_dl_dw_dh': np.mean([p['dsize'] for p in v], 0).tolist(),
                                 'mean_label_lwh': np.mean([p['lab_size'] for p in v], 0).tolist()}
                             for t, v in by_type.items()}}

    flip = np.array([p['flip'] for p in pairs])
    dy = np.array([p['dyaw_mod180'] for p in pairs])
    S['yaw'] = {'flip_rate': float(flip.mean()), 'rms_mod180_deg': rms(dy),
                'p50_p95_mod180_deg': [pct(dy, 50), pct(dy, 95)],
                'flip_rate_by_range': [{'range_m': [lo, hi], 'n': int(m.sum()), 'flip_rate': float(flip[m].mean())}
                                       for lo, hi, m in binned(None, rng, RANGE_BINS) if m.sum()]}
    # do flips persist per object?
    by = defaultdict(list)
    for p in pairs:
        by[p['obj']].append(p['flip'])
    fr = np.array([np.mean(v) for v in by.values() if len(v) >= MIN_FRAMES_PER_OBJECT])
    S['yaw']['per_object_flip_share_hist'] = np.histogram(fr, bins=[0, 0.01, 0.1, 0.5, 0.9, 0.99, 1.0001])[0].tolist()
    S['yaw']['per_object_flip_share_bins'] = [0, 0.01, 0.1, 0.5, 0.9, 0.99, 1.0]

    mr = np.array([m['range'] for m in misses])
    mp = np.array([m['points'] for m in misses])
    pp = np.array([p['points'] for p in pairs])
    rows = []
    for lo, hi in zip(POINT_BINS[:-1], POINT_BINS[1:]):
        nm = int(((mp >= lo) & (mp < hi)).sum())
        nt = int(((pp >= lo) & (pp < hi)).sum())
        rows.append({'points': [lo, hi], 'labels': nm + nt, 'miss_rate': nm / (nm + nt) if nm + nt else None})
    S['miss_by_points'] = rows
    rows = []
    vis_m, vis_p = mp >= 5, pp >= 5
    for lo, hi in zip(RANGE_BINS[:-1], RANGE_BINS[1:]):
        nm = int(((mr >= lo) & (mr < hi) & vis_m).sum())
        nt = int(((rng >= lo) & (rng < hi) & vis_p).sum())
        rows.append({'range_m': [lo, hi], 'labels_ge5_points': nm + nt,
                     'miss_rate': nm / (nm + nt) if nm + nt else None})
    S['miss_by_range_ge5_points'] = rows
    S['miss_rate_ge5_points'] = float(vis_m.sum() / (vis_m.sum() + vis_p.sum()))

    fs = np.array([f['score'] for f in fps])
    near = np.array([f['near_other_car_label'] for f in fps])
    S['false_positives_detail'] = {'per_frame_per_car': len(fps) / (2 * len(res)),
                                   'share_near_other_car_label': float(near.mean()) if len(near) else None}
    ts = np.array([p['score'] for p in pairs])
    rows = []
    for lo, hi in zip(SCORE_BINS[:-1], SCORE_BINS[1:]):
        nt = int(((ts >= lo) & (ts < hi)).sum())
        nf = int(((fs >= lo) & (fs < hi)).sum())
        nf_strict = int(((fs >= lo) & (fs < hi) & ~near).sum())
        rows.append({'score': [lo, hi], 'detections': nt + nf,
                     'share_matched': nt / (nt + nf) if nt + nf else None,
                     'share_matched_or_near_other_label': (nt + nf - nf_strict) / (nt + nf) if nt + nf else None})
    S['score_calibration'] = rows
    return S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('export')
    ap.add_argument('data_dir')
    ap.add_argument('out')
    ap.add_argument('--gate', type=float, default=3.0)
    ap.add_argument('--workers', type=int, default=32)
    ap.add_argument('--gates_extra', type=str, default='',
                    help='comma list of other gates, reported as position RMS only')
    a = ap.parse_args()
    recs = [json.loads(l) for l in gzip.open(a.export, 'rt')]
    with Pool(a.workers) as pool:
        res = pool.map(process_frame, [(r, a.data_dir, a.gate) for r in recs], chunksize=4)
    S = summarize(res, a.gate)
    S['export'] = os.path.basename(a.export)
    for g in [float(x) for x in a.gates_extra.split(',') if x]:
        with Pool(a.workers) as pool:
            rg = pool.map(process_frame, [(r, a.data_dir, g) for r in recs], chunksize=4)
        pr = [p for r in rg for p in r['pairs']]
        S.setdefault('gate_sensitivity', []).append({
            'gate_m': g, 'matched': len(pr),
            'rms_h_per_axis': rms([p['ex'] for p in pr] + [p['ey'] for p in pr]),
            'rms_z': rms([p['ez'] for p in pr])})
    with open(a.out, 'w') as f:
        json.dump(S, f, indent=1)
    print(json.dumps({k: S[k] for k in ('frames', 'detections', 'labels_in_range', 'matched',
                                        'missed_labels', 'false_positives', 'sigma_h_fit_a_b',
                                        'sigma_z_fit_a_b', 'miss_rate_ge5_points')}, indent=1))


if __name__ == '__main__':
    main()
