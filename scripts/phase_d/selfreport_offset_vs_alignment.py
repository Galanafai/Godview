"""Phase D self reports (measurement only, nothing here is used by GodView).

On a V2V4Real split: the offset between each car's detection of the other car
and that other car's self-reported position (its own position in the shared map
frame, `agent_xyz` of the input), per frame, and whether it tracks the
translation GPS-offset alignment estimates between the two cars.

* o_c = (detection by car c of the other car k) - (position of car k itself), the
  detection being the one attributed to the label of the recording car k
  (`cav` of truth.json) nearest to k. Both are in the shared map frame, so
  o_c is a vector in that frame. It holds the difference of the two cars' pose
  errors plus detection noise and the offset between car k's lidar position
  and the centre of its box.
* r_align = -(t_0 - t_1), from a replay run with alignment on (the dump's
  `alignments`, car 0's world): alignment moves source s's reports by t_s so
  that the sources agree, so t_0 - t_1 estimates the negative of the
  difference of the sources' errors.
Compared per frame where all three exist: Pearson correlation of each component
(East and North in the map frame) and the median norm of the difference.

Usage (python3): selfreport_offset_vs_alignment.py INPUTS_DIR DUMP_DIR OUT.json
"""
import glob
import json
import os
import sys

import numpy as np


def main():
    inputs, dumps, out = sys.argv[1:4]
    res = {'scenarios': {}}
    allv = {'o01': [], 'o10': [], 'align': []}
    for f in sorted(glob.glob(dumps + '/*.dump.jsonl')):
        s = os.path.basename(f)[:-len('.dump.jsonl')]
        T = json.load(open(os.path.join(inputs, s, 'truth.json')))
        A = json.load(open(os.path.join(inputs, s, 'attribution.json')))['frames']
        cars = [[json.loads(l) for l in open(os.path.join(inputs, s, 'car%d.jsonl' % c)) if l.strip()] for c in (0, 1)]
        al = {}
        for line in open(f):
            e = json.loads(line)
            a0 = e['alignments'][0]
            by = {x['src']: np.array(x['t_odom'][:2]) for x in a0}
            if 0 in by and 1 in by:
                al[e['frame']] = -(by[0] - by[1])
        rows = []
        for i, fr in enumerate(T['frames']):
            ego = [np.array(x)[:2] for x in fr['ego_odom']]
            o = {}
            for c in (0, 1):
                k = 1 - c
                gids = {x['gid'] for x in fr['objects'] if x['cav'] == k}
                best = None
                for d, g in zip(cars[c][i]['detections'], A[i][c]):
                    if g in gids:
                        v = np.array([d['x'], d['y']]) - ego[k]
                        if best is None or np.linalg.norm(v) < np.linalg.norm(best):
                            best = v
                if best is not None:
                    o[c] = best
            rows.append((o.get(0), o.get(1), al.get(i)))
        both = [(a, b, c) for a, b, c in rows if a is not None and b is not None and c is not None]
        n_o0 = sum(a is not None for a, _, _ in rows)
        n_o1 = sum(b is not None for _, b, _ in rows)
        info = {'frames': len(rows), 'car0_detects_car1': n_o0, 'car1_detects_car0': n_o1, 'alignment_available': len(al),
                'frames_with_all_three': len(both)}
        if len(both) >= 5:
            o01 = np.array([a for a, _, _ in both])
            o10 = np.array([b for _, b, _ in both])
            ra = np.array([c for _, _, c in both])
            avg = (o01 - o10) / 2          # the two cars' views, mount offsets partly cancel
            def corr(x, y):
                return [float(np.corrcoef(x[:, k], y[:, k])[0, 1]) if x[:, k].std() > 1e-9 and y[:, k].std() > 1e-9 else None for k in (0, 1)]
            info.update({
                'median_norm_o01_m': float(np.median(np.linalg.norm(o01, axis=1))),
                'median_norm_o10_m': float(np.median(np.linalg.norm(o10, axis=1))),
                'median_norm_alignment_m': float(np.median(np.linalg.norm(ra, axis=1))),
                'corr_EN_o01_vs_alignment': corr(o01, ra),
                'corr_EN_minus_o10_vs_alignment': corr(-o10, ra),
                'corr_EN_average_vs_alignment': corr(avg, ra),
                'median_norm_difference_o01_minus_alignment_m': float(np.median(np.linalg.norm(o01 - ra, axis=1))),
                'median_norm_difference_average_minus_alignment_m': float(np.median(np.linalg.norm(avg - ra, axis=1))),
                'median_norm_difference_o01_vs_minus_o10_m': float(np.median(np.linalg.norm(o01 + o10, axis=1)))})
            allv['o01'] += o01.tolist(); allv['o10'] += o10.tolist(); allv['align'] += ra.tolist()
        res['scenarios'][s] = info
    if allv['align']:
        o01, o10, ra = (np.array(allv[k]) for k in ('o01', 'o10', 'align'))
        avg = (o01 - o10) / 2
        c = lambda x, y: [float(np.corrcoef(x[:, k], y[:, k])[0, 1]) for k in (0, 1)]
        res['pooled'] = {'frames': len(ra), 'corr_EN_o01_vs_alignment': c(o01, ra), 'corr_EN_average_vs_alignment': c(avg, ra),
                         'median_norm_o01_m': float(np.median(np.linalg.norm(o01, axis=1))),
                         'median_norm_alignment_m': float(np.median(np.linalg.norm(ra, axis=1))),
                         'median_norm_difference_average_minus_alignment_m': float(np.median(np.linalg.norm(avg - ra, axis=1)))}
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
