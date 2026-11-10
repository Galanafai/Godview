"""D4 report-only check: ghost objects near hidden pedestrians in vehicle 1's ego view.

For each of the four pedestrians, over the frames of their hidden interval (from analysis.json), the distance
(horizontal, odom frame) from the person's label centre to the nearest ghost object (status ghost, any class)
in vehicle 1's ego view; per frame, the number of ghost objects within 2 m of any hidden person against the number
of hidden people. A stretch over 1 s (more than 10 frames) with no ghost within 1 m of a person is flagged.
Usage (python -I): d4_ghost_check.py PACKAGE_DIR ANALYSIS.json OUT.json
"""
import json, os, sys
import numpy as np

pk, ana, out = sys.argv[1:4]
A = json.load(open(ana))['pedestrians']
res = {'per_person': {}, 'per_frame': []}
hidden = {g: range(v['interval_frames'][0], v['interval_frames'][1] + 1) for g, v in A.items()}
dist = {g: {} for g in A}
frames = {}
for i in range(100, 240):
    frames[i] = json.load(open(os.path.join(pk, 'frames', '%06d.json' % i)))
for i, r in frames.items():
    gh = np.array([[o['x'], o['y']] for o in r['ego_view'] if o['status'] == 'ghost']).reshape(-1, 2)
    tr = {o['gid']: np.array(o['pos'][:2]) for o in r['truth']}
    hid = [g for g in A if i in hidden[g]]
    near = set()
    for g in hid:
        d = np.linalg.norm(gh - tr[g], axis=1) if len(gh) else np.array([np.inf])
        dist[g][i] = float(d.min())
    if hid:
        cnt = 0
        for k, p in enumerate(gh):
            if any(np.linalg.norm(p - tr[g]) <= 2.0 for g in hid):
                cnt += 1
        res['per_frame'].append({'frame': i, 'people_hidden': len(hid), 'ghosts_within_2m': cnt})
for g, d in dist.items():
    v = np.array(list(d.values())); fr = sorted(d)
    run, stretches = [], []
    for f in fr:
        if d[f] > 1.0:
            run.append(f)
        else:
            if len(run) > 10: stretches.append((run[0], run[-1]))
            run = []
    if len(run) > 10: stretches.append((run[0], run[-1]))
    res['per_person'][g] = {'frames': [fr[0], fr[-1]], 'n': len(fr), 'median_m': float(np.median(v)), 'max_m': float(v.max()),
                            'frames_with_no_ghost_within_1m': int((v > 1.0).sum()), 'stretches_over_1s_no_ghost_within_1m': stretches}
pf = res['per_frame']
res['summary'] = {'frames_with_hidden_people': len(pf), 'mean_people': float(np.mean([p['people_hidden'] for p in pf])),
                  'mean_ghosts_within_2m': float(np.mean([p['ghosts_within_2m'] for p in pf])),
                  'frames_ghosts_less_than_people': sum(p['ghosts_within_2m'] < p['people_hidden'] for p in pf),
                  'frames_ghosts_more_than_people': sum(p['ghosts_within_2m'] > p['people_hidden'] for p in pf)}
json.dump(res, open(out, 'w'), indent=1)
print(json.dumps(res['per_person'], indent=0)); print(res['summary'])
