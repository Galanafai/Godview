# Sanity check of an export: score range, median box size, and for every 10th
# frame the fraction of each car's own labels with a detection within 2 m.
# Usage: python -I check_export.py EXPORT.jsonl.gz DATA_DIR SCRIPTS_V2V4REAL_DIR
import sys, gzip, json, os
import numpy as np
sys.path.insert(0, sys.argv[3])
import v2v4real_io as io
recs = [json.loads(l) for l in gzip.open(sys.argv[1], 'rt')]
root = sys.argv[2]
hwl = np.array([b for r in recs for c in r['cars'] for b in c['box']])
sc = np.array([s for r in recs for c in r['cars'] for s in c['score']])
print('dets', len(hwl), 'score min/med/max %.3f %.3f %.3f' % (sc.min(), np.median(sc), sc.max()))
print('l w h median', np.median(hwl[:, 3:6], 0).round(2), 'z median %.2f' % np.median(hwl[:, 2]))
d_all = []; nlab = 0; nmatch = 0
for r in recs[::10]:
    for c in r['cars']:
        y = io.load_yaml(os.path.join(root, r['scenario'], c['cav_id'], r['timestamp'] + '.yaml'))
        labs = np.array([v['location'][:2] for v in y['vehicles'].values()]) if y['vehicles'] else np.zeros((0, 2))
        det = np.array(c['box'])[:, :2] if c['box'] else np.zeros((0, 2))
        P = np.asarray(y['lidar_pose']); assert np.allclose(P, np.array(c['lidar_pose']))
        for l in labs:
            nlab += 1
            if len(det):
                d = np.min(np.linalg.norm(det - l, axis=1)); d_all.append(d); nmatch += d < 2
print('labels sampled', nlab, 'with a detection within 2 m: %.2f' % (nmatch / nlab), 'median nearest dist %.2f m' % np.median(d_all))
