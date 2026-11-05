"""V2V4Real format check: possible missing ass_id links and large size disagreements.

* Unlinked pairs: car 1 labels without ass_id that lie within 3 m (after the
  pose transform) of a car 0 label no car 1 label links to, grouped by
  (car 0 id, car 1 id), with frame count, median distance and types.
* Linked objects whose label sizes differ by more than 1 m on some axis.

Usage: python -I check_links_and_size_outliers.py $V2V4REAL_DIR/test2
"""
import sys, os, glob, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
import numpy as np

root = sys.argv[1]
for sc in sorted(glob.glob(root + '/*/')):
    frames = sorted(os.path.basename(p)[:-5] for p in glob.glob(sc + '0/*.yaml'))
    near = collections.defaultdict(list); big = {}
    for f in frames:
        d = [load_yaml(f'{sc}{c}/{f}.yaml') for c in (0, 1)]
        T = [np.array(x['lidar_pose']) for x in d]
        V = [x['vehicles'] or {} for x in d]
        inv = np.linalg.inv(T[1]) @ T[0]
        linked0 = {v['ass_id'] for v in V[1].values() if v['ass_id'] != -1}
        for j, v in V[1].items():
            k = v['ass_id']
            if k != -1 and k in V[0]:
                dd = 2 * (np.array(V[0][k]['extent']) - np.array(v['extent']))
                if np.abs(dd).max() > 1.0:
                    big.setdefault((k, j), (np.round(2 * np.array(V[0][k]['extent']), 2), np.round(2 * np.array(v['extent']), 2), V[0][k]['obj_type'], v['obj_type'], f))
                continue
            for k0, v0 in V[0].items():
                if k0 in linked0: continue
                p = (inv @ np.r_[v0['location'], 1])[:2]
                dist = np.linalg.norm(p - np.array(v['location'][:2]))
                if dist < 3.0:
                    near[(k0, j)].append((dist, v0['obj_type'], v['obj_type']))
    print('=====', os.path.basename(sc[:-1]))
    for (k0, j), l in sorted(near.items(), key=lambda x: -len(x[1])):
        print('  unlinked near pair car0 id %s / car1 id %s: %d frames, median %.2f m, types %s/%s' % (k0, j, len(l), np.median([x[0] for x in l]), l[0][1], l[0][2]))
    for (k, j), v in big.items():
        print('  size outlier car0 id %s / car1 id %s: car0 %s car1 %s types %s/%s (first frame %s)' % (k, j, v[0], v[1], v[2], v[3], v[4]))
