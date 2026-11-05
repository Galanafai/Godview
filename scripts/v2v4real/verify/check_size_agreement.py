"""V2V4Real format check: do the two cars label the same object with the same size?

For every object linked by ass_id, compares full sizes (2 * extent) per
dimension. Sizes are constant per label ID over time within a car, so each
object is counted once per scenario as well as per frame.

Usage: python -I check_size_agreement.py $V2V4REAL_DIR/test1
"""
import sys, os, glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
import numpy as np

root = sys.argv[1]
allobj = []
for sc in sorted(glob.glob(root + '/*/')):
    frames = sorted(os.path.basename(p)[:-5] for p in glob.glob(sc + '0/*.yaml'))
    per_obj = {}; const = [0, 0]
    sizes_by_id = [{}, {}]
    for f in frames:
        d = [load_yaml(f'{sc}{c}/{f}.yaml') for c in (0, 1)]
        V = [x['vehicles'] or {} for x in d]
        for c in (0, 1):
            for vid, v in V[c].items():
                sizes_by_id[c].setdefault(vid, set()).add(tuple(np.round(2 * np.array(v['extent']), 4)))
        for vid, v in V[1].items():
            k = v['ass_id']
            if k == -1 or k not in V[0]: continue
            per_obj[(k, vid)] = (2 * np.array(V[0][k]['extent']), 2 * np.array(v['extent']), V[0][k]['obj_type'], v['obj_type'])
    nconst = sum(len(s) == 1 for c in (0, 1) for s in sizes_by_id[c].values()); ntot = sum(len(sizes_by_id[c]) for c in (0, 1))
    print('=====', os.path.basename(sc[:-1]), ' label ids with a single size over time: %d of %d' % (nconst, ntot))
    for (k, j), (s0, s1, t0, t1) in sorted(per_obj.items()):
        d = s0 - s1
        allobj.append(d)
        print('  car0 id %s / car1 id %s (%s/%s): car0 L,W,H %s car1 %s diff %s' % (k, j, t0, t1, np.round(s0, 2), np.round(s1, 2), np.round(d, 2)))
D = np.abs(np.array(allobj))
print('linked objects: %d; |size difference| L,W,H median %s max %s' % (len(D), np.round(np.median(D, 0), 2), np.round(D.max(0), 2)))
