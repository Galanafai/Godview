"""V2V4Real format check: the heading disagreement tail between the two cars.

For every object linked by ass_id, compares world headings (pose yaw +
angle[1]) and classifies each pair: agree (<15 deg), front/back flip
(within 15 deg of 180), or other. Also checks each car's own label track
for frame-to-frame heading jumps, and whether yaw values sit near the
+-180 wrap.

Usage: python -I check_heading_tail.py $V2V4REAL_DIR/test1
"""
import sys, os, glob, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
import numpy as np

def yaw_of(T): return np.degrees(np.arctan2(T[1, 0], T[0, 0]))
def wrap(a): return (a + 180.0) % 360.0 - 180.0

root = sys.argv[1]
for sc in sorted(glob.glob(root + '/*/')):
    name = os.path.basename(sc[:-1])
    frames = sorted(os.path.basename(p)[:-5] for p in glob.glob(sc + '0/*.yaml'))
    D = [[load_yaml(f'{sc}{c}/{f}.yaml') for c in (0, 1)] for f in frames]
    cls = collections.Counter(); other = []; flips_by_obj = collections.Counter(); pairs_by_obj = collections.Counter()
    raw = []
    for i, d in enumerate(D):
        T = [np.array(x['lidar_pose']) for x in d]
        V = [x['vehicles'] or {} for x in d]
        for vid, v in V[1].items():
            k = v['ass_id']
            if k == -1 or k not in V[0]:
                continue
            a0 = V[0][k]['angle'][1]; a1 = v['angle'][1]
            y0 = yaw_of(T[0]) + a0; y1 = yaw_of(T[1]) + a1
            dy = wrap(y0 - y1)
            raw.append((a0, a1))
            pairs_by_obj[k] += 1
            if abs(dy) < 15: cls['agree'] += 1
            elif abs(abs(dy) - 180) < 15: cls['flip'] += 1; flips_by_obj[k] += 1
            else:
                cls['other'] += 1
                other.append((frames[i], k, vid, round(a0, 1), round(a1, 1), round(dy, 1),
                              V[0][k]['obj_type'], v['obj_type'], [round(e, 2) for e in V[0][k]['extent']], [round(e, 2) for e in v['extent']]))
    print('=====', name)
    print('  cross-car pairs:', dict(cls))
    print('  flip pairs per car0 object id:', dict(flips_by_obj), ' of pairs:', {k: pairs_by_obj[k] for k in flips_by_obj})
    for o in other[:12]: print('  other:', o)
    if len(other) > 12: print('  ... %d more' % (len(other) - 12))
    raw = np.array(raw) if raw else np.zeros((0, 2))
    if len(raw): print('  raw angle[1] near wrap (|a|>170) in pairs: car0 %d, car1 %d of %d' % ((np.abs(raw[:, 0]) > 170).sum(), (np.abs(raw[:, 1]) > 170).sum(), len(raw)))
    # Own-track jumps: world heading change of the same label id between consecutive frames.
    for c in (0, 1):
        jumps = collections.Counter(); n = 0; examples = []
        for i in range(1, len(D)):
            Tp = np.array(D[i - 1][c]['lidar_pose']); Tc = np.array(D[i][c]['lidar_pose'])
            Vp = D[i - 1][c]['vehicles'] or {}; Vc = D[i][c]['vehicles'] or {}
            for vid in set(Vp) & set(Vc):
                dy = wrap((yaw_of(Tc) + Vc[vid]['angle'][1]) - (yaw_of(Tp) + Vp[vid]['angle'][1]))
                n += 1
                if abs(abs(dy) - 180) < 20: jumps['flip'] += 1; examples.append((frames[i], vid, round(dy, 1)))
                elif abs(dy) > 20: jumps['other>20'] += 1; examples.append((frames[i], vid, round(dy, 1)))
        print('  car%d own label track, consecutive-frame heading changes: n=%d %s %s' % (c, n, dict(jumps), examples[:6]))

# Which label is right for the scenario 3 tail object (car 0 id 3 = car 1 id 2)?
# Compare each car's labeled world heading with the object's direction of
# motion (from car 0's labeled positions) and with the principal axis of the
# lidar points inside each car's box.
from v2v4real_io import load_pcd_xyz
sc = [s for s in sorted(glob.glob(root + '/*/')) if s.rstrip('/').endswith('10-29-43_3')]
if sc:
    sc = sc[0]
    print('===== tail object, scenario 10-29-43_3: car0 id 3 / car1 id 2')
    W = {}
    for f in range(10, 36):
        fr = '%06d' % f
        d = [load_yaml(f'{sc}{c}/{fr}.yaml') for c in (0, 1)]
        T = [np.array(x['lidar_pose']) for x in d]
        for c, vid in ((0, 3), (1, 2)):
            v = (d[c]['vehicles'] or {}).get(vid)
            if v is None: continue
            W[(f, c)] = ((T[c] @ np.r_[v['location'], 1])[:2], yaw_of(T[c]) + v['angle'][1], T[c], v)
    for f in range(12, 34, 2):
        row = ['frame %d' % f]
        if (f - 2, 0) in W and (f + 2, 0) in W:
            m = W[(f + 2, 0)][0] - W[(f - 2, 0)][0]
            row.append('motion heading %.1f (%.2f m/frame)' % (np.degrees(np.arctan2(m[1], m[0])), np.linalg.norm(m) / 4))
        for c in (0, 1):
            if (f, c) not in W: continue
            p, yw, T, v = W[(f, c)]
            P = load_pcd_xyz(f'{sc}{c}/%06d.pcd' % f)
            L, E, a = np.array(v['location']), np.array(v['extent']), np.radians(v['angle'][1])
            q = P - L; x = np.cos(a) * q[:, 0] + np.sin(a) * q[:, 1]; y = -np.sin(a) * q[:, 0] + np.cos(a) * q[:, 1]
            m = (np.abs(x) < E[0] * 1.3) & (np.abs(y) < E[0] * 1.3) & (np.abs(q[:, 2]) < E[2])
            pts = P[m, :2]
            pca = float('nan')
            if len(pts) >= 10:
                w, U = np.linalg.eigh(np.cov((pts - pts.mean(0)).T))
                ax = U[:, 1]; pca = np.degrees(np.arctan2(ax[1], ax[0])) + yaw_of(T)
            row.append('car%d label %.1f, points axis %.1f (mod 180, n=%d)' % (c, wrap(yw), wrap(pca) if pca == pca else pca, len(pts)))
        print('  ' + ' | '.join(row))
