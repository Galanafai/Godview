"""V2V4Real format check: anchor each scenario's map frame on the Earth, and
look for shared objects that ass_id does not link.

GPS fit: per scenario, a 2-D rigid transform (rotation + translation, no
scale) from lidar_pose x, y of both cars over all frames to local ENU of the
gps latitude and longitude, plus a constant height offset from gps altitude.
The residual includes the GPS antenna to lidar lever arm.

Links: car 1 labels with ass_id = -1 that lie within 3 m of a car 0 label
after the pose transform (possible missing links).

Usage: python -I check_gps_fit_and_links.py $V2V4REAL_DIR/test1
"""
import sys, os, glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
from gps_fit import fit_map_to_enu
import numpy as np

root = sys.argv[1]
for sc in sorted(glob.glob(root + '/*/')):
    name = os.path.basename(sc[:-1])
    frames = sorted(os.path.basename(p)[:-5] for p in glob.glob(sc + '0/*.yaml'))
    D = [[load_yaml(f'{sc}{c}/{f}.yaml') for c in (0, 1)] for f in frames]
    fit = fit_map_to_enu([np.array(x['lidar_pose']) for d in D for x in d], [x['gps'] for d in D for x in d])
    print('=====', name)
    print('  fit: rotation %.3f deg (map x axis from East), origin lat %.7f lon %.7f h %.2f' % (fit['rot_deg'], fit['lat0'], fit['lon0'], fit['h0']))
    print('  horizontal residual: rms %.2f m, max %.2f m; per car rms %s; height residual rms %.2f m' % (
        fit['rms_h'], fit['max_h'], ['%.2f' % r for r in fit['rms_h_per'](2)], fit['rms_v']))
    print('  gps heading minus (pose yaw + rotation): car0 %.2f, car1 %.2f deg' % tuple(fit['heading_offset'](2)))
    near = []; unlinked = 0
    for d in D:
        T = [np.array(x['lidar_pose']) for x in d]
        V = [x['vehicles'] or {} for x in d]
        inv = np.linalg.inv(T[1]) @ T[0]
        P0 = {k: (inv @ np.r_[v['location'], 1])[:2] for k, v in V[0].items()}
        linked0 = {v['ass_id'] for v in V[1].values() if v['ass_id'] != -1}
        for vid, v in V[1].items():
            if v['ass_id'] != -1: continue
            unlinked += 1
            for k, p in P0.items():
                if k in linked0: continue
                dd = np.linalg.norm(p - np.array(v['location'][:2]))
                if dd < 3.0: near.append(dd)
    print('  car1 labels without ass_id: %d; of those within 3 m of an unlinked car0 label: %d %s' % (unlinked, len(near), np.round(sorted(near)[:8], 2)))
