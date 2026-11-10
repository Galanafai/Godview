"""Phase D2: pose sigmas for the V2X-Real converter, by rule, from val labels.

Rule (fixed before any GodView run): for every pair of agents' labels with the
same id in the same frame, the difference of the two boxes' centres per axis
(map frame x, y, z) and of their headings (vehicle classes only; heading mod 360
degrees) is the difference of the two agents' errors plus labeling noise. With two
independent agents of equal error, sigma = sqrt(E[d^2] / 2) per axis. Applied to
pairs of the two vehicles. This attributes all the label disagreement to the
poses, so it is an upper bound on the pose error, and it includes labels
identical to 1 cm (shared annotation), which carry no information on pose.
Reported alongside: without the identical pairs, per pair type (vehicle-vehicle,
unit-unit, vehicle-unit), per scenario.

Usage (python -I): v2xreal_pose_sigma.py SPLIT_DIR OUT.json
"""
import itertools
import json
import os
import sys
from collections import defaultdict

import numpy as np
import yaml

L = yaml.CSafeLoader
VEHICLE_CLASSES = {'Car', 'Van', 'Truck', 'Bus', 'ConcreteTruck'}


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    root, out = sys.argv[1:3]
    pairs = defaultdict(list)       # (kind, scenario) -> list of (dx, dy, dz, dyaw or nan, identical)
    for scen in sorted(os.listdir(root)):
        sd = os.path.join(root, scen)
        agents = sorted(os.listdir(sd), key=int)
        frames = sorted({n.split('.')[0].split('_')[0] for n in os.listdir(os.path.join(sd, agents[0]))})
        for fi in frames:
            ys = {a: yaml.load(open(os.path.join(sd, a, fi + '.yaml')), Loader=L) for a in agents}
            for a, b in itertools.combinations(agents, 2):
                kind = 'vehicle-vehicle' if int(a) > 0 and int(b) > 0 else 'unit-unit' if int(a) < 0 and int(b) < 0 else 'vehicle-unit'
                A, B = ys[a]['vehicles'], ys[b]['vehicles']
                for k in set(A) & set(B):
                    d = np.array(A[k]['location']) - np.array(B[k]['location'])
                    dyaw = wrap(A[k]['angle'][1] - B[k]['angle'][1]) if A[k]['obj_type'] in VEHICLE_CLASSES else float('nan')
                    pairs[(kind, scen)].append((*d, dyaw, bool(np.linalg.norm(d) < 0.01), wrap(2 * dyaw) / 2 if dyaw == dyaw else float('nan')))
    def stats(rows, skip_identical=False):
        r = np.array([x for x in rows if not (skip_identical and x[4])], float)
        if not len(r):
            return None
        sig = [float(np.sqrt(np.mean(r[:, k] ** 2) / 2)) for k in range(3)]
        yaw = r[:, 3][~np.isnan(r[:, 3])]
        yaw180 = r[:, 5][~np.isnan(r[:, 5])]
        return {'pairs': int(len(r)), 'sigma_x': sig[0], 'sigma_y': sig[1], 'sigma_z': sig[2],
                'sigma_h_isotropic': float(np.sqrt((sig[0] ** 2 + sig[1] ** 2) / 2)),
                'sigma_yaw_deg': float(np.sqrt(np.mean(yaw ** 2) / 2)) if len(yaw) else None,
                'sigma_yaw_deg_mod_180': float(np.sqrt(np.mean(yaw180 ** 2) / 2)) if len(yaw180) else None,
                'share_of_pairs_with_heading_differing_over_90_deg': float(np.mean(np.abs(yaw) > 90)) if len(yaw) else None,
                'yaw_pairs': int(len(yaw))}
    res = {'rule': 'sigma = sqrt(E[d^2] / 2) per axis; vehicle-vehicle pairs, all same-id pairs', 'by_kind': {}, 'by_kind_without_identical_pairs': {},
           'by_scenario_vehicle_vehicle': {}}
    for kind in ('vehicle-vehicle', 'unit-unit', 'vehicle-unit'):
        rows = [x for (k, s), v in pairs.items() if k == kind for x in v]
        res['by_kind'][kind] = stats(rows)
        res['by_kind_without_identical_pairs'][kind] = stats(rows, True)
        res.setdefault('identical_share', {})[kind] = float(np.mean([x[4] for x in rows])) if rows else None
    for (k, s), v in sorted(pairs.items()):
        if k == 'vehicle-vehicle':
            res['by_scenario_vehicle_vehicle'][s] = stats(v)
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
