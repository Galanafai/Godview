"""Phase D2 step 1, second pass: ego_speed and frame rate, label id semantics
across agents, and whether the recording agents are labeled.

Usage (python -I): survey_v2xreal_ids.py SPLIT_DIR OUT.json
"""
import json
import os
import sys
from collections import Counter

import numpy as np
import yaml

try:
    Loader = yaml.CSafeLoader
except AttributeError:
    Loader = yaml.SafeLoader


def load(p):
    return yaml.load(open(p), Loader=Loader)


def main():
    root, out = sys.argv[1:3]
    res = {}
    speeds, steps = [], []
    pair_same, pair_diff, noid_near, noid_total = [], [], 0, 0
    per_agent_labels = {'vehicle': [], 'infra': []}
    own_label_dist = []
    types_by_agent = {'vehicle': Counter(), 'infra': Counter()}
    for scen in sorted(os.listdir(root)):
        sd = os.path.join(root, scen)
        agents = sorted(os.listdir(sd), key=int)
        frames = sorted({n.split('.')[0].split('_')[0] for n in os.listdir(os.path.join(sd, agents[0]))})
        prev = {}
        for fi in frames[::2]:
            ys = {a: load(os.path.join(sd, a, fi + '.yaml')) for a in agents}
            for a in agents:
                kind = 'infra' if ys[a]['infra'] else 'vehicle'
                per_agent_labels[kind].append(len(ys[a]['vehicles']))
                for v in ys[a]['vehicles'].values():
                    types_by_agent[kind][v['obj_type']] += 1
                if int(a) > 0:
                    speeds.append(ys[a]['ego_speed'])
                    if a in prev and int(fi) - prev[a][0] == 2:
                        steps.append((ys[a]['ego_speed'], np.linalg.norm(np.array(ys[a]['lidar_pose'][:2]) - prev[a][1]) / 2))
                    prev[a] = (int(fi), np.array(ys[a]['lidar_pose'][:2]))
            for i, a in enumerate(agents):
                for b in agents[i + 1:]:
                    A, B = ys[a]['vehicles'], ys[b]['vehicles']
                    for k in set(A) & set(B):
                        d = np.linalg.norm(np.array(A[k]['location']) - np.array(B[k]['location']))
                        (pair_same if d < 0.01 else pair_diff).append(d)
                    # labels of A with an id absent in B: is there a B label of the same class within 1 m?
                    Bl = [(np.array(v['location']), v['obj_type']) for v in B.values()]
                    for k, v in A.items():
                        if k in B:
                            continue
                        noid_total += 1
                        if any(t == v['obj_type'] and np.linalg.norm(l - np.array(v['location'])) < 1.0 for l, t in Bl):
                            noid_near += 1
            # agents as labels: distance from each agent's lidar xy to the nearest label of any agent
            for a in agents:
                p = np.array(ys[a]['lidar_pose'][:3])
                for b in agents:
                    for v in ys[b]['vehicles'].values():
                        pass
                allv = [np.array(v['location']) for b in agents for v in ys[b]['vehicles'].values()]
                if allv:
                    d = np.linalg.norm(np.array(allv)[:, :2] - p[:2], axis=1)
                    own_label_dist.append((int(a) > 0, float(d.min())))
    res['ego_speed_values'] = {'min': float(np.min(speeds)), 'median': float(np.median(speeds)), 'max': float(np.max(speeds)),
                               'share_zero': float(np.mean(np.array(speeds) == 0))}
    s = np.array([x for x in steps if x[0] > 0.5])
    res['ego_speed_vs_displacement_per_frame'] = {
        'n_frames_with_speed_over_0.5': int(len(s)),
        'speed_over_displacement_median': float(np.median(s[:, 0] / s[:, 1])) if len(s) else None,
        'displacement_m_median': float(np.median(s[:, 1])) if len(s) else None}
    res['labels_per_frame_per_agent'] = {k: {'median': float(np.median(v)), 'max': int(max(v))} for k, v in per_agent_labels.items()}
    res['classes_by_agent_kind'] = {k: dict(v) for k, v in types_by_agent.items()}
    res['same_id_in_two_agents'] = {
        'pairs_identical_location_lt_0.01m': len(pair_same), 'pairs_different': len(pair_diff),
        'different_distance_m_p50_p90_max': [float(np.percentile(pair_diff, 50)), float(np.percentile(pair_diff, 90)), float(np.max(pair_diff))] if pair_diff else None}
    res['label_id_in_one_agent_only'] = {'labels': noid_total, 'with_same_class_label_within_1m_in_the_other_agent': noid_near}
    res['agent_to_nearest_label_xy_m'] = {
        'vehicles_median': float(np.median([d for v, d in own_label_dist if v])),
        'vehicles_p10': float(np.percentile([d for v, d in own_label_dist if v], 10)),
        'infra_median': float(np.median([d for v, d in own_label_dist if not v]))}
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
