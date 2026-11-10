"""Phase D1 (explain only): break down the replay's wrong merges.

Input: --merge-diag lines from v2v4real-replay (one per wrong-merge
object-frame: the world object's members and the label each member's latest
detection was attributed to) and the scenario's truth.json.

Classes per event (gids = the distinct attributed labels):
* multi-link: every gid belongs to one group in truth.json
  `links_to_several_car0_ids` (one car 1 label linked by ass_id to several
  car 0 labels over the scenario, so car 0's re-numbered labels of one object
  look like different objects). A counting artifact.
* label absent: a gid has no label in this frame (its member's latest detection
  is older than the label), so no distance can be taken.
* same-object scale: the two labels' distance is within the 90th percentile of
  the disagreement between the two cars' labels of objects both labeled
  (linked by ass_id) in that scenario. Cannot be told apart from one object
  labeled twice and not linked; flagged with whether the gids are an
  unlinked pair (one car 0 label, one car 1 label).
* distinct: farther than that. A merge of two objects.

Usage (python3): explain_merges.py OUT.json SCENARIO_IN_DIR MERGE_DIR
"""
import glob
import json
import os
import sys
from collections import Counter, defaultdict
from itertools import combinations

import numpy as np


def main():
    out_path, in_dir, merge_dir = sys.argv[1:4]
    total = Counter()
    per_scen = {}
    ranges = defaultdict(Counter)
    dist_bins = Counter()
    types = Counter()
    episodes = Counter()
    examples = []
    for f in sorted(glob.glob(merge_dir + '/*.merge.jsonl')):
        scen = os.path.basename(f)[:-len('.merge.jsonl')]
        truth = json.load(open(os.path.join(in_dir, scen, 'truth.json')))
        groups = [set('0:%s' % k for k in ks) for ks in truth['links_to_several_car0_ids'].values()]
        frames = truth['frames']
        dis = []
        for fr in frames:
            for o in fr['objects']:
                if len(o['labeled_by']) == 2:
                    a, b = np.array(o['per_car_pos']['0']), np.array(o['per_car_pos']['1'])
                    dis.append(float(np.linalg.norm((a - b)[:2])))
        scale = float(np.percentile(dis, 90)) if dis else float('nan')
        by_frame = [{o['gid']: o for o in fr['objects']} for fr in frames]
        C = Counter()
        prev = {}
        for line in open(f):
            if not line.strip():
                continue
            e = json.loads(line)
            i, car = e['frame'], e['car']
            G = sorted({m['gid'] for m in e['members'] if m['gid']})
            if len(G) < 2:
                C['members_missing'] += 1
                continue
            key = (car, tuple(G))
            if prev.get(key) != i - 1:
                episodes[scen] += 1
            prev[key] = i
            C['events'] += 1
            if any(set(G) <= grp for grp in groups):
                C['multi-link (artifact)'] += 1
                continue
            objs = by_frame[i]
            ego = [np.array(x)[:2] for x in frames[i]['ego_odom']]
            if any(g not in objs for g in G):
                C['label absent this frame'] += 1
                continue
            # label position per gid, from the label of the source car of the member when it has one
            pos = {}
            for g in G:
                srcs = [m['src'] for m in e['members'] if m['gid'] == g]
                pc = objs[g]['per_car_pos']
                pos[g] = np.array(pc[str(srcs[0])] if str(srcs[0]) in pc else objs[g]['pos'])
            g1, g2 = min(combinations(G, 2), key=lambda p: np.linalg.norm((pos[p[0]] - pos[p[1]])[:2]))
            d = float(np.linalg.norm((pos[g1] - pos[g2])[:2]))
            mid = (pos[g1] + pos[g2])[:2] / 2
            r_own, r_other = float(np.linalg.norm(mid - ego[car])), float(np.linalg.norm(mid - ego[1 - car]))
            unlinked_pair = {g1[0], g2[0]} == {'0', '1'}
            cls = 'same-object scale' if d <= scale else 'distinct'
            C[cls] += 1
            # A label within 4 m of a recording car's position (convert_detections.py `cav`):
            # that car, or a neighbour closer than the radius resolves.
            n_cav = sum(objs[g]['cav'] is not None for g in G)
            # The scored car itself, as the other car labels it: labeled only by the other
            # car and within 2.5 m of the scored car's position.
            def is_self(g):
                o = objs[g]
                return o['labeled_by'] == [1 - car] and np.linalg.norm(np.array(o['pos'])[:2] - ego[car]) < 2.5
            if cls == 'distinct':
                C['distinct: one label is the scored car itself'] += any(is_self(g) for g in G)
                C['distinct: at least one label is a recording car (cav flag)'] += n_cav >= 1
                C['distinct: both labels are recording cars (cav flag)'] += n_cav >= 2
            if len(G) > 2:
                C['more than two labels'] += 1
            types[(cls, 'unlinked car0/car1 pair' if unlinked_pair else 'other pair')] += 1
            bins = [(0, 1), (1, 2), (2, 3), (3, 5), (5, 10), (10, 1e9)]
            dist_bins[(cls, next('%g-%g' % b if b[1] < 1e8 else '>%g' % b[0] for b in bins if b[0] <= d < b[1]))] += 1
            rb = next('%d-%d' % b if b[1] < 1e8 else '>%d' % b[0] for b in [(0, 20), (20, 40), (40, 60), (60, 80), (80, 1e9)] if b[0] <= r_own < b[1])
            ranges[cls][rb] += 1
            if cls == 'distinct' and len(examples) < 5:
                examples.append({'scenario': scen, 'frame': i, 'car': car, 'gids': [g1, g2], 'dist_m': round(d, 2),
                                 'range_from_scored_car_m': round(r_own, 1), 'range_from_other_car_m': round(r_other, 1)})
        per_scen[scen] = {'cross_car_label_disagreement_p90_m': round(scale, 2), 'linked_multi_groups': [sorted(g) for g in groups],
                          **dict(C), 'episodes': episodes[scen]}
        total.update(C)
    res = {'total': dict(total), 'episodes': sum(episodes.values()), 'per_scenario': per_scen,
           'distance_bins_m': {'%s %s' % k: v for k, v in sorted(dist_bins.items())},
           'range_from_scored_car_m': {k: dict(v) for k, v in ranges.items()},
           'pair_types': {'%s / %s' % k: v for k, v in types.items()}, 'distinct_examples': examples}
    json.dump(res, open(out_path, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
