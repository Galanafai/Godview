"""Phase D4: wrong merges in frames 110 to the end, in vehicle 1's ego view (the demo view) and in the others.

Usage (python -I): d4_wm_detail.py CONV_DIR DUMP.jsonl OUT.json
"""
import json
import os
import sys
from collections import Counter, defaultdict

import numpy as np

conv, dump, out = sys.argv[1:4]
T = json.load(open(os.path.join(conv, 'truth.json')))
frames, agents = T['frames'], T['agents']
folder = {a['index']: a['folder'] for a in agents}
v1 = [a['index'] for a in agents if a['folder'] == '1'][0]
PEDS = {'43', '58', '30', '46'}
FIRST = 110
cls = {o['gid']: o['class_id'] for fr in frames for o in fr['objects']}
res = {'by_agent_ego_view': {}, 'vehicle1_ego_view': {}}
pairs = defaultdict(Counter)
per_ped = {g: Counter() for g in PEDS}
for l in open(dump):
    if not l.strip():
        continue
    e = json.loads(l)
    i = e['frame']
    if i < FIRST:
        continue
    tf = frames[i]
    pos = {o['gid']: np.array(o['pos'][:2]) for o in tf['objects']}
    v1pos = np.array(tf['ego_odom'][v1][:2])
    around = {g for g, p in pos.items() if cls.get(g) in (1, 3) and np.linalg.norm(p - v1pos) <= 30}
    for ag in e['agents']:
        a = folder[ag['index']]
        for o in ag['ego']:
            labs = sorted(set(o['labels']))
            if len(labs) >= 2:
                pairs[a][tuple(labs)] += 1
                res['by_agent_ego_view'].setdefault(a, Counter())['wrong_merges'] += 1
                if a == '1':
                    c = res['vehicle1_ego_view'].setdefault('wrong_merges', 0)
                    res['vehicle1_ego_view']['wrong_merges'] = c + 1
                    for g in PEDS & set(labs):
                        per_ped[g]['frames_in_a_merged_object'] += 1
                        per_ped[g]['merged_with_vehicle'] += any(cls.get(x) in (1, 3) for x in labs if x != g)
                    if set(labs) & around:
                        res['vehicle1_ego_view']['involving_vehicles_around'] = res['vehicle1_ego_view'].get('involving_vehicles_around', 0) + 1
        if ag['index'] == v1:
            have = defaultdict(list)
            for o in ag['ego']:
                for g in o['labels']:
                    have[g].append(len(set(o['labels'])))
            for g in PEDS:
                if g in have:
                    per_ped[g]['frames_present_in_vehicle1_ego_view'] += 1
                    per_ped[g]['frames_with_a_clean_object'] += any(n == 1 for n in have[g])
res['by_agent_ego_view'] = {a: dict(c) for a, c in res['by_agent_ego_view'].items()}
res['vehicle1_per_pedestrian'] = {g: dict(c) for g, c in per_ped.items()}
res['top_merged_label_pairs_by_agent'] = {a: [[list(k), v] for k, v in c.most_common(6)] for a, c in pairs.items()}
res['frames'] = '%d to %d' % (FIRST, len(frames) - 1)
json.dump(res, open(out, 'w'), indent=1)
print(json.dumps(res, indent=1))
