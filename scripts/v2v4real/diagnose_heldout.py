"""Explain held-out misses and duplicates (diagnostic only; changes nothing).

For every missed object and every object shown twice (from the replay's
--diag output), classify it from truth.json: a small object (largest side
under 2 m, pedestrians and riders), another labeled object within 3 m in the
same frame (two labels that may be one object, for example a missing
ass_id link), a car 1 label that ass_id links to several car 0 IDs (the
converter pairs it with the lowest), a cross-car heading flip (the two cars'
labels differ by more than 165 deg), or a size disagreement over 0.5 m.

Usage: python -I diagnose_heldout.py CONVERTED_DIR DIAG_DIR
"""
import collections
import glob
import json
import os
import sys

import numpy as np


def main():
    conv, diag = sys.argv[1], sys.argv[2]
    tot = collections.Counter()
    for d in sorted(glob.glob(os.path.join(conv, '*/'))):
        name = os.path.basename(d.rstrip('/'))
        truth = json.load(open(os.path.join(d, 'truth.json')))
        multi = {'1:%s' % j for j in truth['links_to_several_car0_ids']} | \
                {'0:%s' % min(v) for v in truth['links_to_several_car0_ids'].values()}
        fl = [l for l in open(os.path.join(diag, name + '.diag')) if l.strip()]
        cnt = {'missed': collections.Counter(), 'dup': collections.Counter()}
        for line in fl:
            e = json.loads(line)
            objs = {o['gid']: o for o in truth['frames'][e['frame']]['objects']}
            P = {g: np.array(o['pos']) for g, o in objs.items()}
            for kind in ('missed', 'dup'):
                for g in e[kind]:
                    o = objs[g]
                    tags = []
                    if max(o['size'][:2]) < 2.0:
                        tags.append('small')
                    if any(np.linalg.norm((P[g] - P[h])[:2]) < 3.0 for h in P if h != g):
                        tags.append('label_within_3m')
                    if g in multi:
                        tags.append('multi_link')
                    pc = o['per_car']
                    if '0' in pc and '1' in pc:
                        dy = abs((pc['0']['yaw_enu_deg'] - pc['1']['yaw_enu_deg'] + 180) % 360 - 180)
                        if dy > 165:
                            tags.append('heading_flip')
                        if np.max(np.abs(np.array(pc['0']['size']) - np.array(pc['1']['size']))) > 0.5:
                            tags.append('size_diff_over_0.5m')
                    else:
                        tags.append('one_car_labels')
                    cnt[kind]['total'] += 1
                    for t in tags:
                        cnt[kind][t] += 1
                    if not tags or tags == ['one_car_labels']:
                        cnt[kind]['none_of_these'] += 1
        for kind in ('missed', 'dup'):
            print('%-22s %-6s %s' % (name, kind, dict(cnt[kind])))
            for k, v in cnt[kind].items():
                tot[(kind, k)] += v
    for kind in ('missed', 'dup'):
        print('TOTAL %-6s %s' % (kind, {k: v for (kk, k), v in sorted(tot.items()) if kk == kind}))


if __name__ == '__main__':
    main()
