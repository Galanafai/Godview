"""Phase D1: break down the replay's split label-frames by cause.

Input: the JSON lines written by `v2v4real-replay --split-diag` (one line per
label to which two or more world objects of one car are attributed).

Classes, per split label-frame, from the tracks attributed to the label:
(a) same car: one source car has tracks attributed to the label in two or
    more of the world objects. Sub-causes, from that car's two most recently
    updated such tracks:
    * restart: the younger track was created after the older one's last
      update (the track was dropped and a new one started; the old one is
      still in the world until it is terminated);
    * false beside real: the tracks overlap in time and one of them was
      started by a detection attributed to no label;
    * both real: they overlap and both were started by detections of this
      label;
    * other.
(b) cross car: not (a); the label's objects hold one car's track each and
    were not fused. Sub-causes: an object already holds another track of the
    other car (so the same-source rule blocks the fusion), else the gates
    between the two objects (position, velocity, size, heading), else none.
(c) other.

Usage: python split_breakdown.py OUT.json FILE.splits.jsonl [...]
"""
import json
import sys
from collections import Counter

GATE = {'pos_chi2': 21.11, 'vel_chi2': 21.11, 'size_chi2': 21.11, 'yaw_chi2': 15.13}


def classify(ev):
    g = ev['gid']
    per_src = {0: [], 1: []}          # (object index, member) attributed to g
    for oi, o in enumerate(ev['objects']):
        for m in o['members']:
            if m['last_gid'] == g:
                per_src[m['src']].append((oi, m))
    same = [s for s in (0, 1) if len({oi for oi, _ in per_src[s]}) >= 2]
    if same:
        s = max(same, key=lambda s: len(per_src[s]))
        ms = sorted((m for _, m in per_src[s]), key=lambda m: -m['last_update_ns'])[:2]
        old, young = sorted(ms, key=lambda m: (m['created_ns'], m['track']))
        other_unfused = bool(per_src[1 - s]) and not any(
            oi in {o for o, _ in per_src[s]} for oi, _ in per_src[1 - s])
        info = {'src_is_own': s == ev['car'], 'other_car_unfused_too': other_unfused,
                'gap_frames': round((young['created_ns'] - old['last_update_ns']) / 1e8),
                'hits': (old['hits'], young['hits'])}
        if young['created_ns'] > old['last_update_ns']:
            return 'a', 'restart', info
        if old['first_gid'] is None or young['first_gid'] is None:
            return 'a', 'false beside real', info
        if old['first_gid'] == g and young['first_gid'] == g:
            return 'a', 'both real', info
        return 'a', 'other', info
    if per_src[0] and per_src[1]:
        o0, o1 = ev['objects'][per_src[0][0][0]], ev['objects'][per_src[1][0][0]]
        if any(m['src'] == 1 for m in o0['members']) or any(m['src'] == 0 for m in o1['members']):
            return 'b', 'partner taken', {}
        if len(ev['objects']) == 2:
            for k in ('pos_chi2', 'size_chi2', 'yaw_chi2', 'vel_chi2'):
                if ev[k] is None or ev[k] >= GATE[k]:
                    return 'b', k.replace('_chi2', ' gate'), {'dist_h_m': ev['dist_h_m'], 'dz_m': ev['dz_m']}
            return 'b', 'gates pass', {'dist_h_m': ev['dist_h_m']}
        return 'b', 'more than two objects', {}
    return 'c', 'other', {}


def main():
    out_path, files = sys.argv[1], sys.argv[2:]
    cls, sub = Counter(), Counter()
    gaps, own, both, dist = Counter(), Counter(), 0, []
    n = 0
    for f in files:
        for line in open(f):
            if not line.strip():
                continue
            ev = json.loads(line)
            c, s, info = classify(ev)
            n += 1
            cls[c] += 1
            sub[c + ': ' + s] += 1
            if c == 'a':
                own['own car' if info['src_is_own'] else 'other car'] += 1
                both += info['other_car_unfused_too']
                if s == 'restart':
                    gf = info['gap_frames']
                    gaps['1' if gf <= 1 else '2 to 5' if gf <= 5 else '6 to 10' if gf <= 10 else 'over 10'] += 1
            if 'dist_h_m' in info and s == 'pos gate':
                dist.append(info['dist_h_m'])
    dist.sort()
    res = {'files': files, 'split_label_frames': n, 'classes': dict(cls), 'sub_causes': dict(sorted(sub.items())),
           'a_tracks_from': dict(own), 'a_with_other_car_also_unfused': both,
           'a_restart_gap_frames': dict(gaps),
           'b_pos_gate_dist_h_m_median_p90': [dist[len(dist) // 2], dist[int(len(dist) * 0.9)]] if dist else None}
    json.dump(res, open(out_path, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
