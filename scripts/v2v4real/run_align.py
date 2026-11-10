"""V2V4Real held-out runs with relative alignment (--align), and the
pre-registered rule that picks run B's pose sigma on the val split.

Run A: the frozen held-out settings of run_heldout.py (pose 1.27 / 2.0 m,
0.1 deg; box 0.18 m, 1.7 deg), unchanged, plus --align, on test2 and test3,
run once.

Run B: a smaller pose sigma chosen on the val split by RULE below (written and
committed before any val replay was run), then run once on test2 and test3.
test1 cannot be used to choose it: alignment needs 3 objects both cars report
(AlignConfig::min_pairs) and test1 has at most 2 per frame, so alignment
never fires there (docs/results/v2v4real.md, "Held-out runs with relative
alignment").

RULE (pre-registered):
* Candidates: sigma_h in {0.5, 0.3, 0.2, 0.1} m, sigma_v = sigma_h * 2.0 / 1.27
  (the frozen ratio), sigma_yaw 0.1 deg; box settings frozen; --align on.
* Reference: the frozen settings plus --align (run A's configuration) on val.
* Admissible: pooled merged % (detected by both cars and fused into one /
  detected by both and matched) within 2 percentage points of the reference.
* Chosen: the admissible candidate whose pooled position NEES on objects both
  cars labeled is closest to 3; a tie goes to the larger sigma.
* No admissible candidate: run B is not run.
* Meaningful only if alignment fires in at least 25% of val car-frames
  (frames where a car's world has an offset for both cars) and in at least 2
  of the 3 val scenarios. Otherwise run B is not run and only A is reported.

Usage (python -I):
  run_align.py a      DATA WORK OUT.json             # test2 + test3, frozen + align
  run_align.py select DATA WORK OUT.json             # val: reference + candidates, applies RULE
  run_align.py b      DATA WORK OUT.json SIGMA_H     # test2 + test3, chosen sigma + align
DATA is $V2V4REAL_DIR (with test2, test3, val); WORK holds the converted files.
"""

import glob
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_all import DELAY_MS, REPLAY, convert, pool  # noqa: E402
from run_heldout import FROZEN_BOX, FROZEN_POSE  # noqa: E402

CANDIDATES_H = [0.5, 0.3, 0.2, 0.1]
V_RATIO = FROZEN_POSE['sigma_v'] / FROZEN_POSE['sigma_h']
MERGED_TOL_PTS = 2.0
NEES_TARGET = 3.0
MIN_FIRED_FRACTION = 0.25
MIN_FIRED_SCENARIOS = 2


def decl(sigma_h):
    if sigma_h == FROZEN_POSE['sigma_h']:
        return dict(FROZEN_POSE)
    return {'sigma_h': sigma_h, 'sigma_v': round(sigma_h * V_RATIO, 3),
            'sigma_yaw_deg': FROZEN_POSE['sigma_yaw_deg']}


def short(path):
    return os.path.basename(path.rstrip('/')).replace('testoutput_CAV_data_', '').split('-', 3)[3]


def replay_align(out_dir):
    res = os.path.join(out_dir, 'result_align.json')
    cmd = [REPLAY, '--dir', out_dir, '--out', res, '--delay-ms', str(DELAY_MS), '--align',
           '--box-size-sys-m', str(FROZEN_BOX['box_size_sys_m']),
           '--box-yaw-sys-deg', str(FROZEN_BOX['box_yaw_sys_deg'])]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    with open(res) as fh:
        return json.load(fh)


def headline(results):
    """Pooled metrics plus the percentages and alignment statistics reported."""
    p = pool(results)
    cars = [c for r in results for c in r['cars']]
    offs = [x for c in cars for x in c['align_relative_offset_m']]
    fired = sum(c['aligned_frames'] for c in cars)
    n1 = p['n_nees'] - p['n_both_labeled']
    p['merged_pct'] = 100.0 * p['fused_fraction_when_both_detect'] if p['detected_by_both_matched'] else None
    p['missed_pct'] = 100.0 * p['missed'] / (p['matched'] + p['missed'])
    p['sharing_only_pct'] = 100.0 * p['matched_never_detected_by_own_sensor'] / p['matched']
    p['nees_pos_3d_mean_one_labeled'] = ((p['nees_pos_3d_mean'] * p['n_nees'] - p['nees_pos_3d_mean_both_labeled']
                                          * p['n_both_labeled']) / n1) if n1 and p['n_both_labeled'] else None
    p['aligned_car_frames'] = fired
    p['aligned_fraction'] = fired / p['frames']
    p['align_relative_offset_m_median'] = float(np.median(offs)) if offs else None
    p['align_relative_offset_m_p95'] = float(np.percentile(offs, 95)) if offs else None
    p['align_relative_offset_m_max'] = float(np.max(offs)) if offs else None
    p['align_shifts_over_8p5m'] = int(sum(1 for x in offs if x > 8.5))
    return p


def run_set(scenarios, work, sigma_h, tag):
    d = decl(sigma_h)
    per, results = {}, []
    for sc in scenarios:
        od = os.path.join(work, '%s_h%s' % (tag, sigma_h), short(sc))
        convert(sc, od, d)
        r = replay_align(od)
        results.append(r)
        per[short(sc)] = headline([r])
    return {'pose': d, 'box': FROZEN_BOX, 'align': True, 'scenarios': per, 'total': headline(results)}


def fmt(name, h):
    return ('%-12s frames %5d | merged %5.1f%% missed %4.1f%% sharing-only %4.1f%% | dup %4d false %4d | '
            'idchg/obj %.2f | NEES both %.2f one %s all %.2f | aligned %d (%.0f%%) median rel offset %s m'
            % (name, h['frames'], h['merged_pct'] or float('nan'), h['missed_pct'], h['sharing_only_pct'],
               h['duplicates'], h['false_estimates'], h['id_changes_per_object'] or float('nan'),
               h['nees_pos_3d_mean_both_labeled'] or float('nan'),
               '%.2f' % h['nees_pos_3d_mean_one_labeled'] if h['nees_pos_3d_mean_one_labeled'] is not None else 'n/a',
               h['nees_pos_3d_mean'], h['aligned_car_frames'], 100 * h['aligned_fraction'],
               '%.2f' % h['align_relative_offset_m_median'] if h['align_relative_offset_m_median'] is not None else 'n/a')
            + ' | >8.5 m %d' % h['align_shifts_over_8p5m'])


def show(run):
    for n, h in run['scenarios'].items():
        print(fmt(n, h))
    print(fmt('TOTAL', run['total']))


def heldout(data):
    return [sc for s in ('test2', 'test3') for sc in sorted(glob.glob(os.path.join(data, s, '*/')))]


def main():
    mode, data, work, out_path = sys.argv[1:5]
    if mode == 'a':
        out = {'run': 'A: frozen settings plus --align', 'test2_test3': run_set(heldout(data), work, FROZEN_POSE['sigma_h'], 'heldout')}
        show(out['test2_test3'])
    elif mode == 'b':
        sh = float(sys.argv[5])
        assert sh in CANDIDATES_H, 'sigma_h must be one of the pre-registered candidates'
        out = {'run': 'B: pose sigma chosen on val by the pre-registered rule, plus --align',
               'test2_test3': run_set(heldout(data), work, sh, 'heldout')}
        show(out['test2_test3'])
    elif mode == 'select':
        val = sorted(glob.glob(os.path.join(data, 'val', '*/')))
        ref = run_set(val, work, FROZEN_POSE['sigma_h'], 'val')
        print('reference (frozen + align):')
        show(ref)
        t = ref['total']
        fired_sc = sum(1 for h in ref['scenarios'].values() if h['aligned_car_frames'] > 0)
        meaningful = t['aligned_fraction'] >= MIN_FIRED_FRACTION and fired_sc >= MIN_FIRED_SCENARIOS
        cands = {}
        for sh in CANDIDATES_H:
            cands[str(sh)] = run_set(val, work, sh, 'val')
            print('candidate sigma_h %s:' % sh)
            show(cands[str(sh)])
        adm = [(abs(c['total']['nees_pos_3d_mean_both_labeled'] - NEES_TARGET), -float(k), float(k))
               for k, c in cands.items()
               if abs(c['total']['merged_pct'] - t['merged_pct']) <= MERGED_TOL_PTS]
        chosen = min(adm)[2] if adm and meaningful else None
        out = {'rule': __doc__.split('RULE (pre-registered):')[1].split('Usage')[0].strip(),
               'aligned_fraction_reference': t['aligned_fraction'], 'scenarios_where_alignment_fired': fired_sc,
               'meaningful': meaningful, 'admissible': sorted(a[2] for a in adm), 'chosen_sigma_h': chosen,
               'reference': ref, 'candidates': cands}
        print('meaningful %s (aligned %.0f%% of car-frames, fired in %d scenarios); admissible %s; chosen %s'
              % (meaningful, 100 * t['aligned_fraction'], fired_sc, out['admissible'], chosen))
    else:
        raise SystemExit(__doc__)
    for run in ([out.get('test2_test3')] if 'test2_test3' in out else [out['reference']] + list(out['candidates'].values())):
        for h in list(run['scenarios'].values()) + [run['total']]:
            h.pop('per_frame_matched_missed_dup_false', None)
    with open(out_path, 'w') as fh:
        json.dump(out, fh, indent=1)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
