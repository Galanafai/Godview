"""Run the V2V4Real replay for every declaration and write the JSON summary.

Declarations of the pose uncertainty (see docs/results/v2v4real.md):
* nominal: standalone (non-RTK) datasheet figures of the cars' GPS/INS units;
* loo_<k>: a sigma that covers the measured relative offset, chosen from
  scenario k only and scored on the other two scenarios;
* cross_h*_v*: the smallest and largest leave-one-out horizontal and
  vertical sigmas crossed, to separate the two (scored on scenarios 0, 3);
* reference_rtk: the RTK headline figure, to show what an understated sigma
  does (not a candidate declaration: the measured offsets rule it out).

Each declaration is replayed with the default node configuration, and once
more with the world model's box size gate disabled (a diagnostic, not a
proposed setting).

Usage: python -I run_all.py $V2V4REAL_DIR/test1 $V2V4REAL_DIR/converted OUT.json
"""

import glob
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
REPLAY = os.path.join(REPO, 'target', 'release', 'v2v4real-replay')
PY = sys.executable

# RT3000 v3 datasheet: SPS position 1.5 m CEP (multi-frequency), heading 0.1 deg 1 sigma.
# CEP to per-axis sigma for a circular Gaussian: CEP = 1.1774 sigma.
NOMINAL = {'sigma_h': round(1.5 / 1.1774, 2), 'sigma_v': 2.0, 'sigma_yaw_deg': 0.1}
RTK = {'sigma_h': 0.01, 'sigma_v': 0.02, 'sigma_yaw_deg': 0.1}
DELAY_MS = 100


def short(name):
    return name.split('_2022-03-15-')[1]


def convert(sc_dir, out_dir, decl):
    subprocess.run([PY, '-I', os.path.join(HERE, 'convert.py'), sc_dir, out_dir,
                    '--sigma-h', str(decl['sigma_h']), '--sigma-v', str(decl['sigma_v']),
                    '--sigma-yaw-deg', str(decl['sigma_yaw_deg'])], check=True, stdout=subprocess.DEVNULL)


def replay(out_dir, ablate=None):
    res = os.path.join(out_dir, 'result%s.json' % ('_' + ablate if ablate else ''))
    cmd = [REPLAY, '--dir', out_dir, '--out', res, '--delay-ms', str(DELAY_MS)]
    if ablate:
        cmd += ['--ablate', ablate]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    with open(res) as fh:
        return json.load(fh)


def loo_sigma(truth):
    """Sigma per car such that two independent cars' relative offset is calibrated:
    E|d_h|^2 = 4 sigma_h^2 (two axes, two cars), E[d_z^2] = 2 sigma_v^2."""
    d = np.array([p for f in truth['frames'] if f['offset'] for p in f['offset']['pairs_enu']])
    return {'sigma_h': round(float(np.sqrt(np.mean(np.sum(d[:, :2] ** 2, axis=1)) / 4.0)), 2),
            'sigma_v': round(float(np.sqrt(np.mean(d[:, 2] ** 2) / 2.0)), 2),
            'sigma_yaw_deg': NOMINAL['sigma_yaw_deg'], 'pairs': int(len(d))}


def pool(results):
    """Sums counts and pools means over (scenario, car) results."""
    keys_sum = ['frames', 'matched', 'missed', 'duplicates', 'duplicates_split_across_cars', 'false_estimates',
                'false_estimates_with_id_of_a_previously_matched_object', 'matched_not_expected',
                'self_in_own_world', 'detected_by_both_matched', 'detected_by_both_fused_into_one',
                'matched_never_detected_by_own_sensor', 'objects_gained_only_through_sharing',
                'matched_not_detected_by_own_sensor_this_frame', 'objects_with_id', 'id_changes_total',
                'objects_with_id_change', 'n_nees', 'n_box', 'n_both_labeled']
    cars = [c for r in results for c in r['cars']]
    out = {k: int(sum(c[k] for c in cars)) for k in keys_sum}
    m = out['matched']
    out['rms_m'] = float(np.sqrt(sum(c['rms_m'] ** 2 * c['matched'] for c in cars if c['matched']) / m)) if m else None
    for k in ['nees_pos_3d_mean', 'nees_pos_h_2d_mean', 'nees_pos_v_1d_mean']:
        out[k] = float(sum(c[k] * c['n_nees'] for c in cars if c['n_nees']) / out['n_nees'])
    nb = out['n_both_labeled']
    out['nees_pos_3d_mean_both_labeled'] = (float(sum(c['nees_pos_3d_mean_both_labeled'] * c['n_both_labeled']
                                                      for c in cars if c['n_both_labeled']) / nb) if nb else None)
    out['rms_m_both_labeled'] = (float(np.sqrt(sum(c['rms_m_both_labeled'] ** 2 * c['n_both_labeled']
                                                   for c in cars if c['n_both_labeled']) / nb)) if nb else None)
    out['size_nees_3d_mean'] = float(sum(c['size_nees_3d_mean'] * c['n_box'] for c in cars if c['n_box']) / out['n_box'])
    out['heading_err_deg_p95_worst_car'] = max(c['heading_err_deg_p95'] for c in cars)
    out['heading_err_deg_median_by_car'] = [round(c['heading_err_deg_median'], 2) for c in cars]
    out['heading_err_deg_max'] = max(c['heading_err_deg_max'] for c in cars)
    out['id_changes_per_object'] = out['id_changes_total'] / out['objects_with_id'] if out['objects_with_id'] else None
    out['fused_fraction_when_both_detect'] = (out['detected_by_both_fused_into_one'] / out['detected_by_both_matched']
                                              if out['detected_by_both_matched'] else None)
    return out


def strip(r):
    r = json.loads(json.dumps(r))
    for c in r['cars']:
        c.pop('per_frame_matched_missed_dup_false', None)
    return r


def main():
    data, work, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    scen = sorted(glob.glob(os.path.join(data, '*/')))
    names = [os.path.basename(s.rstrip('/')) for s in scen]
    summary = {'delay_ms': DELAY_MS, 'scenarios': [short(n) for n in names], 'declarations': {}}

    def run_decl(label, decl, which):
        entry = {'declared': decl, 'scored_on': [short(names[i]) for i in which], 'runs': {}}
        for ablate in (None, 'no-size-gate'):
            res = []
            for i in which:
                od = os.path.join(work, label, short(names[i]))
                if ablate is None:
                    convert(scen[i], od, decl)
                res.append(replay(od, ablate))
            entry['runs']['default' if ablate is None else ablate] = {
                'pooled': pool(res), 'per_scenario': [strip(r) for r in res]}
        summary['declarations'][label] = entry
        p = entry['runs']['default']['pooled']
        print('%-14s h %.2f v %.2f | matched %d missed %d dup %d false %d | rms %.2f nees3 %.2f both-labeled %.2f (n %d) h %.2f v %.2f | fused %d/%d | idchg/obj %.2f | gained %d'
              % (label, decl['sigma_h'], decl['sigma_v'], p['matched'], p['missed'], p['duplicates'], p['false_estimates'],
                 p['rms_m'], p['nees_pos_3d_mean'], p['nees_pos_3d_mean_both_labeled'], p['n_both_labeled'], p['nees_pos_h_2d_mean'], p['nees_pos_v_1d_mean'],
                 p['detected_by_both_fused_into_one'], p['detected_by_both_matched'], p['id_changes_per_object'],
                 p['objects_gained_only_through_sharing']))

    run_decl('nominal', NOMINAL, range(len(scen)))
    run_decl('reference_rtk', RTK, range(len(scen)))
    # Relative offsets and gps fit per scenario (independent of the declaration).
    truths = []
    for i in range(len(scen)):
        with open(os.path.join(work, 'nominal', short(names[i]), 'truth.json')) as fh:
            truths.append(json.load(fh))
    for k in range(len(scen)):
        decl = loo_sigma(truths[k])
        decl['chosen_from'] = short(names[k])
        run_decl('loo_from_' + short(names[k]), decl, [i for i in range(len(scen)) if i != k])

    # Horizontal and vertical separately: the smallest and largest leave-one-out
    # sigmas crossed, scored on scenarios 0 and 3 (the two with large offsets).
    loo = [loo_sigma(t) for t in truths]
    lo = min(loo, key=lambda d: d['sigma_h'])
    hi = max(loo, key=lambda d: d['sigma_h'])
    for sh in (lo['sigma_h'], hi['sigma_h']):
        for sv in (lo['sigma_v'], hi['sigma_v']):
            decl = {'sigma_h': sh, 'sigma_v': sv, 'sigma_yaw_deg': NOMINAL['sigma_yaw_deg']}
            run_decl('cross_h%.2f_v%.2f' % (sh, sv), decl, [0, 1])

    summary['relative_offset'] = {}
    summary['gps_fit'] = {}
    for i, t in enumerate(truths):
        per = [f['offset'] for f in t['frames'] if f['offset']]
        d = np.array([p for o in per for p in o['pairs_enu']])
        fm = np.array([o['mean_enu'] for o in per])
        summary['relative_offset'][short(names[i])] = {
            'pairs': int(len(d)), 'frames_with_pairs': int(len(per)),
            'pair_mean_enu_m': np.round(d.mean(0), 3).tolist(), 'pair_std_enu_m': np.round(d.std(0), 3).tolist(),
            'pair_abs_h_median_m': round(float(np.median(np.linalg.norm(d[:, :2], axis=1))), 3),
            'pair_abs_h_max_m': round(float(np.max(np.linalg.norm(d[:, :2], axis=1))), 3),
            'frame_mean_abs_h_min_max_m': [round(float(np.min(np.linalg.norm(fm[:, :2], axis=1))), 3),
                                           round(float(np.max(np.linalg.norm(fm[:, :2], axis=1))), 3)],
            'per_frame': [{'frame': f['frame'], 'n': f['offset']['n'],
                           'mean_enu_m': np.round(f['offset']['mean_enu'], 3).tolist()} for f in t['frames'] if f['offset']],
        }
        summary['gps_fit'][short(names[i])] = t['gps_fit']
        summary['gps_fit'][short(names[i])]['anchor'] = t['anchor']
        summary['gps_fit'][short(names[i])]['linked_pairs_class_disagree'] = t['linked_pairs_class_disagree']
        summary['gps_fit'][short(names[i])]['links_to_several_car0_ids'] = t['links_to_several_car0_ids']
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w') as fh:
        json.dump(summary, fh, indent=1)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
