"""Held-out confirmation on V2V4Real test2 and test3 with frozen settings.

The settings below were fixed (docs/results/v2v4real.md, "Held-out
confirmation", commit d301dde) before test2 and test3 were downloaded. Do not
change them after seeing results. The converter, replay and scoring are the
ones used for test1 (run_all.py), unchanged.

Usage: python -I run_heldout.py $V2V4REAL_DIR $V2V4REAL_DIR/converted_heldout OUT.json
(expects $V2V4REAL_DIR/test2 and $V2V4REAL_DIR/test3)
"""

import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_all import convert, pool, replay, strip  # noqa: E402

FROZEN_POSE = {'sigma_h': 1.27, 'sigma_v': 2.0, 'sigma_yaw_deg': 0.1}
FROZEN_BOX = {'box_size_sys_m': 0.18, 'box_yaw_sys_deg': 1.7}


def short(name):
    return name.replace('testoutput_CAV_data_', '')


def disagreement(truth):
    """Size and heading disagreement of objects both cars labeled (reported,
    not used to set anything)."""
    ds, dy, obj = [], [], {}
    for f in truth['frames']:
        for o in f['objects']:
            pc = o['per_car']
            if '0' not in pc or '1' not in pc:
                continue
            d = np.array(pc['0']['size']) - np.array(pc['1']['size'])
            h = (pc['0']['yaw_enu_deg'] - pc['1']['yaw_enu_deg'] + 180.0) % 360.0 - 180.0
            ds.append(d)
            dy.append(h)
            obj.setdefault(o['gid'], []).append(np.r_[np.abs(d), abs(h)])
    if not ds:
        return {'pair_frames': 0, 'objects': 0}
    ds, dy = np.array(ds), np.array(dy)
    ady = np.abs(dy)
    flip = ady > 165.0
    per_obj_max = np.array([np.max(v, axis=0) for v in obj.values()])
    return {
        'pair_frames': int(len(ds)), 'objects': len(obj),
        'size_abs_diff_lwh_m_median': np.round(np.median(np.abs(ds), 0), 3).tolist(),
        'size_abs_diff_lwh_m_p95': np.round(np.percentile(np.abs(ds), 95, axis=0), 3).tolist(),
        'size_abs_diff_lwh_m_max': np.round(np.max(np.abs(ds), 0), 3).tolist(),
        'size_diff_pooled_std_over_sqrt2_m': round(float(np.std(ds.reshape(-1)) / np.sqrt(2)), 3),
        'objects_with_size_diff_over_0p5m': int(np.sum(per_obj_max[:, :3].max(1) > 0.5)),
        'objects_with_size_diff_over_1m': int(np.sum(per_obj_max[:, :3].max(1) > 1.0)),
        'heading_abs_diff_deg_median': round(float(np.median(ady)), 2),
        'heading_abs_diff_deg_p95': round(float(np.percentile(ady, 95)), 2),
        'heading_flip_pair_frames_over_165deg': int(flip.sum()),
        'heading_15_to_165deg_pair_frames': int(np.sum((ady > 15) & ~flip)),
        'objects_with_a_flip': int(np.sum(per_obj_max[:, 3] > 165.0)),
        'heading_diff_std_over_sqrt2_deg_without_flips': round(float(np.std(dy[~flip]) / np.sqrt(2)), 2),
    }


def offsets(truth):
    per = [f['offset'] for f in truth['frames'] if f['offset']]
    if not per:
        return {'pairs': 0}
    d = np.array([p for o in per for p in o['pairs_enu']])
    fm = np.array([o['mean_enu'] for o in per])
    return {
        'pairs': int(len(d)), 'frames_with_pairs': int(len(per)),
        'pair_mean_enu_m': np.round(d.mean(0), 3).tolist(), 'pair_std_enu_m': np.round(d.std(0), 3).tolist(),
        'pair_abs_h_median_m': round(float(np.median(np.linalg.norm(d[:, :2], axis=1))), 3),
        'pair_abs_h_p95_m': round(float(np.percentile(np.linalg.norm(d[:, :2], axis=1), 95)), 3),
        'pair_abs_h_max_m': round(float(np.max(np.linalg.norm(d[:, :2], axis=1))), 3),
        'pair_abs_v_p95_m': round(float(np.percentile(np.abs(d[:, 2]), 95)), 3),
        'frame_mean_abs_h_min_max_m': [round(float(np.min(np.linalg.norm(fm[:, :2], axis=1))), 3),
                                       round(float(np.max(np.linalg.norm(fm[:, :2], axis=1))), 3)],
        'per_frame': [{'frame': f['frame'], 'n': f['offset']['n'],
                       'mean_enu_m': np.round(f['offset']['mean_enu'], 3).tolist()} for f in truth['frames'] if f['offset']],
    }


def main():
    data, work, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    summary = {'frozen': {'pose': FROZEN_POSE, 'box': FROZEN_BOX, 'frozen_in_commit': 'd301dde'},
               'scenarios': {}, 'total': None}
    results = []
    for split in ('test2', 'test3'):
        for sc in sorted(glob.glob(os.path.join(data, split, '*/'))):
            name = short(os.path.basename(sc.rstrip('/')))
            od = os.path.join(work, name)
            convert(sc, od, FROZEN_POSE)
            r = replay(od, None, FROZEN_BOX, tag='_frozen')
            results.append(r)
            with open(os.path.join(od, 'truth.json')) as fh:
                truth = json.load(fh)
            p = pool([r])
            summary['scenarios'][name] = {
                'split': split, 'frames': len(truth['frames']), 'pooled_both_cars': p, 'result': strip(r),
                'gps_fit': dict(truth['gps_fit'], anchor=truth['anchor'],
                                linked_pairs_class_disagree=truth['linked_pairs_class_disagree'],
                                links_to_several_car0_ids=truth['links_to_several_car0_ids']),
                'relative_offset': offsets(truth), 'label_disagreement': disagreement(truth),
            }
            print('%-12s %s frames %d | matched %d missed %d dup %d (split %d) false %d (coast %d) | rms %.2f/%.2f nees %.2f/%.2f h %.2f v %.2f | fused %d/%d | size NEES %.2f | hdg med %s p95 %.1f | id %d/%d | gained %d (%d)'
                  % (name, split, len(truth['frames']), p['matched'], p['missed'], p['duplicates'], p['duplicates_split_across_cars'],
                     p['false_estimates'], p['false_estimates_with_id_of_a_previously_matched_object'],
                     p['rms_m'], p['rms_m_both_labeled'] or float('nan'), p['nees_pos_3d_mean'], p['nees_pos_3d_mean_both_labeled'] or float('nan'),
                     p['nees_pos_h_2d_mean'], p['nees_pos_v_1d_mean'], p['detected_by_both_fused_into_one'], p['detected_by_both_matched'],
                     p['size_nees_3d_mean'], p['heading_err_deg_median_by_car'], p['heading_err_deg_p95_worst_car'],
                     p['id_changes_total'], p['objects_with_id'], p['objects_gained_only_through_sharing'], p['matched_never_detected_by_own_sensor']))
    summary['total'] = pool(results)
    t = summary['total']
    print('TOTAL matched %d missed %d dup %d (split %d) false %d | rms %.2f/%.2f nees %.2f/%.2f | fused %d/%d | size NEES %.2f | id %d/%d = %.2f'
          % (t['matched'], t['missed'], t['duplicates'], t['duplicates_split_across_cars'], t['false_estimates'], t['rms_m'],
             t['rms_m_both_labeled'], t['nees_pos_3d_mean'], t['nees_pos_3d_mean_both_labeled'], t['detected_by_both_fused_into_one'],
             t['detected_by_both_matched'], t['size_nees_3d_mean'], t['id_changes_total'], t['objects_with_id'], t['id_changes_per_object']))
    with open(out_path, 'w') as fh:
        json.dump(summary, fh, indent=1)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
