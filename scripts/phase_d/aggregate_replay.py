"""Phase D1: GodView's own metrics, summed over both cars and all scenarios.

Usage: aggregate_replay.py OUT.json NAME=DIR [NAME=DIR ...] [--skip SCENARIO_SUBSTRING]
DIR holds the replay's per-scenario JSON outputs (not the dumps).
"""
import glob
import json
import math
import sys


def agg(d, skip=None):
    T = {}
    sq = sqb = nees = sizenees = 0.0
    for f in sorted(glob.glob(d + '/*.json')):
        if skip and skip in f:
            continue
        r = json.load(open(f))
        for c in r['cars']:
            for k in ('frames', 'matched', 'missed', 'duplicates', 'duplicates_split_across_cars', 'false_estimates',
                      'detected_by_both_matched', 'detected_by_both_fused_into_one',
                      'matched_never_detected_by_own_sensor', 'id_changes_total', 'objects_with_id',
                      'wrong_merges_object_frames', 'split_label_frames', 'false_from_sharing_object_frames',
                      'objects_gained_only_through_sharing', 'n_both_labeled', 'n_box', 'coasting_object_frames'):
                T[k] = T.get(k, 0) + c[k]
            T['world_objects'] = T.get('world_objects', 0) + c['world_objects_per_frame'] * c['frames']
            sq += c['rms_m'] ** 2 * c['matched']
            if c['n_both_labeled']:
                sqb += c['rms_m_both_labeled'] ** 2 * c['n_both_labeled']
                nees += c['nees_pos_3d_mean_both_labeled'] * c['n_both_labeled']
            if c['n_box']:
                sizenees += c['size_nees_3d_mean'] * c['n_box']
    T['rms_m'] = math.sqrt(sq / T['matched'])
    T['rms_m_both_labeled'] = math.sqrt(sqb / T['n_both_labeled'])
    T['nees3d_both_labeled'] = nees / T['n_both_labeled']
    T['size_nees'] = sizenees / T['n_box']
    T['world_objects_per_car_frame'] = T.pop('world_objects') / T['frames']
    T['false_from_sharing_per_car_frame'] = T['false_from_sharing_object_frames'] / T['frames']
    T['gained_object_frames_per_car_frame'] = T['matched_never_detected_by_own_sensor'] / T['frames']
    T['id_changes_per_object'] = T['id_changes_total'] / T['objects_with_id']
    return T


if __name__ == '__main__':
    args = sys.argv[2:]
    skip = None
    if '--skip' in args:
        i = args.index('--skip')
        skip = args[i + 1]
        args = args[:i] + args[i + 2:]
    out = {}
    for a in args:
        name, d = a.split('=', 1)
        out[name] = agg(d, skip)
        print(name, json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in out[name].items()}))
    json.dump(out, open(sys.argv[1], 'w'), indent=1)
