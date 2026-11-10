"""Phase D1: run named tracker configurations on val and summarize them.

For each NAME=FLAGS: the replay on the three val scenarios (with world dumps),
then the dataset AP of car 0's world (scripts/phase_d/ap_eval.py), then one
summary row: AP@0.5 / AP@0.7 (all frames), duplicates, split label-frames,
wrong merges, coasting object-frames, world objects per frame, objects gained
and false objects introduced by sharing (per car-frame).

Usage (python3, not -I; runs the V2V4Real env's python for the AP):
  ablate_val.py OUT_DIR [--no-sharing] [--align] [--variants pass,...]
                NAME=FLAGS [NAME=FLAGS ...]
FLAGS is one string of replay flags, e.g. "--fresh-only --score-mode max".
The committed settings are always passed first.
"""
import argparse
import glob
import json
import os
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
D = '/workspace/data/v2v4real'
BIN = os.path.join(ROOT, 'target/release/v2v4real-replay')
PY = '/workspace/tools/envs/v2v4real/bin/python'
BASE = ['--box-size-sys-m', '0.50', '--box-yaw-sys-deg', '5.7', '--box-center-sys-frac', '0.14',
        '--box-center-sys-z-m', '0.22']
SCEN = sorted(os.listdir(D + '/d1_replay_in/val2'))


def run_replays(out, name, flags, extra):
    d = os.path.join(out, name)
    os.makedirs(d, exist_ok=True)
    procs = []
    for s in SCEN:
        cmd = [BIN, '--dir', f'{D}/d1_replay_in/val2/{s}', '--out', f'{d}/{s}.json',
               '--dump', f'{d}/{s}.dump.jsonl'] + BASE + extra + shlex.split(flags)
        procs.append(subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    for p in procs:
        assert p.wait() == 0, (name, 'replay failed')
    return d


def run_ap(out, name, d, variants, baselines):
    res = os.path.join(out, name + '.ap.json')
    cmd = [PY, '-I', os.path.join(ROOT, 'scripts/phase_d/ap_eval.py'), '--opencood', '/workspace/tools/V2V4Real',
           '--export', D + '/d1_detections/val.jsonl.gz', '--inputs', D + '/d1_replay_in/val2',
           '--out', res, '--dumps', f'{name}={d}', '--score-keys', 'class_prob']
    if variants:
        cmd += ['--variants', variants]
    if not baselines:
        cmd += ['--no-baselines']
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return json.load(open(res))


def replay_totals(d):
    T = {}
    n_obj = 0
    for f in sorted(glob.glob(d + '/*.json')):
        r = json.load(open(f))
        for c in r['cars']:
            for k in ('frames', 'matched', 'missed', 'duplicates', 'false_estimates', 'split_label_frames',
                      'wrong_merges_object_frames', 'coasting_object_frames', 'false_from_sharing_object_frames',
                      'matched_never_detected_by_own_sensor'):
                T[k] = T.get(k, 0) + c[k]
            n_obj += c['world_objects_per_frame'] * c['frames']
    T['world_objects_per_car_frame'] = n_obj / T['frames']
    T['false_from_sharing_per_car_frame'] = T['false_from_sharing_object_frames'] / T['frames']
    T['gained_per_car_frame'] = T['matched_never_detected_by_own_sensor'] / T['frames']
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--no-sharing', action='store_true')
    ap.add_argument('--align', action='store_true')
    ap.add_argument('--variants', default='')
    ap.add_argument('--baselines', action='store_true', help='also score No Fusion and Late Fusion once')
    ap.add_argument('specs', nargs='+')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    extra = (['--no-sharing'] if a.no_sharing else []) + (['--align'] if a.align else [])
    specs = [x.split('=', 1) for x in a.specs]
    with ThreadPoolExecutor(max_workers=len(specs)) as ex:
        dirs = list(ex.map(lambda sp: run_replays(a.out, sp[0], sp[1], extra), specs))
        aps = list(ex.map(lambda t: run_ap(a.out, t[0][0], t[1], a.variants, a.baselines and t[0][0] == specs[0][0]),
                          zip(specs, dirs)))
    summary = {}
    for (name, flags), d, res in zip(specs, dirs, aps):
        row = {'flags': flags, 'replay': replay_totals(d), 'ap': {}}
        for m, v in res['ap'].items():
            row['ap'][m] = {k: [round(100 * v[k]['all'][x], 2) for x in ('ap_50', 'ap_70')] for k in v}
        summary[name] = row
        print(name, json.dumps({m: x['all_frames'] for m, x in row['ap'].items()}), {k: round(x, 2) for k, x in row['replay'].items()
              if k in ('duplicates', 'split_label_frames', 'coasting_object_frames', 'world_objects_per_car_frame',
                       'false_from_sharing_per_car_frame', 'gained_per_car_frame')})
    json.dump(summary, open(os.path.join(a.out, 'summary.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
