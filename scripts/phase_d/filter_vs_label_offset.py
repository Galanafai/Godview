"""Phase D (explanation only): how close the ego view's translation estimates are to the
cross-car label offset, the only independent reference for the two cars' relative
pose error in V2V4Real.

Reference per frame: the median over objects labeled by both cars (recording cars
excluded, at least 3) of (label of car 0 - label of car 1), horizontal: the translation
to add to car 1's reports to land in car 0's frame. Estimate: the dump's `alignments`
of car 0's world (the entry for source 1, odometry frame). Reports the median error
and the share of frames within 0.5 m, for frames with both.

Usage (python3): filter_vs_label_offset.py INPUTS_DIR RUN_DIR [RUN_DIR ...]
"""
import glob
import json
import os
import sys

import numpy as np


def main():
    inputs = sys.argv[1]
    for run in sys.argv[2:]:
        errs, mags, n_ref, n_est = [], [], 0, 0
        for f in sorted(glob.glob(run + '/*.dump.jsonl')):
            s = os.path.basename(f)[:-len('.dump.jsonl')]
            T = json.load(open(os.path.join(inputs, s, 'truth.json')))
            for line in open(f):
                e = json.loads(line)
                fr = T['frames'][e['frame']]
                both = [o for o in fr['objects'] if len(o['labeled_by']) == 2 and o['cav'] is None]
                if len(both) < 3:
                    continue
                ref = np.median([np.array(o['per_car_pos']['0'])[:2] - np.array(o['per_car_pos']['1'])[:2] for o in both], axis=0)
                n_ref += 1
                est = [np.array(x['t_odom'][:2]) for x in e['alignments'][0] if x['src'] == 1]
                if est:
                    n_est += 1
                    errs.append(float(np.linalg.norm(est[0] - ref)))
                    mags.append(float(np.linalg.norm(ref)))
        errs = np.array(errs)
        print('%-26s frames with a label offset %d, with an estimate %d; error median %.2f m, p90 %.2f m, within 0.5 m %.2f; median label offset %.2f m'
              % (os.path.basename(run), n_ref, n_est, np.median(errs), np.percentile(errs, 90), np.mean(errs < 0.5), np.median(mags)))


if __name__ == '__main__':
    main()
