"""Validation of the standard (score-ranked, VOC 2010) AP in ap_eval.py.

1. Hand-checked toy cases (values worked out by hand in the comments).
2. On val, against known implementations:
   * ranking and cumulation: scikit-learn's average_precision_score on the
     same true/false labels, recall rescaled to the number of ground-truth
     boxes, compared with the same step-wise area without the VOC envelope;
   * interpolation: opencood's `voc_ap` (the dataset's VOC 2010 area) on the
     same recall / precision lists against ap_eval's;
   * the standard AP's true positives never exceed the dataset matching's.

Usage (python -I, V2V4Real env): validate_standard_ap.py OPENCOOD EXPORT.jsonl.gz OUT.json
"""
import gzip
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ap_eval as E  # noqa: E402


def box(cx, cy, l=4.0, w=2.0):
    return (E.TEMPLATE * [l, w, 1.5]) + [cx, cy, 0.0]


def frame_stats(dets, gts, th):
    """dets: [(cx, cy, score)], gts: [(cx, cy)] -> (dataset stat, standard stat)."""
    dc = np.array([box(x, y) for x, y, _ in dets]).reshape(-1, 8, 3)
    sc = np.array([s for _, _, s in dets])
    gc = np.array([box(x, y) for x, y in gts]).reshape(-1, 8, 3)
    dp, gp = E.polys(dc), E.polys(gc)
    ious = E.iou_matrix(dp, gp)
    ds, ss = E.new_dataset_stat(), E.new_standard_stat()
    args = (ious, sc, E.dists(dp), E.dists(gp), [False] * len(gts), th, -np.inf, np.inf)
    return args, ds, ss


def toy():
    res = {}
    # Case 1. GT: frame 1 {A at 0, B at 20}, frame 2 {C at 0}. Detections:
    # frame 1: A' (0.9, true), X at 50 (0.7, false), B' (0.6, true); frame 2:
    # C' (0.8, true). Dataset AP: true/false list in frame order TP FP TP TP,
    # precision 1, 1/2, 2/3, 3/4 at recall 1/3, 1/3, 2/3, 1; envelope 1, 3/4,
    # 3/4, 3/4: AP = 1/3 + (1/3)(3/4) + (1/3)(3/4) = 5/6.
    # Standard AP: ranked 0.9 TP, 0.8 TP, 0.7 FP, 0.6 TP: precision 1, 1, 2/3,
    # 3/4 at recall 1/3, 2/3, 2/3, 1; envelope 1, 1, 3/4, 3/4:
    # AP = 1/3 + 1/3 + (1/3)(3/4) = 11/12.
    f1 = frame_stats([(0, 0, 0.9), (50, 0, 0.7), (20, 0, 0.6)], [(0, 0), (20, 0)], 0.5)
    f2 = frame_stats([(0, 0, 0.8)], [(0, 0)], 0.5)
    ds, ss = E.new_dataset_stat(), E.new_standard_stat()
    for args, _, _ in (f1, f2):
        E.dataset_match(*args, ds)
        E.standard_match(*args, ss)
    res['case1'] = {'dataset': E.dataset_ap(ds), 'expected_dataset': 5 / 6,
                    'standard': E.standard_ap(ss), 'expected_standard': 11 / 12}
    assert abs(res['case1']['dataset'] - 5 / 6) < 1e-12 and abs(res['case1']['standard'] - 11 / 12) < 1e-12
    # Case 2. Two ground-truth boxes 1 m apart (4 x 2 m, overlapping); D1 at
    # the first (0.9, IoU 1 with it, 0.6 with the second), D2 at x = 0.4 (0.8):
    # IoU 0.818 with the first, 0.739 with the second, both over 0.7. Dataset:
    # D2 takes the second after the first is removed, both true: AP = 1.
    # Standard: D2's best box is the first, already taken: false; ranked TP FP,
    # 2 ground truths: precision 1, 1/2 at recall 1/2, 1/2: AP = 1/2.
    args, _, _ = frame_stats([(0, 0, 0.9), (0.4, 0, 0.8)], [(0, 0), (1, 0)], 0.7)
    ds, ss = E.new_dataset_stat(), E.new_standard_stat()
    E.dataset_match(*args, ds)
    E.standard_match(*args, ss)
    res['case2'] = {'dataset': E.dataset_ap(ds), 'expected_dataset': 1.0,
                    'standard': E.standard_ap(ss), 'expected_standard': 0.5}
    assert abs(res['case2']['dataset'] - 1.0) < 1e-12 and abs(res['case2']['standard'] - 0.5) < 1e-12
    # Case 3. Ranking is by score across frames, one ground-truth box in total:
    # frame 1 has a false positive scored 0.9 (no ground truth there), frame 2 a
    # true positive scored 0.5. Ranked FP TP: precision 0, 1/2 at recall 0, 1:
    # AP = 1/2 (the dataset's frame-order list would give the same here).
    ss = E.new_standard_stat()
    for dets, gts in (([(30, 0, 0.9)], []), ([(0, 0, 0.5)], [(0, 0)])):
        args, _, _ = frame_stats(dets, gts, 0.5)
        E.standard_match(*args, ss)
    res['case3'] = {'standard': E.standard_ap(ss), 'expected_standard': 0.5}
    assert abs(res['case3']['standard'] - 0.5) < 1e-12
    return res


def val_checks(opencood, export):
    sys.path.insert(0, opencood)
    from sklearn.metrics import average_precision_score
    from opencood.utils import eval_utils
    recs = [json.loads(l) for l in gzip.open(export, 'rt')]
    out = {}
    for th in (0.5, 0.7):
        ss, ds = E.new_standard_stat(), E.new_dataset_stat()
        for r in recs:
            gt = np.array(r['gt_corners_ego'], float).reshape(-1, 8, 3)
            dc = np.array(r['cars'][0]['corners'], float).reshape(-1, 8, 3)
            sc = np.array(r['cars'][0]['score'], float)
            gp, dp = E.polys(gt), E.polys(dc)
            ious = E.iou_matrix(dp, gp)
            a = (ious, sc, E.dists(dp), E.dists(gp), [False] * len(gp), th, -np.inf, np.inf)
            E.standard_match(*a, ss)
            E.dataset_match(*a, ds)
        n_gt, tp = ss['gt'], np.array(ss['tp'])
        s = np.array(ss['score'])
        # Tie-free scores that keep the stable (frame) order among equal scores.
        sj = s - np.arange(len(s)) * 1e-13
        # sklearn on the detections alone has recall relative to the matched
        # ground truth; ground truth never matched is never recalled, so the
        # recall axis is rescaled by tp / n_gt (a constant factor on every step).
        sk = float(average_precision_score(tp, sj)) * tp.sum() / n_gt
        order = np.argsort(-sj, kind='stable')
        t = tp[order]
        tp_c, fp_c = np.cumsum(t), np.cumsum(1 - t)
        rec, pre = tp_c / n_gt, tp_c / (tp_c + fp_c)
        step = float(np.sum(np.diff(np.r_[0.0, rec]) * pre))     # no envelope
        voc_lib, _, _ = eval_utils.voc_ap(rec.tolist(), pre.tolist())
        # ap_eval's own area on the same stable order (scores themselves, ties by frame order)
        mine = E.standard_ap(ss)
        out['iou_%s' % th] = {
            'ground_truth': n_gt, 'detections': len(tp), 'true_positives': int(tp.sum()),
            'dataset_matching_true_positives': int(sum(ds['tp'])),
            'sklearn_average_precision': sk, 'stepwise_area_ap_eval_logic': step,
            'voc_ap_opencood_on_same_curve': float(voc_lib), 'ap_eval_standard_ap': mine}
        assert abs(sk - step) < 1e-9, (sk, step)
        assert abs(voc_lib - mine) < 1e-9, (voc_lib, mine)
        assert int(tp.sum()) <= int(sum(ds['tp']))
    return out


if __name__ == '__main__':
    res = {'toy': toy()}
    res['val_no_fusion_0.2'] = val_checks(sys.argv[1], sys.argv[2])
    json.dump(res, open(sys.argv[3], 'w'), indent=1)
    print(json.dumps(res, indent=1))
