"""Phase D1 step 1: export each car's PointPillars detections, per frame.

Runs the V2V4Real late-fusion checkpoint on every car of every frame, exactly
as the late-fusion baseline does, and writes each car's boxes in its own lidar
frame with scores. Per car the post-processing is the baseline's own:
sigmoid score > score_threshold (0.2 in the checkpoint config), rotated NMS at
nms_thresh (0.15), then the GT range mask (x within +-100 m, y within +-40 m).
The only difference from late fusion is that NMS runs per car in its own frame
instead of once over both cars' projected boxes.

Also written per frame: each car's lidar_pose, the car-to-ego transform the
baseline uses, and the dataset's ground truth (both cars' labels, deduplicated
by the dataset's own rule, in the ego car's frame), so that the dataset's AP
can be recomputed later without the data loader.

As a check it recomputes, with the dataset's eval_utils, No Fusion AP (ego
car's export only) and Late Fusion AP (both cars' exports projected to the ego
frame, then joint NMS), to compare with inference.py.

Data YAML files are read with the safe V2VLoader (scripts/v2v4real) instead
of the repository's yaml.Loader.

Usage (V2V4Real env, PYTHONPATH=<V2V4Real repo>):
  python export_detections.py --model_dir DIR --data_dir DIR --out FILE.jsonl
"""
import argparse
import gzip
import json
import os
import random
import sys
from types import SimpleNamespace

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'v2v4real'))
import v2v4real_io  # noqa: E402

import opencood.hypes_yaml.yaml_utils as yaml_utils  # noqa: E402
import opencood.data_utils.datasets.basedataset as basedataset  # noqa: E402

basedataset.load_yaml = lambda path, opt=None: v2v4real_io.load_yaml(path)

from opencood.data_utils.datasets import build_dataset  # noqa: E402
from opencood.tools import train_utils  # noqa: E402
from opencood.utils import box_utils, eval_utils  # noqa: E402


def frame_key(dataset, idx):
    """(scenario folder, timestamp, ego cav id, [cav ids]) for a dataset index."""
    si = 0
    for i, ele in enumerate(dataset.len_record):
        if idx < ele:
            si = i
            break
    db = dataset.scenario_database[si]
    ti = idx if si == 0 else idx - dataset.len_record[si - 1]
    ts = dataset.return_timestamp_key(db, ti)
    cavs = list(db.keys())
    ego = [c for c in cavs if db[c]['ego']][0]
    return si, os.path.basename(dataset.scenario_folders[si]), ts, ego, cavs


def decode_own_frame(pp, content, out):
    """Baseline per-car post-processing with an identity transform.

    Returns boxes (N, 7) as x, y, z, l, w, h, yaw, corners (N, 8, 3) and
    scores (N,). The network regresses x, y, z, h, w, l, yaw (config order
    'hwl'); box_utils.boxes_to_corners_3d reorders columns 3 to 5 of its input
    tensor in place to l, w, h, and the library's post_process relies on the
    same call, so the returned boxes are in l, w, h order. Checked: they equal
    box_utils.corner_to_center(corners) on all val detections.
    """
    anchor_box = content['anchor_box']
    prob = F.sigmoid(out['psm'].permute(0, 2, 3, 1)).reshape(1, -1)
    batch_box3d = pp.delta_to_boxes3d(out['rm'], anchor_box)
    mask = torch.gt(prob, pp.params['target_args']['score_threshold']).view(1, -1)
    boxes3d = torch.masked_select(batch_box3d[0], mask.unsqueeze(2).repeat(1, 1, 7)[0]).view(-1, 7)
    scores = torch.masked_select(prob[0], mask[0])
    if len(boxes3d) == 0:
        return boxes3d, boxes3d.new_zeros((0, 8, 3)), scores
    corners = box_utils.boxes_to_corners_3d(boxes3d, order=pp.params['order'])
    keep = box_utils.nms_rotated(corners, scores, pp.params['nms_thresh'])
    boxes3d, corners, scores = boxes3d[keep], corners[keep], scores[keep]
    m = box_utils.get_mask_for_boxes_within_range_torch(corners)
    return boxes3d[m], corners[m], scores[m]


def new_stat():
    return {0.5: {'tp': [], 'fp': [], 'gt': 0}, 0.7: {'tp': [], 'fp': [], 'gt': 0}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model_dir', required=True)
    ap.add_argument('--data_dir', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--check_library', type=int, default=50,
                    help='compare own decoding with the library post_process '
                         'on the first N frames')
    opt = ap.parse_args()

    random.seed(opt.seed)
    np.random.seed(opt.seed)
    torch.manual_seed(opt.seed)

    hypes = yaml_utils.load_yaml(None, SimpleNamespace(model_dir=opt.model_dir))
    hypes['validate_dir'] = opt.data_dir
    assert hypes['fusion']['core_method'] == 'LateFusionDataset'
    assert not hypes['wild_setting']['async'] and not hypes['wild_setting']['loc_err']

    dataset = build_dataset(hypes, visualize=False, train=False)
    loader = DataLoader(dataset, batch_size=1, num_workers=opt.workers,
                        collate_fn=dataset.collate_batch_test, shuffle=False,
                        pin_memory=False, drop_last=False,
                        worker_init_fn=lambda w: np.random.seed(opt.seed * 1000 + w))
    model = train_utils.create_model(hypes).cuda()
    epoch, model = train_utils.load_saved_model(opt.model_dir, model)
    model.eval()
    pp = dataset.post_processor

    stat_nf, stat_late = new_stat(), new_stat()
    n_det = {}
    mismatches = 0
    os.makedirs(os.path.dirname(os.path.abspath(opt.out)), exist_ok=True)
    with gzip.open(opt.out, 'wt') as f:
        for idx, batch in enumerate(loader):
            si, scen, ts, ego_id, cav_ids = frame_key(dataset, idx)
            db = dataset.scenario_database[si]
            with torch.no_grad():
                batch = train_utils.to_device(batch, 'cuda')
                cars, proj_corners, proj_scores = [], [], []
                for key, content in batch.items():
                    cav_id = ego_id if key == 'ego' else key
                    out = model(content)
                    boxes, corners, scores = decode_own_frame(pp, content, out)
                    if idx < opt.check_library:
                        ident = dict(content)
                        ident['transformation_matrix'] = torch.eye(4, device='cuda')
                        lib_c, lib_s = pp.post_process({key: ident}, {key: out})
                        if lib_c is None:
                            ok = len(corners) == 0
                        else:
                            ok = (lib_c.shape == corners.shape and
                                  torch.allclose(lib_c, corners) and
                                  torch.allclose(lib_s, scores))
                        mismatches += 0 if ok else 1
                    T = content['transformation_matrix']
                    if len(corners):
                        proj_corners.append(box_utils.project_box3d(corners, T))
                        proj_scores.append(scores)
                    pose = v2v4real_io.load_yaml(db[cav_id][ts]['yaml'])['lidar_pose']
                    cars.append({
                        'cav_id': cav_id,
                        'is_ego': key == 'ego',
                        'lidar_pose': np.asarray(pose, dtype=float).tolist(),
                        'T_to_ego': T.cpu().numpy().tolist(),
                        # x y z l w h yaw: centre (m), length along the
                        # heading, width, height (full sizes, m), yaw (rad,
                        # not wrapped; front/back is not determined by the
                        # network, which has no direction classifier).
                        'box': boxes.cpu().numpy().round(5).tolist(),
                        'corners': corners.cpu().numpy().round(4).tolist(),
                        'score': scores.cpu().numpy().round(5).tolist(),
                    })
                    n_det[cav_id] = n_det.get(cav_id, 0) + len(scores)

                gt = pp.generate_gt_bbx(batch)
                ego_car = [c for c in cars if c['is_ego']][0]
                ego_corners = torch.tensor(ego_car['corners'], device='cuda').float() \
                    if ego_car['score'] else None
                ego_scores = torch.tensor(ego_car['score'], device='cuda').float() \
                    if ego_car['score'] else None
                # Late Fusion from the per-car exports: joint NMS + range mask
                if proj_corners:
                    pc = torch.vstack(proj_corners)
                    ps = torch.cat(proj_scores)
                    keep = box_utils.nms_rotated(pc, ps, pp.params['nms_thresh'])
                    pc, ps = pc[keep], ps[keep]
                    m = box_utils.get_mask_for_boxes_within_range_torch(pc)
                    pc, ps = pc[m], ps[m]
                else:
                    pc = ps = None
                for th in (0.5, 0.7):
                    eval_utils.caluclate_tp_fp(ego_corners, ego_scores, gt, stat_nf, th)
                    eval_utils.caluclate_tp_fp(pc, ps, gt, stat_late, th)

            rec = {'idx': idx, 'scenario': scen, 'timestamp': ts, 'ego_cav': ego_id,
                   'cars': cars, 'gt_corners_ego': gt.cpu().numpy().round(4).tolist()}
            f.write(json.dumps(rec) + '\n')
            if idx % 200 == 0:
                print('frame', idx, flush=True)

    res = {'frames': idx + 1, 'checkpoint_epoch': epoch,
           'score_threshold': pp.params['target_args']['score_threshold'],
           'nms_thresh': pp.params['nms_thresh'], 'detections_per_cav': n_det,
           'library_decode_mismatches': mismatches,
           'library_decode_frames_checked': min(opt.check_library, idx + 1)}
    for name, st in (('nofusion', stat_nf), ('late_from_export', stat_late)):
        a5, _, _ = eval_utils.calculate_ap(st, 0.5)
        a7, _, _ = eval_utils.calculate_ap(st, 0.7)
        res[name] = {'ap_50': a5, 'ap_70': a7}
    print(json.dumps(res, indent=1))
    with open(opt.out.replace('.jsonl.gz', '') + '_summary.json', 'w') as g:
        json.dump(res, g, indent=1)


if __name__ == '__main__':
    main()
