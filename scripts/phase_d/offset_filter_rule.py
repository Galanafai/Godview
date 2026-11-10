"""Phase D: the rule for the offset filter's parameters, from V2V4Real val inputs.

Uses only val labels and detections (truth.json, attribution.json, car*.jsonl of
`convert_detections.py`); nothing from a GodView run. All translations are "the
translation to add to car B's reports so that they land in car A's frame" for
A = car 0, B = car 1, in the shared map frame (m).

* q (random-walk power spectral density of the offset, m^2/s, horizontal and
  vertical): the offset between the cars' frames per frame is the per-frame
  median over objects labeled by both cars of (label of car 0 - label of car 1)
  (at least 3 objects). For a lag of 2 s inside a scenario,
  q = max(0, E[(d(t + 2 s) - d(t))^2] - 2 var_noise) / 2 s per axis, with
  var_noise the within-frame label scatter over the number of objects. Horizontal
  q is the mean of East and North.
* sigma_obj (m): the scatter of per-object detection offsets (detection by car 0
  minus detection by car 1 of one object, attributed to the same label) around
  their per-frame median (frames with at least 3 such objects, recording cars
  excluded), as 1.4826 x MAD, horizontal (East and North pooled) and vertical.
* sigma_link (m): the same scatter of an agent-agent link around the per-frame
  object median: (a) car 1's detection of car 0 against car 0's self position,
  (b) car 0's detection of car 1 against car 1's self position, horizontal; the
  self position is the lidar position plus the footprint offset below.
* footprint: the median centre offset in the lidar frame (x forward, y left) and
  size of the other car's label of a car (the label within 4 m of its lidar
  position), over val.
* gate: chi-square 99.9 percent, 3 degrees of freedom (16.27), standard.
* init sigma: max_offset / 3 = 8 / 3 m per axis (the alignment search radius).
* max age (s): how long an estimate counts as current without a measurement,
  1 / q_h clamped to [1, 10] (the time for the random walk to reach 1 m).

Usage (python3): offset_filter_rule.py INPUTS_DIR OUT.json
"""
import glob
import json
import os
import sys

import numpy as np

LAG = 20          # frames, 2 s at 10 Hz
DT = 0.1


def mad_sd(x):
    x = np.asarray(x, float)
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if len(x) else float('nan')


def main():
    inputs, out = sys.argv[1:3]
    d_series = {}
    var_noise_num = [[], [], []]
    n_objs = []
    foot_off, foot_size = [], []
    scen_data = {}
    for sd in sorted(glob.glob(inputs + '/*')):
        s = os.path.basename(sd)
        T = json.load(open(sd + '/truth.json'))
        A = json.load(open(sd + '/attribution.json'))['frames']
        cars = [[json.loads(l) for l in open(sd + '/car%d.jsonl' % c) if l.strip()] for c in (0, 1)]
        scen_data[s] = (T, A, cars)
        series = []
        for i, fr in enumerate(T['frames']):
            both = [o for o in fr['objects'] if len(o['labeled_by']) == 2 and o['cav'] is None]
            if len(both) >= 3:
                d = np.array([np.array(o['per_car_pos']['0']) - np.array(o['per_car_pos']['1']) for o in both])
                series.append((i, np.median(d, axis=0)))
                for k in range(3):
                    var_noise_num[k].append(np.var(d[:, k]) / len(d))
            else:
                series.append((i, None))
            # footprint of each recording car, from the other car's label of it
            for k in (0, 1):
                ego = np.array(fr['ego_odom'][k])
                yaw = np.radians(cars[k][i]['agent_yaw_deg'])
                for o in fr['objects']:
                    if o['cav'] == k and str(1 - k) in o['per_car_pos']:
                        v = np.array(o['per_car_pos'][str(1 - k)]) - ego
                        c, sn = np.cos(yaw), np.sin(yaw)
                        foot_off.append([c * v[0] + sn * v[1], -sn * v[0] + c * v[1]])
                        foot_size.append(o['size'])
        d_series[s] = series
    # q from lagged differences
    diffs = [[], [], []]
    for s, series in d_series.items():
        for i in range(len(series) - LAG):
            a, b = series[i][1], series[i + LAG][1]
            if a is not None and b is not None:
                for k in range(3):
                    diffs[k].append((b[k] - a[k]) ** 2)
    tau = LAG * DT
    noise = [float(np.mean(v)) for v in var_noise_num]
    msd = [float(np.mean(v)) for v in diffs]
    q = [max(0.0, (msd[k] - 2 * noise[k]) / tau) for k in range(3)]
    q_h, q_v = (q[0] + q[1]) / 2, q[2]
    foot_off, foot_size = np.array(foot_off), np.array(foot_size)
    off_x, off_y = float(np.median(foot_off[:, 0])), float(np.median(foot_off[:, 1]))
    # per-object and link scatter
    res_obj_h, res_obj_v, res_link = [], [], []
    link_bias = []
    for s, (T, A, cars) in scen_data.items():
        for i, fr in enumerate(T['frames']):
            gid_to = {}
            for c in (0, 1):
                for d, g in zip(cars[c][i]['detections'], A[i][c]):
                    if g is not None:
                        gid_to.setdefault((c, g), []).append(np.array([d['x'], d['y'], d['z']]))
            objs = [o for o in fr['objects'] if o['cav'] is None and (0, o['gid']) in gid_to and (1, o['gid']) in gid_to]
            if len(objs) < 3:
                continue
            t_i = np.array([gid_to[(0, o['gid'])][0] - gid_to[(1, o['gid'])][0] for o in objs])
            med = np.median(t_i, axis=0)
            r = t_i - med
            res_obj_h += r[:, :2].ravel().tolist()
            res_obj_v += r[:, 2].tolist()
            # links
            ego = [np.array(x) for x in fr['ego_odom']]
            yaws = [np.radians(cars[c][i]['agent_yaw_deg']) for c in (0, 1)]
            selfp = []
            for c in (0, 1):
                cs, sn = np.cos(yaws[c]), np.sin(yaws[c])
                selfp.append(ego[c] + np.array([cs * off_x - sn * off_y, sn * off_x + cs * off_y, 0.0]))
            def det_of(c, k):
                best = None
                gids = {o['gid'] for o in fr['objects'] if o['cav'] == k}
                for g in gids:
                    for v in gid_to.get((c, g), []):
                        if best is None or np.linalg.norm(v[:2] - ego[k][:2]) < np.linalg.norm(best[:2] - ego[k][:2]):
                            best = v
                return best
            d10 = det_of(1, 0)     # car 1's detection of car 0
            d01 = det_of(0, 1)     # car 0's detection of car 1
            if d10 is not None:
                t = selfp[0] - d10
                res_link.append((t - med)[:2].tolist())
            if d01 is not None:
                t = d01 - selfp[1]
                res_link.append((t - med)[:2].tolist())
    res_link = np.array(res_link)
    sigma_obj_h, sigma_obj_v = mad_sd(res_obj_h), mad_sd(res_obj_v)
    sigma_link = mad_sd(res_link.ravel())
    res = {
        'rule': 'docs/results/phase_d_d1_plan.md "Offset filter and ego view: rule and expectations"',
        'q_h_m2_per_s': q_h, 'q_v_m2_per_s': q_v, 'q_by_axis': q, 'lagged_mean_square_by_axis': msd,
        'label_noise_variance_of_the_frame_offset_by_axis': noise, 'lag_s': tau,
        'frames_with_label_offset': int(sum(sum(x[1] is not None for x in v) for v in d_series.values())),
        'lagged_pairs': len(diffs[0]),
        'sigma_obj_h_m': sigma_obj_h, 'sigma_obj_v_m': sigma_obj_v,
        'object_offset_residuals': len(res_obj_v),
        'sigma_link_h_m': sigma_link, 'link_measurements': int(len(res_link)),
        'link_residual_median_east_north_m': np.median(res_link, axis=0).tolist(),
        'gate_chi2_3dof_99.9': 16.27, 'init_sigma_m': 8.0 / 3.0,
        'max_age_s': float(np.clip(1.0 / q_h if q_h > 0 else 10.0, 1.0, 10.0)),
        'footprint_centre_offset_lidar_frame_m_forward_left': [off_x, off_y],
        'footprint_size_lwh_m_median': np.median(foot_size, axis=0).tolist(),
        'footprint_frames': int(len(foot_off)),
    }
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
