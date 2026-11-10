"""Phase D2: the recording vehicle's footprint and the lidar's place in it, by rule.

The other vehicle's label of a vehicle (the nearest label within 2.5 m of its
lidar position in the other agent's list) gives its box size and centre. Reported
in the vehicle's own lidar frame: the median centre offset (x forward, y left)
and the median size, over all frames and both vehicles. Used for the self
report's footprint in the converter (centre = lidar position + this offset).

Usage (python -I): v2xreal_footprint.py SPLIT_DIR OUT.json
"""
import json
import os
import sys

import numpy as np
import yaml

L = yaml.CSafeLoader


def x_to_world(p):
    x, y, z, roll, yaw, pitch = p
    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    cr, sr = np.cos(np.radians(roll)), np.sin(np.radians(roll))
    cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    m = np.eye(4)
    m[:3, :3] = [[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                 [sp, -cp * sr, cp * cr]]
    m[:3, 3] = [x, y, z]
    return m


def main():
    root, out = sys.argv[1:3]
    off, size, heading = [], [], []
    for scen in sorted(os.listdir(root)):
        sd = os.path.join(root, scen)
        vs = [a for a in sorted(os.listdir(sd), key=int) if int(a) > 0]
        if len(vs) < 2:
            continue
        frames = sorted({n.split('.')[0].split('_')[0] for n in os.listdir(os.path.join(sd, vs[0]))})
        for fi in frames[::2]:
            ys = {a: yaml.load(open(os.path.join(sd, a, fi + '.yaml')), Loader=L) for a in vs}
            for a in vs:
                T = x_to_world(ys[a]['lidar_pose'])
                inv = np.linalg.inv(T)
                other = [b for b in vs if b != a][0]
                best = None
                for v in ys[other]['vehicles'].values():
                    d = np.linalg.norm(np.array(v['location'][:2]) - T[:2, 3])
                    if d < 2.5 and (best is None or d < best[0]):
                        best = (d, v)
                if best:
                    v = best[1]
                    p = inv @ np.r_[v['location'], 1.0]
                    off.append(p[:2])
                    size.append([2 * e for e in v['extent']])
                    yaw_l = (v['angle'][1] - ys[a]['lidar_pose'][4] + 180) % 360 - 180
                    heading.append(yaw_l)
    off, size = np.array(off), np.array(size)
    res = {'frames_with_the_other_vehicles_label': len(off),
           'centre_offset_in_lidar_frame_m_median_xy': np.median(off, axis=0).tolist(),
           'centre_offset_p10_p90_x': [float(np.percentile(off[:, 0], 10)), float(np.percentile(off[:, 0], 90))],
           'centre_offset_p10_p90_y': [float(np.percentile(off[:, 1], 10)), float(np.percentile(off[:, 1], 90))],
           'size_lwh_m_median': np.median(size, axis=0).tolist(), 'size_lwh_m_p10_p90': [np.percentile(size, 10, axis=0).tolist(), np.percentile(size, 90, axis=0).tolist()],
           'heading_of_label_relative_to_lidar_yaw_deg_median_abs': float(np.median(np.abs(heading)))}
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
