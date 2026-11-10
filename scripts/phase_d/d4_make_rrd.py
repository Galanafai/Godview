"""Phase D4: build the Rerun recording from the scene package.

Usage (python -I, rerun venv): d4_make_rrd.py PACKAGE_DIR OUT.rrd
Entities: world/static (points), world/moving (points), world/truth (label boxes), world/ego_view (vehicle 1's
view: solid green, ghost grey, text id), world/agent{i}/detections, world/agent{i} (lidar position), and one
pinhole plus encoded image per camera. Detections are dataset annotations with noise, not a trained detector.
"""
import glob
import json
import os
import sys

import numpy as np
import rerun as rr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d4_scene_analysis import pose_matrix, rot  # noqa: E402

PEDS = {'43': (255, 60, 60), '58': (255, 200, 0), '30': (40, 120, 255), '46': (60, 220, 60)}


def boxes(objs, key_pos, key_size, key_yaw):
    c = [o[key_pos] for o in objs]
    s = [np.array(o[key_size]) / 2 for o in objs]
    q = [rr.Quaternion(xyzw=list(_quat(o[key_yaw]))) for o in objs]
    return c, s, q


def _quat(yaw):
    h = np.radians(yaw) / 2
    return (0, 0, np.sin(h), np.cos(h))


def main():
    pk, out = sys.argv[1:3]
    man = json.load(open(os.path.join(pk, 'manifest.json')))
    cal = json.load(open(os.path.join(pk, 'calibration.json')))
    origin = np.array(man['origin_map'])
    rr.init('godview_d4_scene', spawn=False)
    rr.save(out)
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    st = np.load(os.path.join(pk, 'points', 'static_odom.npy'))
    rr.log('world/static', rr.Points3D(st, colors=(120, 120, 130), radii=0.04), static=True)
    mov = np.load(os.path.join(pk, 'points', 'moving_odom.npz'))
    f0, f1 = man['frames']
    v1 = man['vehicle_of_interest']['index']
    for i in range(f0, f1 + 1):
        fr = '%06d' % i
        rec = json.load(open(os.path.join(pk, 'frames', fr + '.json')))
        rr.set_time('frame', sequence=i)
        rr.set_time('time', duration=rec['t_s'])
        rr.log('world/moving', rr.Points3D(mov[fr], colors=(255, 160, 40), radii=0.05))
        tr = rec['truth']
        rr.log('world/truth', rr.Boxes3D(centers=[o['pos'] for o in tr], half_sizes=[np.array(o['size']) / 2 for o in tr],
                                          rotation_axis_angles=[rr.RotationAxisAngle(axis=(0, 0, 1), degrees=o['yaw_enu_deg']) for o in tr],
                                          labels=[o['gid'] for o in tr],
                                          colors=[PEDS.get(o['gid'], (90, 90, 90)) for o in tr]))
        ev = rec['ego_view']
        cols = [(40, 200, 80) if o['status'] == 'solid' else (170, 170, 170) for o in ev]
        rr.log('world/ego_view', rr.Boxes3D(centers=[[o['x'], o['y'], o['z']] for o in ev],
                                             half_sizes=[[o['box']['l'] / 2, o['box']['w'] / 2, o['box']['h'] / 2] for o in ev],
                                             rotation_axis_angles=[rr.RotationAxisAngle(axis=(0, 0, 1), degrees=o['box']['yaw_deg']) for o in ev],
                                             labels=[f"{o['id']} {o['status']} src={o['contributing_agents']}" + (' MERGED' if o['merged_labels'] else '') for o in ev],
                                             colors=cols))
        for a in rec['agents']:
            k = f"world/agent{a['index']}"
            rr.log(k, rr.Points3D([a['ego_odom']], colors=(255, 255, 255), radii=0.5))
            d = a['detections']
            if d:
                rr.log(k + '/detections', rr.Points3D([[x['x'], x['y'], x['z']] for x in d], colors=(0, 180, 255), radii=0.15))
            else:
                rr.log(k + '/detections', rr.Clear(recursive=False))
        for key, c in cal.items():
            folder, cam = key.split('/')
            c = c if c.get('static') else c.get(fr)
            if c is None:
                continue
            T = pose_matrix(np.array(c['lidar_pose_map'])) @ np.array(c['extrinsic_cam_to_lidar'])
            T[:3, 3] -= origin
            ent = f'world/cam/{folder}/{cam}'
            rr.log(ent, rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]))
            K = np.array(c['intrinsic'])
            rr.log(ent, rr.Pinhole(image_from_camera=K, width=1920, height=1080, image_plane_distance=3.0))
            p = os.path.join(pk, 'cameras', folder, cam, fr + '.jpeg')
            rr.log(ent, rr.EncodedImage(path=p))
    print('ok')


if __name__ == '__main__':
    main()
