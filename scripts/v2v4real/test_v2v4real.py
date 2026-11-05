"""Unit tests for the V2V4Real readers and the gps fit (no dataset needed).

Run: python -I scripts/v2v4real/test_v2v4real.py
"""
import base64
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gps_fit import R_EARTH, fit_map_to_enu  # noqa: E402
from v2v4real_io import load_pcd_xyz, load_yaml  # noqa: E402

YAML = """ego_speed: 0
gps:
- !!python/object/apply:numpy.core.multiarray.scalar
  - &id001 !!python/object/apply:numpy.dtype
    args: [f8, false, true]
    state: !!python/tuple [3, <, null, null, null, -1, -1, 0]
  - !!binary |
    {scalar}
lidar_pose: !!python/object/apply:numpy.core.multiarray._reconstruct
  args:
  - !!python/name:numpy.ndarray ''
  - !!python/tuple [0]
  - !!binary |
    Yg==
  state: !!python/tuple
  - 1
  - !!python/tuple [4, 4]
  - *id001
  - false
  - !!binary |
    {matrix}
vehicles: {{}}
"""

EVIL = """x: !!python/object/apply:os.system ["echo pwned"]
"""


class Tests(unittest.TestCase):
    def test_numpy_tags_decode(self):
        m = np.arange(16, dtype='<f8').reshape(4, 4)
        text = YAML.format(scalar=base64.b64encode(np.float64(39.5).tobytes()).decode(),
                           matrix=base64.b64encode(m.tobytes()).decode())
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            f.write(text)
        d = load_yaml(f.name)
        self.assertEqual(d['gps'], [39.5])
        np.testing.assert_array_equal(np.array(d['lidar_pose']), m)
        self.assertEqual(d['vehicles'], {})

    def test_other_python_tags_are_refused(self):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            f.write(EVIL)
        with self.assertRaises(Exception):
            load_yaml(f.name)

    def test_pcd(self):
        with tempfile.NamedTemporaryFile('w', suffix='.pcd', delete=False) as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                    'WIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA ascii\n1 2 3 0\n4 5 6 0\n')
        np.testing.assert_array_equal(load_pcd_xyz(f.name), [[1, 2, 3], [4, 5, 6]])

    def test_gps_fit_recovers_rotation_and_origin(self):
        rot = np.radians(1.3)
        lat0, lon0, h0 = 39.97, -83.02, 220.0
        poses, gps = [], []
        for k in range(50):
            x, y, yaw = -700 + 3.0 * k, 250 - 1.0 * k, 0.4
            T = np.eye(4)
            T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
            T[:3, 3] = [x, y, 4.0]
            e = np.cos(rot) * x - np.sin(rot) * y
            n = np.sin(rot) * x + np.cos(rot) * y
            lat = lat0 + np.degrees(n / R_EARTH)
            lon = lon0 + np.degrees(e / (R_EARTH * np.cos(np.radians(lat0))))
            poses.append(T)
            gps.append([lat, lon, h0 + 4.0, yaw + rot, 0, 0])
        f = fit_map_to_enu(poses, gps)
        self.assertAlmostEqual(f['rot_deg'], 1.3, places=3)
        self.assertLess(f['rms_h'], 0.05)
        self.assertAlmostEqual(f['lat0'], lat0, places=6)
        self.assertAlmostEqual(f['lon0'], lon0, places=6)
        self.assertAlmostEqual(f['h0'], h0, places=6)
        self.assertLess(abs(f['heading_offset'](1)[0]), 0.01)


if __name__ == '__main__':
    unittest.main()
