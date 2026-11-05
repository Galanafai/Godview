"""Anchor a V2V4Real scenario's map frame on the Earth using the gps field."""
import numpy as np

R_EARTH = 6378137.0


def yaw_deg(T):
    return np.degrees(np.arctan2(T[1, 0], T[0, 0]))


def fit_map_to_enu(poses, gps):
    """Fits ENU = R(rot) * map_xy + t over all given (lidar_pose, gps) pairs.

    poses: list of 4x4 lidar_pose matrices; gps: matching list of
    [lat, lon, alt, heading_rad, ...]. Cars alternate in the order given
    (index % n_cars) for the per-car summaries. Returns the rotation of the
    map x axis counterclockwise from East (deg), the geodetic point of the
    map origin, and the residuals.
    """
    P = np.array([T[:2, 3] for T in poses])
    Z = np.array([T[2, 3] for T in poses])
    G = np.array([g[:3] for g in gps], dtype=float)
    lat_c, lon_c = G[:, 0].mean(), G[:, 1].mean()
    e = np.radians(G[:, 1] - lon_c) * R_EARTH * np.cos(np.radians(lat_c))
    n = np.radians(G[:, 0] - lat_c) * R_EARTH
    Q = np.c_[e, n]
    pm, qm = P.mean(0), Q.mean(0)
    H = (P - pm).T @ (Q - qm)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    t = qm - R @ pm
    res = (P @ R.T + t) - Q
    rh = np.linalg.norm(res, axis=1)
    h_off = np.mean(G[:, 2] - Z)
    rv = (Z + h_off) - G[:, 2]
    rot = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    # Map origin in ENU is t; convert to geodetic (small-area approximation).
    lat0 = lat_c + np.degrees(t[1] / R_EARTH)
    lon0 = lon_c + np.degrees(t[0] / (R_EARTH * np.cos(np.radians(lat_c))))
    heading = np.array([np.degrees(g[3]) for g in gps])
    pyaw = np.array([yaw_deg(T) for T in poses]) + rot
    hoff = (heading - pyaw + 180) % 360 - 180
    return {
        'rot_deg': float(rot), 'lat0': float(lat0), 'lon0': float(lon0), 'h0': float(h_off),
        'rms_h': float(np.sqrt(np.mean(rh ** 2))), 'max_h': float(rh.max()),
        'rms_v': float(np.sqrt(np.mean(rv ** 2))),
        'rms_h_per': lambda k: [float(np.sqrt(np.mean(rh[i::k] ** 2))) for i in range(k)],
        'heading_offset': lambda k: [float(np.median(hoff[i::k])) for i in range(k)],
        'residuals_h': rh,
    }
