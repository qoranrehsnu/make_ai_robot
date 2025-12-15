#!/usr/bin/env python3
import numpy as np

try:
    from scipy.spatial import cKDTree
    HAS_SCIPY = True
except Exception:
    HAS_SCIPY = False
    cKDTree = None

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def scan_to_pcd(msg: LaserScan, stride: int = 1) -> np.ndarray:
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    if ranges.size == 0:
        return np.zeros((0, 2), dtype=np.float64)

    idx = np.where(
        (ranges > msg.range_min) &
        (ranges < msg.range_max) &
        np.isfinite(ranges)
    )[0]

    if idx.size == 0:
        return np.zeros((0, 2), dtype=np.float64)

    if stride > 1:
        idx = idx[::stride]

    r = ranges[idx]
    ang = msg.angle_min + idx * msg.angle_increment
    x = r * np.cos(ang)
    y = r * np.sin(ang)
    return np.column_stack((x, y)).astype(np.float64)


def map_to_pcd(map_msg: OccupancyGrid, threshold: int = 50) -> np.ndarray:
    w = map_msg.info.width
    h = map_msg.info.height
    res = map_msg.info.resolution
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y

    data = np.asarray(map_msg.data, dtype=np.int16).reshape((h, w))
    ys, xs = np.where(data >= threshold)

    x = ox + (xs + 0.5) * res
    y = oy + (ys + 0.5) * res
    return np.column_stack((x, y)).astype(np.float64)


def build_kdtree(points: np.ndarray):
    if not HAS_SCIPY or points is None or len(points) == 0:
        return None
    try:
        return cKDTree(points)
    except Exception:
        return None


def compute_normals_2d(points: np.ndarray, sensor_origin=(0.0, 0.0), flip_towards_sensor=True) -> np.ndarray:
    n = len(points)
    if n < 3:
        return np.zeros((n, 2), dtype=np.float64)

    p_prev = np.roll(points, 1, axis=0)
    p_next = np.roll(points, -1, axis=0)
    tangent = p_next - p_prev

    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    norm = np.linalg.norm(normals, axis=1) + 1e-12
    normals = normals / norm[:, None]

    if flip_towards_sensor:
        so = np.array(sensor_origin, dtype=np.float64)
        v = so[None, :] - points
        dot = np.sum(normals * v, axis=1)
        flip = dot < 0
        normals[flip] *= -1.0

    return normals.astype(np.float64)


def icp_2d_point_to_line_with_cov(
    src: np.ndarray,
    tgt: np.ndarray,
    init_T: np.ndarray = None,
    max_iter: int = 8,
    dist_thresh: float = 0.35,
    min_corr: int = 30,
    huber_delta: float = 0.2,
    damping: float = 1e-4,
):
    """
    Point-to-line ICP (2D), returns:
      T (3x3): src -> tgt
      Cov (3x3): covariance of [dx, dy, dtheta]
      info: {num_corr, sigma2, cond}
    """
    if src is None or tgt is None or len(src) < 3 or len(tgt) < 3:
        T = np.eye(3, dtype=np.float64)
        Cov = np.diag([1.0, 1.0, 1.0])
        return T, Cov, {"num_corr": 0, "sigma2": None, "cond": None}

    if init_T is None:
        dx = dy = dth = 0.0
    else:
        dx = float(init_T[0, 2])
        dy = float(init_T[1, 2])
        dth = float(np.arctan2(init_T[1, 0], init_T[0, 0]))

    tree = build_kdtree(tgt)
    normals = compute_normals_2d(tgt, sensor_origin=(0.0, 0.0), flip_towards_sensor=True)

    final_JTJ = None
    final_r = None
    final_m = 0
    final_cond = None

    for _ in range(max_iter):
        c, s = np.cos(dth), np.sin(dth)
        R = np.array([[c, -s], [s, c]], dtype=np.float64)
        t = np.array([dx, dy], dtype=np.float64)

        src_tf = (R @ src.T).T + t

        if tree is not None:
            dists, idx = tree.query(src_tf, k=1)
        else:
            diff = src_tf[:, None, :] - tgt[None, :, :]
            D = np.linalg.norm(diff, axis=2)
            idx = np.argmin(D, axis=1)
            dists = D[np.arange(D.shape[0]), idx]

        mask = dists < dist_thresh
        m = int(np.count_nonzero(mask))
        final_m = m
        if m < min_corr:
            break

        src_m = src_tf[mask]
        tgt_m = tgt[idx[mask]]
        n_m = normals[idx[mask]]

        px = src_m[:, 0]
        py = src_m[:, 1]
        nx = n_m[:, 0]
        ny = n_m[:, 1]

        rx = px - tgt_m[:, 0]
        ry = py - tgt_m[:, 1]
        r = nx * rx + ny * ry

        # Jacobian wrt [dx, dy, dtheta]
        J_theta = nx * (-py) + ny * (px)
        A = np.column_stack((nx, ny, J_theta)).astype(np.float64)
        b = (-r).astype(np.float64)

        # Huber weights
        if huber_delta is not None and huber_delta > 0.0:
            abs_r = np.abs(r)
            w = np.ones_like(abs_r, dtype=np.float64)
            big = abs_r > huber_delta
            w[big] = huber_delta / (abs_r[big] + 1e-12)
            sw = np.sqrt(w)
            A = A * sw[:, None]
            b = b * sw

        JTJ = A.T @ A + damping * np.eye(3, dtype=np.float64)
        JTr = A.T @ b

        try:
            final_cond = float(np.linalg.cond(JTJ))
        except Exception:
            final_cond = None

        try:
            delta = np.linalg.solve(JTJ, JTr)
        except np.linalg.LinAlgError:
            delta = np.linalg.lstsq(JTJ, JTr, rcond=None)[0]

        dx += float(delta[0])
        dy += float(delta[1])
        dth = wrap_angle(dth + float(delta[2]))

        final_JTJ = JTJ
        final_r = r

        if np.linalg.norm(delta) < 1e-4:
            break

    c, s = np.cos(dth), np.sin(dth)
    T = np.eye(3, dtype=np.float64)
    T[:2, :2] = np.array([[c, -s], [s, c]], dtype=np.float64)
    T[:2, 2] = np.array([dx, dy], dtype=np.float64)

    if final_JTJ is None or final_r is None or final_m < 4:
        Cov = np.diag([1.0, 1.0, 1.0])
        return T, Cov, {"num_corr": int(final_m), "sigma2": None, "cond": final_cond}

    dof = max(final_m - 3, 1)
    sigma2 = float((final_r.T @ final_r) / dof)

    try:
        JTJ_inv = np.linalg.inv(final_JTJ)
    except np.linalg.LinAlgError:
        JTJ_inv = np.linalg.pinv(final_JTJ)

    Cov = sigma2 * JTJ_inv
    return T, Cov, {"num_corr": int(final_m), "sigma2": sigma2, "cond": final_cond}