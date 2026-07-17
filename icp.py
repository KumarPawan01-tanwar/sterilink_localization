"""
2D ICP for laser scan matching.

Implements the same core ideas as Andrea Censi's Canonical Scan Matcher (CSM):
  * point-to-line metric (PL-ICP) with fallback to vanilla point-to-point ICP
  * initial guess ("first_guess") support
  * maximum correspondence distance gating
  * outlier trimming (discard the worst X% of correspondences)
  * convergence thresholds on translation (epsilon_xy) and rotation (epsilon_theta)

Pure Python + NumPy (+ SciPy cKDTree for fast nearest neighbours, with a
brute-force fallback if SciPy is unavailable).
"""

import numpy as np

try:
    from scipy.spatial import cKDTree
    _HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    _HAVE_SCIPY = False


# ---------------------------------------------------------------------------
# Small 2D rigid-transform helpers
# ---------------------------------------------------------------------------

def transform_matrix(x, y, theta):
    """3x3 homogeneous transform from (x, y, theta)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, x],
                     [s,  c, y],
                     [0.0, 0.0, 1.0]])


def transform_params(T):
    """(x, y, theta) from a 3x3 homogeneous transform."""
    return T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])


def apply_transform(T, points):
    """Apply 3x3 homogeneous transform T to an (N, 2) point array."""
    return points @ T[:2, :2].T + T[:2, 2]


# ---------------------------------------------------------------------------
# Nearest neighbours
# ---------------------------------------------------------------------------

def nearest_neighbor(src, dst):
    """For each point in src (N,2) find nearest point in dst (M,2).

    Returns (distances (N,), indices (N,)).
    """
    if _HAVE_SCIPY:
        tree = cKDTree(dst)
        distances, indices = tree.query(src, k=1)
        return distances, indices
    # brute force fallback
    diff = src[:, None, :] - dst[None, :, :]
    d2 = np.einsum('ijk,ijk->ij', diff, diff)
    indices = np.argmin(d2, axis=1)
    distances = np.sqrt(d2[np.arange(len(src)), indices])
    return distances, indices


# ---------------------------------------------------------------------------
# Closed-form fits
# ---------------------------------------------------------------------------

def best_fit_transform(A, B):
    """Least-squares rigid transform mapping A (N,2) onto B (N,2) via SVD.

    Returns (T 3x3, R 2x2, t (2,)).
    """
    assert A.shape == B.shape
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:          # reflection -> fix
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T, R, t


def point_to_line_fit(src, dst, normals):
    """One linearized point-to-line step (the PL in PL-ICP).

    Minimizes sum_i ( n_i . (R src_i + t - dst_i) )^2 with the small-angle
    approximation R ~ [[1, -th], [th, 1]].  Returns a 3x3 transform.
    """
    nx, ny = normals[:, 0], normals[:, 1]
    px, py = src[:, 0], src[:, 1]

    # rows of the least-squares system  J [tx ty th]^T = r
    J = np.column_stack([nx, ny, nx * (-py) + ny * px])
    r = np.einsum('ij,ij->i', normals, dst - src)

    sol, *_ = np.linalg.lstsq(J, r, rcond=None)
    tx, ty, th = sol
    return transform_matrix(tx, ty, th)


def estimate_normals(points, k=6):
    """Estimate a unit normal per point from its k nearest neighbours
    (PCA: normal = eigenvector of the smallest eigenvalue)."""
    n = len(points)
    k = min(k, n - 1)
    normals = np.zeros_like(points)
    if _HAVE_SCIPY:
        tree = cKDTree(points)
        _, idx = tree.query(points, k=k + 1)
    else:
        diff = points[:, None, :] - points[None, :, :]
        d2 = np.einsum('ijk,ijk->ij', diff, diff)
        idx = np.argsort(d2, axis=1)[:, :k + 1]
    for i in range(n):
        nbrs = points[idx[i]]
        cov = np.cov(nbrs.T)
        w, v = np.linalg.eigh(cov)
        normals[i] = v[:, 0]          # smallest eigenvalue -> normal
    return normals


# ---------------------------------------------------------------------------
# Main ICP
# ---------------------------------------------------------------------------

def icp(A, B,
        init_pose=None,
        max_iterations=20,
        tolerance=1e-6,
        max_correspondence_dist=None,
        outliers_max_perc=1.0,
        use_point_to_line=False,
        epsilon_xy=None,
        epsilon_theta=None):
    """Iterative Closest Point: find T that maps A (N,2) onto B (M,2).

    Parameters mirror the CSM options:
      init_pose               3x3 first guess (CSM: first_guess)
      max_correspondence_dist gate for valid correspondences (CSM: max_correspondence_dist)
      outliers_max_perc       keep only this fraction of best matches (CSM: outliers_maxPerc)
      use_point_to_line       PL-ICP metric instead of vanilla ICP
      epsilon_xy / epsilon_theta  per-iteration convergence thresholds

    Returns (T 3x3, mean_error, iterations).
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    src = A.copy()
    T_total = np.eye(3) if init_pose is None else np.array(init_pose, dtype=float)
    src = apply_transform(T_total, src)

    normals = estimate_normals(B) if use_point_to_line else None

    prev_error = None
    mean_error = np.inf
    i = 0
    for i in range(1, max_iterations + 1):
        distances, indices = nearest_neighbor(src, B)

        mask = np.ones(len(src), dtype=bool)
        if max_correspondence_dist is not None:
            mask &= distances < max_correspondence_dist
        if outliers_max_perc < 1.0 and mask.sum() > 3:
            thresh = np.quantile(distances[mask], outliers_max_perc)
            mask &= distances <= thresh
        if mask.sum() < 3:
            break  # not enough correspondences to constrain the solution

        s = src[mask]
        d = B[indices[mask]]

        if use_point_to_line:
            T_step = point_to_line_fit(s, d, normals[indices[mask]])
        else:
            T_step, _, _ = best_fit_transform(s, d)

        src = apply_transform(T_step, src)
        T_total = T_step @ T_total

        mean_error = distances[mask].mean()
        dx, dy, dth = transform_params(T_step)
        if epsilon_xy is not None and epsilon_theta is not None:
            if np.hypot(dx, dy) < epsilon_xy and abs(dth) < epsilon_theta:
                break
        elif prev_error is not None and abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return T_total, mean_error, i
