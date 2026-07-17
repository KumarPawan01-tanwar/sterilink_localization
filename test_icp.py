import time
import numpy as np

import icp

# ---------------------------------------------------------------------------
# Test constants (2D versions of the classic ICP test harness)
# ---------------------------------------------------------------------------
N = 100            # points per synthetic dataset
num_tests = 50     # iterations per test
noise_sigma = .01  # additive Gaussian noise
translation = .1   # max random translation
rotation = .1      # max random rotation (rad)


def random_pose():
    t = (np.random.rand(2) - 0.5) * 2 * translation
    th = (np.random.rand() - 0.5) * 2 * rotation
    return t, th


# ---------------------------------------------------------------------------
# 1. Closed-form best-fit (known correspondences)
# ---------------------------------------------------------------------------
def test_best_fit():
    total_time = 0
    for _ in range(num_tests):
        A = np.random.rand(N, 2)
        t, th = random_pose()
        T_true = icp.transform_matrix(t[0], t[1], th)
        B = icp.apply_transform(T_true, A)
        B += np.random.randn(N, 2) * noise_sigma

        start = time.time()
        T, R, tt = icp.best_fit_transform(B, A)   # maps B back onto A
        total_time += time.time() - start

        C = icp.apply_transform(T, B)
        assert np.allclose(C, A, atol=6 * noise_sigma)
        # T should be the inverse of the applied transform
        assert np.allclose(T @ T_true, np.eye(3), atol=6 * noise_sigma)
    print('best fit time: {:.3}'.format(total_time / num_tests))


# ---------------------------------------------------------------------------
# 2. Vanilla ICP (unknown, shuffled correspondences)
# ---------------------------------------------------------------------------
def test_icp():
    total_time = 0
    for _ in range(num_tests):
        A = np.random.rand(N, 2)
        t, th = random_pose()
        T_true = icp.transform_matrix(t[0], t[1], th)
        B = icp.apply_transform(T_true, A)
        B += np.random.randn(N, 2) * noise_sigma
        np.random.shuffle(B)                       # break correspondence

        start = time.time()
        T, mean_error, its = icp.icp(B, A, tolerance=1e-8, max_iterations=50)
        total_time += time.time() - start

        C = icp.apply_transform(T, B)
        d, _ = icp.nearest_neighbor(C, A)
        assert d.mean() < 6 * noise_sigma
        assert np.allclose(T @ T_true, np.eye(3), atol=0.1)
    print('icp time: {:.3}'.format(total_time / num_tests))


# ---------------------------------------------------------------------------
# 3. Point-to-line ICP on structured (wall-like) data — the CSM use case
# ---------------------------------------------------------------------------
def make_room_scan(pose_x=0.0, pose_y=0.0, pose_th=0.0,
                   n_beams=360, max_range=10.0):
    """Simulate a 360° lidar inside a 8m x 6m rectangular room."""
    walls = [(-4, -3, 4, -3), (4, -3, 4, 3), (4, 3, -4, 3), (-4, 3, -4, -3)]
    angles = np.linspace(-np.pi, np.pi, n_beams, endpoint=False)
    pts = []
    for a in angles:
        wa = a + pose_th
        dx, dy = np.cos(wa), np.sin(wa)
        best = max_range
        for x1, y1, x2, y2 in walls:
            ex, ey = x2 - x1, y2 - y1
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            s = ((x1 - pose_x) * ey - (y1 - pose_y) * ex) / denom
            u = ((x1 - pose_x) * dy - (y1 - pose_y) * dx) / -denom if denom else -1
            # solve for intersection parameters properly
            # ray: p + s*d ; wall: w1 + u*(w2-w1)
            u = ((pose_x - x1) * dy - (pose_y - y1) * dx) / (ex * dy - ey * dx) \
                if abs(ex * dy - ey * dx) > 1e-12 else -1
            if s > 0 and 0 <= u <= 1 and s < best:
                best = s
        if best < max_range:
            # return point in the *laser* frame
            pts.append([best * np.cos(a), best * np.sin(a)])
    return np.array(pts)


def test_plicp_scan_matching():
    """A scan taken after a small robot motion should be matched back."""
    for _ in range(10):
        t, th = random_pose()
        ref = make_room_scan(0, 0, 0)
        cur = make_room_scan(t[0], t[1], th)
        cur += np.random.randn(*cur.shape) * noise_sigma

        T, err, its = icp.icp(cur, ref,
                              max_iterations=30,
                              max_correspondence_dist=0.5,
                              outliers_max_perc=0.9,
                              use_point_to_line=True,
                              epsilon_xy=1e-7, epsilon_theta=1e-7)
        x, y, yaw = icp.transform_params(T)
        # T maps points from the current laser frame into the reference laser
        # frame, i.e. it IS the pose of the new frame in the old one — the
        # robot motion. So (x, y, yaw) should match the true (t, th).
        assert abs(yaw - th) < 0.02, (yaw, th)
        assert np.linalg.norm(np.array([x, y]) - t) < 0.05, (x, y, t)
        # the strongest check: alignment error is tiny
        C = icp.apply_transform(T, cur)
        d, _ = icp.nearest_neighbor(C, ref)
        assert d.mean() < 6 * noise_sigma
    print('pl-icp scan matching ok')


# ---------------------------------------------------------------------------
# 4. First-guess support (large motion recovered thanks to the prediction)
# ---------------------------------------------------------------------------
def test_first_guess():
    ref = make_room_scan(0, 0, 0)
    cur = make_room_scan(0.8, 0.4, 0.35)          # big jump: vanilla ICP struggles
    guess = icp.transform_matrix(0.7, 0.35, 0.3)  # rough odometry prediction

    T, err, _ = icp.icp(cur, ref, init_pose=guess,
                        max_iterations=40,
                        max_correspondence_dist=0.5,
                        use_point_to_line=True,
                        epsilon_xy=1e-7, epsilon_theta=1e-7)
    C = icp.apply_transform(T, cur)
    d, _ = icp.nearest_neighbor(C, ref)
    assert d.mean() < 0.02
    print('first-guess ok, residual {:.4f} m'.format(d.mean()))


if __name__ == "__main__":
    np.random.seed(0)
    test_best_fit()
    test_icp()
    test_plicp_scan_matching()
    test_first_guess()
    print('\nAll tests passed.')
