import numpy as np

import icp
from test_icp import make_room_scan


def simulate(base_to_laser, n_steps=40, noise=0.005, seed=1):
    """Drive a robot on a smooth arc and run the matcher pipeline."""
    rng = np.random.default_rng(seed)

    laser_to_base = np.linalg.inv(base_to_laser)

    # matcher state (mirrors LaserScanMatcher)
    f2b = np.eye(3)
    f2b_kf = np.eye(3)
    kf_dist_linear_sq = 0.10 ** 2
    kf_dist_angular = np.deg2rad(10.0)

    # ground-truth robot trajectory: forward + gentle turn
    true_pose = np.eye(3)

    def laser_pose(base_pose):
        return base_pose @ base_to_laser

    lp = laser_pose(true_pose)
    x, y, th = icp.transform_params(lp)
    keyframe = make_room_scan(x, y, th)
    keyframe += rng.normal(0, noise, size=keyframe.shape)

    errors = []
    for step in range(n_steps):
        # move the robot: 4 cm forward, ~1 degree turn per step
        step_T = icp.transform_matrix(0.04, 0.0, np.deg2rad(1.0))
        true_pose = true_pose @ step_T

        lx, ly, lth = icp.transform_params(laser_pose(true_pose))
        curr = make_room_scan(lx, ly, lth)
        curr += rng.normal(0, noise, size=curr.shape)

        # ---- same math as LaserScanMatcher.process_scan --------------------
        pr_ch = f2b @ np.linalg.inv(f2b_kf)
        pr_ch_l = laser_to_base @ np.linalg.inv(f2b) @ pr_ch @ f2b @ base_to_laser

        T, err, _ = icp.icp(curr, keyframe,
                            init_pose=pr_ch_l,
                            max_iterations=30,
                            max_correspondence_dist=0.3,
                            outliers_max_perc=0.9,
                            use_point_to_line=True,
                            epsilon_xy=1e-7, epsilon_theta=1e-7)
        assert err < 0.1, f'matching failed at step {step}: err={err}'

        corr_ch = base_to_laser @ T @ laser_to_base
        f2b = f2b_kf @ corr_ch

        # keyframe management
        cx, cy, cth = icp.transform_params(corr_ch)
        if abs(cth) > kf_dist_angular or cx * cx + cy * cy > kf_dist_linear_sq:
            keyframe = curr
            f2b_kf = f2b.copy()
        # ---------------------------------------------------------------------

        ex, ey, eth = icp.transform_params(np.linalg.inv(true_pose) @ f2b)
        errors.append((np.hypot(ex, ey), abs(eth)))

    return np.array(errors)


def test_pipeline_laser_at_origin():
    errors = simulate(np.eye(3))
    pos_err, ang_err = errors[:, 0], errors[:, 1]
    print(f'laser@origin  final pos err {pos_err[-1]*100:.2f} cm, '
          f'max {pos_err.max()*100:.2f} cm, '
          f'max ang err {np.rad2deg(ang_err.max()):.3f} deg')
    assert pos_err.max() < 0.05      # < 5 cm over the whole run
    assert ang_err.max() < np.deg2rad(2.0)


def test_pipeline_offset_laser():
    """Laser mounted 20 cm forward, 5 cm left, rotated 30° on the base —
    checks the base<->laser propagation is correct."""
    b2l = icp.transform_matrix(0.20, 0.05, np.deg2rad(30.0))
    errors = simulate(b2l, seed=2)
    pos_err, ang_err = errors[:, 0], errors[:, 1]
    print(f'offset laser  final pos err {pos_err[-1]*100:.2f} cm, '
          f'max {pos_err.max()*100:.2f} cm, '
          f'max ang err {np.rad2deg(ang_err.max()):.3f} deg')
    assert pos_err.max() < 0.05
    assert ang_err.max() < np.deg2rad(2.0)


if __name__ == '__main__':
    test_pipeline_laser_at_origin()
    test_pipeline_offset_laser()
    print('\nPipeline tests passed.')
