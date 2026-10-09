"""Tests for the pure parts of scripts/odometry_joint_state_publisher.py. No ROS."""

import math
import random
import statistics
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from odometry_joint_state_publisher import (  # noqa: E402
    NoisyOdometry,
    twist_in_base,
    wrap,
)

PARAMS = dict(
    distance_error_at_10m=0.05,
    heading_error_at_10m=0.0175,
    heading_error_per_turn=0.0175,
    teleport_distance=0.5,
    teleport_angle=0.5,
)
START = (0.0, 0.0, 0.0)


def straight_path(length, steps):
    x0, y0, yaw = START
    return [
        (
            x0 + math.cos(yaw) * length * i / steps,
            y0 + math.sin(yaw) * length * i / steps,
            yaw,
        )
        for i in range(steps + 1)
    ]


def run(model, path):
    pose = None
    for x, y, yaw in path:
        pose, _ = model.update(x, y, yaw)
    return pose


def test_first_message_starts_at_the_true_pose():
    pose, jumped = NoisyOdometry(**PARAMS).update(*START)
    assert pose == START and not jumped


def test_noise_disabled_passes_odometry_through():
    path = straight_path(10.0, 200)
    pose = run(NoisyOdometry(noise_enabled=False, **PARAMS), path)
    assert pose == pytest.approx(path[-1])


def test_parked_base_does_not_drift():
    # Jitter below the noisy-step threshold is applied exactly, so nothing accumulates.
    rng = random.Random(9)
    model = NoisyOdometry(seed=3, **PARAMS)
    x0, y0, yaw0 = START
    for _ in range(5000):
        truth = (
            x0 + rng.uniform(-3e-4, 3e-4),
            y0 + rng.uniform(-3e-4, 3e-4),
            yaw0 + rng.uniform(-4e-4, 4e-4),
        )
        pose, _ = model.update(*truth)
    assert pose == pytest.approx(truth)


def test_teleport_resets_drift_to_the_new_pose():
    model = NoisyOdometry(seed=1, **PARAMS)
    run(model, straight_path(10.0, 200))
    pose, jumped = model.update(8.0, 7.2, 3.0)
    assert jumped and pose == pytest.approx((8.0, 7.2, 3.0))


def test_drift_matches_the_configured_error_and_ignores_the_rate():
    # 1-sigma after a 10 m straight drive, at two message rates.
    end = straight_path(10.0, 1)[-1]
    for steps in (200, 1000):
        along, heading = [], []
        for seed in range(300):
            x, y, yaw = run(
                NoisyOdometry(seed=seed, **PARAMS), straight_path(10.0, steps)
            )
            ex, ey = x - end[0], y - end[1]
            along.append(ex * math.cos(START[2]) + ey * math.sin(START[2]))
            heading.append(wrap(yaw - START[2]))
        assert math.isclose(
            statistics.pstdev(along), PARAMS["distance_error_at_10m"], rel_tol=0.15
        )
        assert math.isclose(
            statistics.pstdev(heading), PARAMS["heading_error_at_10m"], rel_tol=0.15
        )


def test_turn_in_place_adds_heading_error_only():
    x0, y0, yaw0 = START
    path = [(x0, y0, wrap(yaw0 + 2.0 * math.pi * i / 120)) for i in range(121)]
    heading = []
    for seed in range(300):
        x, y, yaw = run(NoisyOdometry(seed=seed, **PARAMS), path)
        assert (x, y) == pytest.approx((x0, y0))
        heading.append(wrap(yaw - yaw0))
    assert math.isclose(
        statistics.pstdev(heading), PARAMS["heading_error_per_turn"], rel_tol=0.15
    )


def test_twist_in_base_rotates_world_velocity_into_the_body_frame():
    # Facing +y, a world +y velocity is straight ahead.
    assert twist_in_base(0.0, 0.3, math.pi / 2) == pytest.approx((0.3, 0.0))
    assert twist_in_base(0.3, 0.0, math.pi / 2) == pytest.approx((0.0, -0.3))
