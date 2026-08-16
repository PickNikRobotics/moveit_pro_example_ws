"""Tests for the external Pi driver heartbeat contract."""

from pathlib import Path
import sys


SCRIPTS_DIR = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from external_driver_contract import (  # noqa: E402
    DriverHeartbeat,
    REQUIRED_JOINTS,
    validate_joint_sample,
)


def test_complete_finite_joint_sample_is_valid() -> None:
    names = sorted(REQUIRED_JOINTS)

    assert validate_joint_sample(names, [0.0] * len(names)) is None


def test_invalid_joint_sample_reports_missing_and_non_finite_values() -> None:
    assert "missing" in validate_joint_sample(["Rotation_R"], [0.0])
    names = sorted(REQUIRED_JOINTS)

    assert "non-finite" in validate_joint_sample(names, [float("nan")] * len(names))


def test_heartbeat_distinguishes_startup_from_stale_driver() -> None:
    heartbeat = DriverHeartbeat(started_at=10.0, startup_timeout=30.0, stale_after=5.0)

    assert heartbeat.failure_reason(40.0) is None
    assert "no valid" in heartbeat.failure_reason(40.01)

    heartbeat.record_valid_sample(50.0)
    assert heartbeat.failure_reason(55.0) is None
    assert "stopped" in heartbeat.failure_reason(55.01)
