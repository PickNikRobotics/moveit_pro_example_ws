# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Offline math/window tests; no ROS nodes or hardware."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import xacro

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fit_gravity", PACKAGE / "scripts/fit_gravity.py")
fit_gravity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit_gravity)


@pytest.fixture
def model():
    return fit_gravity.GravityModel(xacro.process_file(
        str(PACKAGE / "description/rebot.urdf.xacro"), mappings={"hardware_interface": "mock"}).toxml())


def samples():
    t = np.arange(100) * .01 + 10
    # Seven measured positions followed by seven measured efforts.
    q = np.tile([.1, .8, 1.2, .2, .1, -.2, .009], (100, 1))
    effort = np.sin(np.arange(100)[:, None] + np.arange(7)[None, :]) * .01 + 1
    return np.column_stack([t, q, effort]), np.column_stack([t, np.zeros((100, 3))])


def test_valid_static_window():
    s, h = samples()
    result = fit_gravity.window_attempt(s, h, 10)
    assert result["count"] == 100
    assert not result["errors"]


@pytest.mark.parametrize("case", ["constant", "comm", "drift", "missing", "nan", "health_gap"])
def test_bad_window_rejected(case):
    s, h = samples()
    if case == "constant":
        s[:, 8] = 1.0
    elif case == "comm":
        h[15, 1] = -2
    elif case == "drift":
        s[-1, 1] += .004
    elif case == "missing":
        s = s[5:]
    elif case == "nan":
        s[5, 8] = np.nan
    else:
        h = h[10:]
    assert fit_gravity.window_attempt(s, h, 10)["errors"]


def test_synthetic_fit_holds_out_every_dataset_and_reload_agrees(model, tmp_path):
    rng = np.random.default_rng(303)
    expected = model.parameters.copy()
    expected[model.first_moments] += rng.normal(0, .015, len(model.first_moments))
    rows = []
    for dataset in range(2):
        for pose in range(1, 25):
            q = rng.uniform([-.3, .5, .5, -.8, -.4, -.7], [.3, 2.4, 3.7, .9, .4, .7])
            approach = {"q_rad": q.tolist(), "tau_Nm": (model.regressor(q) @ expected).tolist()}
            rows.append({"pose": pose, "dataset": dataset, "approaches": [approach]})
    parameters, report = fit_gravity.fit(model, rows)
    assert report["rank"] == 10
    assert set(report["holdout"]["pose_ids"]) == {5, 10, 15, 20}
    assert report["holdout"]["count"] == 8
    assert max(report["holdout"]["fitted"]["rmse_Nm"]) < 1e-10
    assert np.array_equal(parameters[::4], model.parameters[::4])
    path = tmp_path / "calibration.urdf"
    model.write_calibration(path, parameters)
    reloaded = fit_gravity.GravityModel(path.read_text())
    assert reloaded.geometry() == model.geometry()
    for _ in range(20):
        q = rng.uniform([-.3, .5, .5, -.8, -.4, -.7], [.3, 2.4, 3.7, .9, .4, .7])
        np.testing.assert_allclose(reloaded.regressor(q) @ reloaded.parameters,
                                   model.regressor(q) @ expected, atol=1e-10)


def test_named_dataset_retries_once_then_skips(monkeypatch, model):
    from types import SimpleNamespace as Message

    s, h = samples()
    good, _ = samples()
    first = s.copy()
    first[:, 8] = 1.0  # First attempt is constant; second succeeds.
    retry = good.copy()
    retry[:, 0] += 1
    invalid = good.copy()
    invalid[:, 0] += 10
    invalid[:, 8] = 1.0
    invalid_retry = invalid.copy()
    invalid_retry[:, 0] += 1
    rows = np.vstack([first, retry, invalid, invalid_retry])

    def timestamp(t):
        return Message(sec=int(t), nanosec=round((t-int(t))*1e9))

    def messages(path, topics):
        if path == "recording":
            import xml.etree.ElementTree as ET
            yield "/robot_description", Message(data=ET.tostring(model.root, encoding="unicode")), 0
            return
        for label, start in ((25, 10), (26, 20)):
            yield "/rosout", Message(msg=f"Gravity waypoint {label:02d}: static window begins",
                                      stamp=timestamp(start)), start
        for row in rows:
            stamp = row[0]
            yield "/joint_states", Message(name=[*fit_gravity.JOINTS, "joint7"], position=row[1:8],
                                            effort=row[8:15], header=Message(stamp=timestamp(stamp))), stamp
            yield "/robstride_hardware_interface/robstride_state", Message(
                id=list(range(1,8)), enabled=[True]*7, fault_bits=[0]*7, comm_state=0), stamp
            yield "/robstride_hardware_interface/thermal_tripped", Message(data=False), stamp

    monkeypatch.setattr(fit_gravity, "messages", messages)
    _, observations, audit, _, _ = fit_gravity.load_dataset("recording", "health")
    assert [x["pose"] for x in observations] == [25]
    assert observations[0]["approaches"][0]["start_epoch_s"] == 11
    assert [len(x["attempts"]) for x in audit] == [2, 2]
    assert all(x["errors"] for x in audit[1]["attempts"])


def test_underdetermined_fit_refused(model):
    q = [.1, .8, 1.2, .2, .1, -.2]
    rows = [{"pose": i, "approaches": [{"q_rad": q, "tau_Nm": [0] * 6}]} for i in range(1, 25)]
    with pytest.raises(ValueError, match="Underdetermined"):
        fit_gravity.fit(model, rows)
