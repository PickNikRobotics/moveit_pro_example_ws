# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Live per-objective progress + structured /rosout capture for the
objectives integration test.

Pytest's default ``--capture=fd`` redirects fds 1 and 2, so when CTest kills
the test on timeout (TIMEOUT 600 in CMakeLists.txt) all per-test output is
lost and the CI log shows nothing past pytest's "collected N items" header.

The runtest hooks write directly to fd 2, bypassing the capture, so the CI
log always shows which objective was running when the timeout fired and how
long each completed objective took.

The ``capture_rosout`` fixture launches ``capture_rosout.py`` as a
subprocess so no rclpy state lives in the pytest process. The integration
test's ``execute_objective_resource`` fixture (in
moveit_pro_test_utils/objective_test_fixture.py) starts the backend via
``multiprocessing.Process(target=run_launch_files).start()`` — on Linux
that forks pytest. Any rclpy.init() called in pytest beforehand leaves the
forked child with a half-initialized DDS participant (FDs inherited, the
spinner thread does not survive fork), which breaks /robot_description
discovery and causes BehaviorContext to SIGABRT on backend boot.
"""

import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import pytest

_started_at: dict[str, float] = {}

# Topics recorded per test for post-mortem playback (moveit_pro#20427).
# Kinematic state + controller tracking only -- no images or point clouds, so
# a 90 s test stays in the tens of MB. Topics that do not exist on a given
# distro (e.g. the humble-only `state` topic) are simply never captured;
# rosbag2 subscribes on discovery and an absent topic is not an error.
_BAG_TOPICS = [
    "/clock",
    "/joint_states",
    "/tf",
    "/tf_static",
    "/joint_trajectory_controller/controller_state",
    "/joint_trajectory_controller/state",
    "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
    "/joint_trajectory_controller/follow_joint_trajectory/_action/feedback",
    "/platform_velocity_controller/odometry",
    "/platform_velocity_controller/odom",
]

# Bags land under build/<pkg>/test_results/ (pytest's cwd is the package
# build dir), which the CI reusable workflow already uploads as the
# `test-results-*` artifact with `if: always()` -- no workflow change needed.
_BAG_ROOT = Path.cwd() / "test_results" / "bags"

# Objectives under investigation in moveit_pro#20427: keep their bags even on
# PASS so a healthy execution is available to diff against a hung one.
_ALWAYS_KEEP = re.compile(r"plan_path_along_surface")


@pytest.hookimpl(tryfirst=True, hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """Stash each phase's report on the item so fixtures can read the outcome.

    The ``record_rosbag`` teardown uses ``item.rep_call`` to decide whether a
    test failed and its bag must be kept.
    """
    outcome = yield
    rep = outcome.get_result()
    setattr(item, "rep_" + rep.when, rep)


@pytest.fixture(autouse=True)
def record_rosbag(request):
    """Record a per-test rosbag of joint/TF/controller state for playback.

    Diagnostic instrumentation for the `Plan Path Along Surface` execute-hang
    (moveit_pro#20427). ``ros2 bag record`` runs as a subprocess for the same
    fork/DDS reason as ``capture_rosout`` -- no rclpy state may live in the
    pytest process (see module docstring).

    Keep policy on teardown: bags of failing tests and of the objectives under
    investigation are kept (they ride the existing test-results CI artifact);
    everything else is deleted so the artifact stays small. Playback locally:
    ``ros2 bag play <dir> --clock`` + RViz with ``use_sim_time``.
    """
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", request.node.name)[-80:]
    bag_dir = _BAG_ROOT / f"{safe_name}-{datetime.now().strftime('%H%M%S')}"
    _BAG_ROOT.mkdir(parents=True, exist_ok=True)

    proc = subprocess.Popen(
        ["ros2", "bag", "record", "--include-hidden-topics", "-o", str(bag_dir)]
        + _BAG_TOPICS,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    # Give the recorder a moment to create subscriptions so the first motion
    # of the objective is not missed.
    time.sleep(2.0)

    yield

    # SIGINT lets rosbag2 flush and finalize the bag; SIGKILL fallback in
    # case the recorder wedges.
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)

    rep_call = getattr(request.node, "rep_call", None)
    rep_setup = getattr(request.node, "rep_setup", None)
    failed = (rep_call is not None and rep_call.failed) or (
        rep_setup is not None and rep_setup.failed
    )
    if not failed and not _ALWAYS_KEEP.search(request.node.name):
        shutil.rmtree(bag_dir, ignore_errors=True)


def pytest_runtest_logstart(nodeid, location):
    _started_at[nodeid] = time.monotonic()
    os.write(2, f"  START   {nodeid}\n".encode())


def pytest_runtest_logreport(report):
    if report.when != "call":
        return
    nodeid = report.nodeid
    start = _started_at.get(nodeid)
    elapsed_str = f"{time.monotonic() - start:.1f}s" if start is not None else "n/a"
    outcome = report.outcome.upper()  # PASSED / FAILED / SKIPPED
    line = f"  {outcome:7s} {nodeid} ({elapsed_str})"
    if report.failed and report.longrepr:
        reason = str(report.longrepr).splitlines()[-1][:200]
        line += f"\n    └─ {reason}"
    os.write(2, (line + "\n").encode())


@pytest.fixture(scope="session", autouse=True)
def capture_rosout():
    """Run capture_rosout.py as a subprocess to mirror /rosout to NDJSON.

    Subprocess isolation is mandatory — see the module docstring for the
    fork/DDS interaction that requires it. The subprocess has its own
    process image, so any rclpy / DDS state it sets up never reaches the
    pytest process that later gets forked by execute_objective_resource.

    Creates ``$ROS_LOG_DIR`` synchronously here so the directory exists
    before backend nodes (also writing into it) start, and so the
    subprocess's first ``mkdir(exist_ok=True)`` has nothing to race on.
    """
    out_dir = Path(os.environ.get("ROS_LOG_DIR", "."))
    out_dir.mkdir(parents=True, exist_ok=True)

    script = Path(__file__).parent / "capture_rosout.py"
    proc = subprocess.Popen([sys.executable, str(script)])

    try:
        yield
    finally:
        # SIGINT triggers rclpy's default shutdown handler in the
        # subprocess, which lets the script flush the NDJSON file before
        # exiting. SIGKILL fallback in case the script wedges.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
