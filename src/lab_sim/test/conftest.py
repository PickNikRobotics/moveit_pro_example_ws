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
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

_started_at: dict[str, float] = {}


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
