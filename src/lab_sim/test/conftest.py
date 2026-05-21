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

"""Live per-objective progress for objectives_integration_test.

Pytest's default ``--capture=fd`` redirects fds 1 and 2, so when CTest kills
the test on timeout (TIMEOUT 600 in CMakeLists.txt) all per-test output is
lost and the CI log shows nothing past pytest's "collected N items" header.

These hooks write directly to fd 2, bypassing the capture, so the CI log
always shows which objective was running when the timeout fired and how long
each completed objective took — the information needed to triage flakes,
budget overruns, and runner regressions.
"""

import os
import time

_started_at: dict[str, float] = {}


def pytest_runtest_logstart(nodeid, location):
    _started_at[nodeid] = time.monotonic()
    os.write(2, f"  START   {nodeid}\n".encode())


def pytest_runtest_logreport(report):
    if report.when != "call":
        return
    nodeid = report.nodeid
    elapsed = time.monotonic() - _started_at.get(nodeid, time.monotonic())
    outcome = report.outcome.upper()  # PASSED / FAILED / SKIPPED
    line = f"  {outcome:7s} {nodeid} ({elapsed:.1f}s)"
    if report.failed and report.longrepr:
        reason = str(report.longrepr).splitlines()[-1][:200]
        line += f"\n    └─ {reason}"
    os.write(2, (line + "\n").encode())
