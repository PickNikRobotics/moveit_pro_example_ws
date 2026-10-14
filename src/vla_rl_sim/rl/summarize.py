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
"""Collect every results_*.json under the given run directories into one Markdown table."""

import json
import sys
from pathlib import Path

rows = []
for d in map(Path, sys.argv[1:]):
    for f in sorted(d.glob("results_*.json")):
        for key, r in json.loads(f.read_text()).items():
            if isinstance(r, dict) and "success" in r:
                rows.append(
                    (
                        d.name,
                        r["label"],
                        r.get("engine", "mjbatch"),
                        r["episodes"],
                        r["success"],
                        r["ci95"],
                        r.get("mean_first_success_step"),
                    )
                )
print(
    "| run | policy | engine | episodes | success | 95% CI | mean first-success step |"
)
print("|---|---|---|---|---|---|---|")
for run, label, eng, n, p, ci, fs in rows:
    fs = "-" if fs is None else f"{fs:.0f}"
    print(f"| {run} | {label} | {eng} | {n} | {p:.3f} | ±{ci:.3f} | {fs} |")
