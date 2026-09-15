#!/usr/bin/env python3

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

"""Wrap LeRobot's SO-101 calibration for offline, one-arm-at-a-time bring-up.

This script installs nothing - it only shells out to an already-installed
``lerobot``. Before travelling, build the venv it expects:

    python3 -m venv ~/lerobot-venv
    ~/lerobot-venv/bin/pip install 'lerobot[feetech] @ git+https://github.com/huggingface/lerobot@a656a982afe4132eb48a729f06118c115631ac02'

That git ref (which reports itself as lerobot 0.5.2) is what this was verified
against on the bench. It is not a PyPI release: the released 0.5.1 lacks the
feetech motor-position overflow fix (huggingface/lerobot#3373), so its wheels
leave STS3215 motors in multi-turn mode and homing offsets come out
inconsistent. A released tag past that ref (0.6.x) should carry the fix but
was not verified on this bench.

Then, with that venv's Python active, calibrate one arm at a time - support
the arm on something before starting, since calibration drops servo torque:

    python3 calibrate_so101.py follower
    python3 calibrate_so101.py leader

Each run prints the homing pose to hold, then execs LeRobot's own
``lerobot_calibrate`` (interactively - answer its own prompts as they appear;
this script only wraps it), then checks the calibration JSON landed under
``~/.cache/huggingface/lerobot/calibration/``. Both bus adapters need the
operator in the ``dialout`` group; this script checks and warns if not.
"""

import argparse
import grp
import os
import subprocess
import sys
from pathlib import Path

CALIBRATION_DIR = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration"

# lerobot_calibrate takes --robot.* for a robot, --teleop.* for a teleoperator;
# the SO-101 leader is registered as a teleoperator, not a second robot.
ARMS = {
    "follower": {
        "flag_prefix": "robot",
        "type": "so101_follower",
        "default_port": "/dev/so101_follower",
        "default_id": "so101_follower",
        # Captured from ~/.cache/huggingface/lerobot/calibration on the bench
        # workstation; confirm against your own lerobot install if this drifts.
        "calibration_subdir": CALIBRATION_DIR / "robots" / "so_follower",
    },
    "leader": {
        "flag_prefix": "teleop",
        "type": "so101_leader",
        "default_port": "/dev/so101_leader",
        "default_id": "so101_leader",
        "calibration_subdir": CALIBRATION_DIR / "teleoperators" / "so_leader",
    },
}

HOMING_POSE_MESSAGE = """
Hold the arm at its homing pose, then press Enter:
  - every joint at the mid-point of its range of motion
  - wrist_roll centered/neutral (LeRobot's own range-of-motion sweep, which
    runs after homing, does not move wrist_roll, so there is no separate
    check for it here)
  - gripper NEAR-CLOSED - NOT the "roughly half-open" pose LeRobot's own
    prompt suggests. This URDF's gripper zero is the near-closed end of
    travel; homing with the jaw half open puts every gripper command about
    512 ticks off.
"""


def calibration_json_path(arm, calibration_id):
    return ARMS[arm]["calibration_subdir"] / f"{calibration_id}.json"


def check_dialout_group():
    """True if the current process's groups include dialout (or it can't tell)."""
    try:
        dialout_gid = grp.getgrnam("dialout").gr_gid
    except KeyError:
        return True
    return dialout_gid in os.getgroups()


def check_lerobot_available():
    return (
        subprocess.run(
            [sys.executable, "-c", "import lerobot"], capture_output=True
        ).returncode
        == 0
    )


def run_calibration(arm, port, calibration_id):
    spec = ARMS[arm]
    existing = calibration_json_path(arm, calibration_id)
    if existing.is_file():
        print(
            f"{existing} already exists - lerobot_calibrate will ask "
            "'use existing calibration?'; answer 'c' to recalibrate from scratch."
        )
    print(HOMING_POSE_MESSAGE)
    input(
        f"Ready to calibrate the {arm} ({calibration_id} on {port})? Press Enter to start."
    )
    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.lerobot_calibrate",
        f"--{spec['flag_prefix']}.type={spec['type']}",
        f"--{spec['flag_prefix']}.port={port}",
        f"--{spec['flag_prefix']}.id={calibration_id}",
    ]
    mtime_before = existing.stat().st_mtime if existing.is_file() else None
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as error:
        raise SystemExit(
            f"lerobot_calibrate exited with status {error.returncode}. If it "
            "reported 'motor not found', check the bus power supply first: the "
            "leader board runs on 5 V and faults on the 12 V brick (see the "
            "README's 'The leader board runs on 5 V' note)."
        ) from error
    except KeyboardInterrupt:
        raise SystemExit("calibration interrupted before completing.") from None
    if not existing.is_file():
        raise SystemExit(
            f"lerobot_calibrate exited without writing {existing} - "
            "calibration did not complete."
        )
    if existing.stat().st_mtime == mtime_before:
        print(f"Kept existing {existing} (not rewritten).")
    else:
        print(f"Wrote {existing}")


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("arm", choices=sorted(ARMS), help="which bus to calibrate")
    parser.add_argument(
        "--port", default=None, help="serial port (default: this arm's udev symlink)"
    )
    parser.add_argument(
        "--id",
        dest="calibration_id",
        default=None,
        help="LeRobot calibration id (default: so101_<arm>)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = parse_args(argv)

    if not check_dialout_group():
        print(
            "WARNING: not in the 'dialout' group; the Feetech bus will fail to "
            "open. Run: sudo usermod -aG dialout $USER, then log out and back in.",
            file=sys.stderr,
        )

    if not check_lerobot_available():
        raise SystemExit(
            "lerobot is not importable in this Python environment. This script "
            "does not install anything - activate the venv built before "
            "travelling: python3 -m venv ~/lerobot-venv && "
            "~/lerobot-venv/bin/pip install 'lerobot[feetech] @ git+https://"
            "github.com/huggingface/lerobot@a656a982afe4132eb48a729f06118c115631ac02'"
        )

    spec = ARMS[args.arm]
    port = args.port or spec["default_port"]
    calibration_id = args.calibration_id or spec["default_id"]
    run_calibration(args.arm, port, calibration_id)


if __name__ == "__main__":
    main()
