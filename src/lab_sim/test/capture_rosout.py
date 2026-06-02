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

"""Standalone /rosout → NDJSON capture, invoked as a subprocess from
src/lab_sim/test/conftest.py.

Runs in its own process so no rclpy state lives in the pytest process. The
integration test's ``execute_objective_resource`` fixture (in
moveit_pro_test_utils/objective_test_fixture.py) launches the backend via
``multiprocessing.Process(target=run_launch_files).start()`` — on Linux that
forks pytest. If pytest has called ``rclpy.init()`` at fork time, the child
inherits the DDS participant's FDs but not its spinner thread, leaving DDS
discovery half-initialized; /robot_description then never reaches
move_group / objective_server_node and BehaviorContext SIGABRTs on boot.

Subscribes to /rosout with the same QoS the rcl logging publisher uses
(depth=1000, RELIABLE, TRANSIENT_LOCAL, KEEP_LAST). Writes one JSON object
per line to ``$ROS_LOG_DIR/rosout.ndjson`` (fallback: current dir).
"""

import json
import os
import sys
import threading
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Log message severity byte → ROS log level name. Use literal ints rather
# than `Log.DEBUG` etc. — the message constants are numpy uint8 in some
# rclpy builds, which hash differently from Python int and miss the dict
# lookup, leaving levels in the NDJSON as bare numbers like "30".
_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# Match the /rosout publisher's QoS exactly so a late-joining subscriber gets
# replayed history (TRANSIENT_LOCAL) up to the publisher's depth. ROS 2's rcl
# logging handler publishes /rosout with depth=1000 / RELIABLE /
# TRANSIENT_LOCAL / KEEP_LAST. If the subscriber is VOLATILE, no historical
# replay happens even if the publisher stores history — both ends must opt in.
_ROSOUT_QOS = QoSProfile(
    depth=1000,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def main() -> int:
    out_dir = Path(os.environ.get("ROS_LOG_DIR", "."))
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "rosout.ndjson"

    write_lock = threading.Lock()

    rclpy.init()
    node = rclpy.create_node("test_rosout_capture")

    with open(out_path, "w", encoding="utf-8") as out_file:

        def on_log(msg: Log) -> None:
            ts = msg.stamp.sec + msg.stamp.nanosec / 1e9
            entry = {
                "ts": ts,
                "level": _LEVEL_NAMES.get(msg.level, str(msg.level)),
                "node": msg.name,
                "msg": msg.msg,
            }
            with write_lock:
                out_file.write(json.dumps(entry, ensure_ascii=False) + "\n")
                out_file.flush()  # don't lose the last lines on a crash

        node.create_subscription(Log, "/rosout", on_log, _ROSOUT_QOS)

        # rclpy.spin returns when SIGINT triggers rclpy.shutdown(); some
        # rclpy versions surface that as KeyboardInterrupt instead.
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
