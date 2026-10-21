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
"""Start the installed mock/sim leader path with a fake at the SDK boundary.

The bridge, reader and ROS services are real; no USB or CAN device is exposed.
Preparation proves HOME/model/native collision scene readiness without engaging.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import DurabilityPolicy, QoSProfile
from moveit_msgs.srv import GetPlanningScene
import xacro


# A fresh interpreter constructs launch actions, never a fork of a DDS participant.
LAUNCH = """
import importlib.util
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from moveit_studio_utils_py.system_config import load_system_config
config = load_system_config(sys.argv[1], Path(sys.argv[2]))[0]
config.ros_global_params['rebot_leader_enabled'] = True
path = Path(get_package_share_directory('rebot_bench_base_config')) / 'launch/rebot_leader.launch.py'
spec = importlib.util.spec_from_file_location('leader_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
nodes = module.configured_nodes(config)
assert len(nodes) == 1  # mock/sim must never launch a real thermal owner
service = LaunchService()
service.include_launch_description(LaunchDescription(nodes))
sys.exit(service.run())
"""


def group_exists(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


@pytest.mark.parametrize("package", ["rebot_bench_base_config", "rebot_bench_sim"])
def test_installed_configured_bridge_and_single_reader_start_and_clean_up(
    package, tmp_path
):
    # Record each SDK owner, including a duplicate reader if one is started.
    marker = tmp_path / "reader-pids"
    (tmp_path / "motorbridge_smart_servo.py").write_text(
        r"""
import os
from pathlib import Path
from types import SimpleNamespace
class FashionStarServo:
    def __init__(self, port, baudrate):
        with Path(__file__).with_name('reader-pids').open('a') as stream:
            stream.write(str(os.getpid()) + '\n')
    def ping(self, motor_id): return True
    def unlock(self, motor_id): pass
    def read_angle(self, motor_id): return SimpleNamespace(raw_deg=0.0)
    def close(self): pass
"""
    )
    env = dict(
        os.environ,
        PYTHONPATH=str(tmp_path) + os.pathsep + os.environ.get("PYTHONPATH", ""),
    )
    config = load_system_config(package, tmp_path)[0]
    robot = config.hardware.robot_description
    mappings = {
        key: str(value) for entry in robot.urdf_params for key, value in entry.items()
    }
    urdf = Path(get_package_share_directory(robot.urdf.package)) / robot.urdf.path
    description = xacro.process_file(str(urdf), mappings=mappings).toxml()

    rclpy.init()
    node = rclpy.create_node("leader_startup_test")
    pub = node.create_publisher(
        String,
        "/robot_description",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    joint_pub = node.create_publisher(JointState, "/joint_states", 10)
    scene_service = node.create_service(
        GetPlanningScene, "/get_planning_scene", lambda request, response: response
    )
    prepare = node.create_client(Trigger, "/rebot_leader_bridge/prepare")
    with (tmp_path / "launch.log").open("w+") as log:
        process = subprocess.Popen(
            [sys.executable, "-c", LAUNCH, package, str(tmp_path)],
            env=env,
            start_new_session=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            pub.publish(String(data=description))
            deadline = time.monotonic() + 20
            prepared = False
            future = None
            while time.monotonic() < deadline and process.poll() is None:
                state = JointState()
                state.header.stamp = node.get_clock().now().to_msg()
                state.name = [f"joint{i}" for i in range(1, 8)]
                state.position = state.velocity = [0.0] * 7
                joint_pub.publish(state)
                rclpy.spin_once(node, timeout_sec=0.02)
                if future is not None and future.done():
                    prepared = future.result().success
                    future = None
                    if prepared:
                        break
                if future is None and prepare.service_is_ready():
                    future = prepare.call_async(Trigger.Request())
            log.seek(0)
            assert prepared, (
                "Configured bridge never became HOME/model/scene ready:\n" + log.read()
            )
            assert marker.exists()
            assert (
                len(marker.read_text().splitlines()) == 1
            ), "Multiple USB reader owners"
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            # Reader owns a separate session; the bridge must reap it itself.
            reader_pids = (
                [int(pid) for pid in marker.read_text().splitlines()]
                if marker.exists()
                else []
            )
            survivors = [
                pid for pid in [process.pid, *reader_pids] if group_exists(pid)
            ]
            for pid in survivors:
                os.killpg(pid, signal.SIGKILL)
            node.destroy_node()
            rclpy.shutdown()
            log.seek(0)
            assert not survivors, "Bridge/reader process group leaked:\n" + log.read()
            assert process.returncode == 0, (
                "Launch did not shut down cleanly:\n" + log.read()
            )
