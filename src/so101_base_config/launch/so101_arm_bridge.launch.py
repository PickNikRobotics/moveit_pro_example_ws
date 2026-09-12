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

"""Launch so101_arm_bridge.py, picking its --real/--fake source.

The ``source`` launch argument controls which: ``auto`` (the default) reads
the same merged config (``moveit_studio_utils_py.system_config.load_system_config``,
the loader ``moveit_pro run`` itself uses - see AGENTS.md's config-inheritance
section) and resolves to ``real`` when ``hardware_interface`` is ``"real"``,
``fake`` otherwise - falling back to ``fake`` (safe on every hardware
interface) if that config cannot be loaded, e.g. this file launched
standalone outside a full instance. ``fake`` or ``real`` forces that source
regardless of config - this is how `Wiggle SO101 Follower` stays runnable on
real hardware even while `Mirror SO101 Follower`'s own ``auto`` instance is
mirroring the leader (see the README's *Wiggle test* section).

``node_name`` lets more than one instance of this launch file run at once,
each under its own node name and so its own ``~/mirror`` trigger-service
address - `runtime.launch.xml` does exactly that for the wiggle-forcing
instance.

``real_slew_rate_rad_s``, ``leader_wrist_roll_offset_rad``, ``leader_timeout_s``,
and ``leader_port`` tune the real-mode leader mirror without a code edit -
see *Tuning the mirror* in the README. All four feed the node parameters of
the same name (``leader_wrist_roll_offset_rad`` becomes one entry of the
``leader_offset_rad`` list parameter, at ``wrist_roll``'s fixed position -
every other joint's offset stays ``0.0``, since it is the only one with a
nonzero bench value). They apply regardless of ``source``, since the node
declares them unconditionally, but only matter once mirroring in ``--real``.
The primary way to change one is ``config/so101_drivers.yaml``, which
``so101_drivers.launch.py`` reads for its own defaults and forwards here -
the literal defaults declared below matter only when this file is launched
directly, bypassing that YAML.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import get_config_package, load_system_config


def select_source(source_arg, hardware_interface):
    """Resolve the bridge's actual source.

    ``"fake"``/``"real"`` force that source regardless of config; ``"auto"``
    (or anything else) defers to ``hardware_interface`` - ``"real"`` only for
    exactly ``"real"``, ``"fake"`` for ``"mock"``, unset, or anything else.
    """
    if source_arg in ("fake", "real"):
        return source_arg
    return "real" if hardware_interface == "real" else "fake"


def read_hardware_interface():
    """Read hardware_interface from the merged config.

    ``USER_WS`` is only set inside a full instance launch (``moveit_pro
    run``); its absence is the one case safe to read as "assume mock/fake" -
    e.g. this file launched standalone, outside a full instance. Any other
    failure to load the config (missing ``MOVEIT_CONFIG_PACKAGE``, a missing
    or malformed ``config.yaml``) means an instance actually is running with
    a broken configuration, and must fail the launch loudly rather than
    silently falling back to the wiggle sine while the operator thinks the
    leader is driving.
    """
    if "USER_WS" not in os.environ:
        return None
    config, _, _ = load_system_config(get_config_package(), Path(os.environ["USER_WS"]))
    params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    return params.get("hardware_interface")


def launch_bridge(context, *args, **kwargs):
    del args, kwargs
    source_arg = LaunchConfiguration("source").perform(context)
    node_name = LaunchConfiguration("node_name").perform(context)
    resolved_source = select_source(source_arg, read_hardware_interface())
    real_slew_rate_rad_s = float(
        LaunchConfiguration("real_slew_rate_rad_s").perform(context)
    )
    wrist_roll_offset_rad = float(
        LaunchConfiguration("leader_wrist_roll_offset_rad").perform(context)
    )
    leader_timeout_s = float(LaunchConfiguration("leader_timeout_s").perform(context))
    leader_port = LaunchConfiguration("leader_port").perform(context)
    # Fixed JOINT_NAMES order (so101_arm_bridge.py): shoulder_pan,
    # shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
    leader_offset_rad = [0.0, 0.0, 0.0, 0.0, wrist_roll_offset_rad, 0.0]
    return [
        Node(
            package="so101_base_config",
            executable="so101_arm_bridge.py",
            name=node_name,
            arguments=[f"--{resolved_source}"],
            parameters=[
                {
                    "real_slew_rate_rad_s": real_slew_rate_rad_s,
                    "leader_offset_rad": leader_offset_rad,
                    "leader_timeout_s": leader_timeout_s,
                    "leader_port": leader_port,
                }
            ],
            output="both",
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source",
                default_value="auto",
                description="'auto' (default) picks fake/real from the "
                "config's hardware_interface; 'fake' or 'real' forces that "
                "source regardless of config.",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="so101_arm_bridge",
                description="Node name, so more than one instance of this "
                "launch file can run at once under distinct trigger-service "
                "addresses.",
            ),
            DeclareLaunchArgument(
                "real_slew_rate_rad_s",
                default_value="4.0",
                description="--real mode: how fast (rad/s) the commanded "
                "target may glide toward the leader's pose.",
            ),
            DeclareLaunchArgument(
                "leader_wrist_roll_offset_rad",
                default_value="1.68",
                description="--real mode: leader_offset_rad's wrist_roll "
                "entry, correcting a leader servo homed off from the "
                "follower - every other joint's offset stays 0.0.",
            ),
            DeclareLaunchArgument(
                "leader_timeout_s",
                default_value="0.5",
                description="--real mode: how long the leader bus may go "
                "without a good read before the follower stops moving.",
            ),
            DeclareLaunchArgument(
                "leader_port",
                default_value="/dev/so101_leader",
                description="--real mode: the leader's Feetech bus device.",
            ),
            OpaqueFunction(function=launch_bridge),
        ]
    )
