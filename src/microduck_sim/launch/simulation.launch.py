"""Launch the single policy/physics authority for the simulated robot."""

import os
from launch import LaunchDescription
from moveit_studio_utils_py.launch_common import (
    NodeWithAnsiLogging,
    fail_launch_on_process_exit,
)


def generate_launch_description():
    node = NodeWithAnsiLogging(
        package="microduck_sim",
        executable="microduck_bridge.py",
        prefix=os.environ["MICRODUCK_PYTHON"],
    )
    return LaunchDescription([node, fail_launch_on_process_exit(node)])
