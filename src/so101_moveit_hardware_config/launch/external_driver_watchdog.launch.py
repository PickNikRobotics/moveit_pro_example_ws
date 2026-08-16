"""Keep MoveIt Pro's drivers sidecar aligned with the external Pi driver."""

from launch import LaunchDescription
from moveit_studio_utils_py.launch_common import (
    fail_launch_on_process_exit,
    NodeWithAnsiLogging,
)


def generate_launch_description() -> LaunchDescription:
    """Monitor the read-only joint-state heartbeat published by the Pi."""
    watchdog = NodeWithAnsiLogging(
        package="so101_moveit_hardware_config",
        executable="external_driver_watchdog.py",
        name="so101_external_driver_watchdog",
    )
    return LaunchDescription([watchdog, fail_launch_on_process_exit(watchdog)])
