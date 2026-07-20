# Persistent driver-side processes for lab_sim.
#
# Launched by MoveIt Pro via the simulated_robot_driver_persist_launch_file
# config field. Lives in the "drivers to persist" lifecycle so it survives
# agent_bridge restarts — that matters because the Meta Quest headset holds
# a TCP socket open against ros_tcp_endpoint, and tearing that down would
# force the user to reconnect from the headset.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ros_tcp_endpoint_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint",
        emulate_tty=True,
        parameters=[
            {"ROS_IP": "192.168.1.34"},
        ],
        output="screen",
    )

    # Debug-only static TF linking the disjoint quest TF root into the robot tree
    # at the world frame. The numerical values are wrong — quest and world have
    # no real geometric relationship — but the link satisfies the canTransform()
    # check inside marker_utils::transformPoseToBaseFrame() so VisualizePose calls
    # against quest-frame poses (controller markers in v11) can render.
    #
    # No control path uses this transform: the v11 kinematic math composes the
    # controller delta in quest and applies it to the EE in world directly,
    # bypassing TF entirely. The lie is contained to RViz markers.
    #
    # Before shipping v11, remove the quest-frame VisualizePose calls from the
    # objective and delete this node — neither will be needed.
    static_tf_world_to_quest = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_world_to_quest",
        output="log",
        arguments=[
            "0", "0", "0",  # x y z (identity translation; the link is geometrically meaningless)
            "0", "0", "0",  # yaw pitch roll
            "world", "quest",
        ],
    )

    return LaunchDescription([ros_tcp_endpoint_node, static_tf_world_to_quest])
