# Copyright 2024 PickNik Inc.
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

# Non-composable launch mode is not supported — hangar_sim always uses use_composition:=True.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    localization = LaunchConfiguration("localization")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    def check_map_exists(context):
        # map_server is loaded in both the localization and map-only paths below,
        # so the map must exist regardless of the localization flag.
        map_file = context.perform_substitution(map_yaml_file)
        if not os.path.exists(map_file):
            raise RuntimeError(
                f"Map file not found: {map_file}\n"
                "Run SLAM first (slam:=True) to build a map, or provide a map path via map:=<path>."
            )
        return []

    map_check = OpaqueFunction(function=check_map_exists)

    # Load map_server and amcl into the shared nav2_container when localization is enabled.
    load_localization_nodes = LoadComposableNodes(
        condition=IfCondition(localization),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="beluga_amcl",
                plugin="beluga_amcl::AmclNode",
                name="amcl",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            ),
        ],
    )

    # Load map_server only (no AMCL) when localization is disabled.
    # A static map->odom TF is expected from the parent launch in this case.
    load_map_server_only = LoadComposableNodes(
        condition=UnlessCondition(localization),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server"],
                    }
                ],
            ),
        ],
    )

    # Both lidars reach AMCL on one topic, a scan at a time. See the node's
    # docstring for why they are not combined into one 360-degree scan.
    scan_localization_relay = Node(
        condition=IfCondition(localization),
        package="hangar_sim",
        executable="scan_localization_relay.py",
        name="scan_localization_relay",
        parameters=[{"use_sim_time": use_sim_time}],
        output="log",
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("map", description="Full path to map yaml file to load")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "container_name",
            default_value="nav2_container",
            description="Name of the component container to load nodes into",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "localization",
            default_value="True",
            description="Run beluga_amcl for map-based localization. Set False to use a static map->odom TF instead.",
        )
    )

    ld.add_action(map_check)
    ld.add_action(scan_localization_relay)
    ld.add_action(load_localization_nodes)
    ld.add_action(load_map_server_only)

    return ld
