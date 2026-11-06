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

# Loads map_server and beluga_amcl into the container robot_drivers_to_persist_sim.launch.py starts
# when config.yaml sets localization: true (and not slam: true).

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def _check_map_exists(context):
    map_file = LaunchConfiguration("map").perform(context)
    if not os.path.exists(map_file):
        raise RuntimeError(
            f"Map file not found: {map_file}. Set slam: true in config.yaml, save /map with "
            "`ros2 run nav2_map_server map_saver_cli -f lunar_map`, install the resulting YAML "
            "and image under maps/, then set localization: true; or pass map:=<path>."
        )
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    container_name = LaunchConfiguration("container_name")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    map_server = ComposableNode(
        package="nav2_map_server",
        plugin="nav2_map_server::MapServer",
        name="map_server",
        parameters=[configured_params],
        remappings=remappings,
    )

    def lifecycle_manager(node_names):
        return ComposableNode(
            package="nav2_lifecycle_manager",
            plugin="nav2_lifecycle_manager::LifecycleManager",
            name="lifecycle_manager_localization",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": node_names,
                }
            ],
        )

    load_localization_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            map_server,
            ComposableNode(
                package="beluga_amcl",
                plugin="beluga_amcl::AmclNode",
                name="amcl",
                parameters=[configured_params],
                remappings=remappings,
            ),
            lifecycle_manager(["map_server", "amcl"]),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "container_name", default_value="localization_container"
            ),
            OpaqueFunction(function=_check_map_exists),
            load_localization_nodes,
        ]
    )
