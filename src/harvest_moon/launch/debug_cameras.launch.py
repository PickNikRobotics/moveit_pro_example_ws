from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pylon_launch = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'launch',
        'pylon_ros2_camera.launch.py'
    )

    zed_launch = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    )

    harvest_moon_dir = get_package_share_directory('harvest_moon')

    return LaunchDescription([
        # Basler camera 1 — isolated scope so camera_id doesn't leak to siblings
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pylon_launch),
                launch_arguments={
                    'node_name': 'pylon_ros2_camera_node',
                    'camera_id': 'basler_cam_1',
                    'config_file': os.path.join(harvest_moon_dir, 'config', 'basler_cam_1.yaml'),
                    'mtu_size': '1500',
                    'startup_user_set': 'Default',
                }.items(),
            ),
        ]),
        # Basler camera 2
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pylon_launch),
                launch_arguments={
                    'node_name': 'pylon_ros2_camera_node',
                    'camera_id': 'basler_cam_2',
                    'config_file': os.path.join(harvest_moon_dir, 'config', 'basler_cam_2.yaml'),
                    'mtu_size': '1500',
                    'startup_user_set': 'Default',
                }.items(),
            ),
        ]),
        # ZED X stereo camera
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch),
                launch_arguments={
                    'camera_name': 'zed_x',
                    'camera_model': 'zedx',
                    'serial_number': '47845360',
                    'camera_id': '-1',
                    'publish_urdf': 'true',
                    'publish_tf': 'true',
                    'publish_map_tf': 'false',
                    'ros_params_override_path': os.path.join(harvest_moon_dir, 'config', 'zed_x_overrides.yaml'),
                }.items(),
            ),
        ]),
    ])
