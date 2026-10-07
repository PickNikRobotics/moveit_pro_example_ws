import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Turns the two MuJoCo 3D-lidar clouds into /scan_front and /scan_rear.
    # SICK nanoScan3 values measured from the real scanner's recorded scans:
    # 275 deg sweep, 0.10-40 m. The 6 deg trims drop the beams that graze the
    # chassis window; noise and rounding match the real sensor.
    lidar_flattener = Node(
        package="mobile_fr3_duo_sim",
        executable="lidar_flattener.py",
        name="lidar_flattener",
        parameters=[
            {
                "angle_min": 0.0,
                "angle_max": math.radians(275.0),
                "trim_low_deg": 6.0,
                "trim_high_deg": 6.0,
                "range_min": 0.10,
                "range_max": 40.0,
                "range_noise_stddev": 0.003,
                "range_quantum": 0.001,
            }
        ],
        output="log",
    )

    # AMCL and slam_toolbox each take one scan topic, so the two 275 deg scans
    # are merged into a 360 deg /scan in base_link.
    scan_merger = Node(
        package="mobile_fr3_duo_sim",
        executable="scan_merger.py",
        name="scan_merger",
        output="log",
    )

    # The only publisher of the planar joints (odom -> base_link) and of /odom for
    # Nav2: the simulator's true base pose with drift added, so AMCL has a real
    # correction to make. odometry_drift:=false makes the odometry exact.
    odometry_drift = DeclareLaunchArgument(
        "odometry_drift",
        default_value="true",
        description="Add drift to the simulated odometry.",
    )
    odometry = Node(
        package="mobile_fr3_duo_sim",
        executable="odometry_joint_state_publisher.py",
        name="odometry_joint_state_publisher",
        parameters=[
            {
                "noise_enabled": ParameterValue(
                    LaunchConfiguration("odometry_drift"), value_type=bool
                )
            }
        ],
        output="log",
    )

    # Scan frames: z up, X at beam 0 (the clockwise end of each 275 deg sweep),
    # at the URDF lidar mounting points. Static on base_link, as in hangar_sim.
    static_tf_lidar_front_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_front_ros",
        arguments=[
            "--x",
            "0.3275",
            "--y",
            "0.2175",
            "--z",
            "0.19065",
            "--yaw",
            str(math.radians(-92.5)),
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_front_ROS",
        ],
        output="log",
    )
    static_tf_lidar_rear_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_rear_ros",
        arguments=[
            "--x",
            "-0.3275",
            "--y",
            "-0.2175",
            "--z",
            "0.19065",
            "--yaw",
            str(math.radians(87.5)),
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_rear_ROS",
        ],
        output="log",
    )

    # world -> mj_world anchors the simulator's own frames, such as the scene cameras.
    static_tf_world_to_mj_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_mj_world",
        arguments=["--frame-id", "world", "--child-frame-id", "mj_world"],
        output="log",
    )

    # Frames, Nav2 and the /cmd_vel bridge from mobile_fr3_duo_mock, with AMCL for map -> odom,
    # this package's laser layers on top of its Nav2 parameters, and the true heading for the bridge.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mobile_fr3_duo_mock"), "launch", "navigation.launch.py"]
            )
        ),
        launch_arguments={
            "use_amcl": "true",
            "mock_odometry": "false",
            "params_overlay": PathJoinSubstitution(
                [FindPackageShare("mobile_fr3_duo_sim"), "params", "nav2_params.yaml"]
            ),
            "heading_topic": "/ground_truth/odom",
        }.items(),
    )

    return LaunchDescription(
        [
            odometry_drift,
            lidar_flattener,
            scan_merger,
            odometry,
            static_tf_lidar_front_ros,
            static_tf_lidar_rear_ros,
            static_tf_world_to_mj_world,
            navigation,
        ]
    )
