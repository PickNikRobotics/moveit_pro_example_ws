"""Launch the pi0 topic bridge alongside kinova_sim MoveIt Pro.

Publishes JointTrajectory on /pi0_trajectory, which the GetPi0Trajectory BT
behavior (inside the objective server) picks up and feeds to ExecuteTrajectory.
This routes execution through MoveIt's internal pipeline — no DDS isolation
issues — so it must be paired with the "Run Pi0 Policy" objective.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    bridge_script = (
        "/home/ubuntu/user_ws/src/moveit_pro_kinova_configs"
        "/kinova_sim/scripts/pi0_bridge_topic.py"
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "bash", "-c",
                f"pip install --quiet openpi-client websockets && "
                f"python3 {bridge_script} --prompt 'pick up the red cube'"
            ],
            output="screen",
            name="pi0_topic_bridge",
        ),
    ])
