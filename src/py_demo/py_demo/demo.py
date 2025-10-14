import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time
import threading

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

from moveit_pro import RobotModel, RobotState, kinematics
from moveit_pro.kinematics import cartesian
from moveit_pro.planners import PlanningScene, pro_rrt, create_trajectory_message_from_waypoints

joint_states = {}

def translation_matrix(tx, ty, tz):
    matrix = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])
    return matrix

# Listener callback to update joint_states dictionary
def joint_state_callback(msg: JointState):
    for i in range(len(msg.name)):
        joint_states[msg.name[i]] = msg.position[i]

# This is a helper function to get the current robot state from joint_states
def current_state(model : RobotModel) -> RobotState:
    state = RobotState(model)
    cur_pos = state.joint_positions
    for name in model.get_joint_model_group("manipulator").active_joint_model_names:
        cur_pos[name] = joint_states[name]
    state.joint_positions = cur_pos
    state.update()
    return state

# Send a trajectory to the action server and wait for it to complete
def follow_trajectory(action_client: ActionClient, traj: JointTrajectory):
    cmd = FollowJointTrajectory.Goal()
    cmd.trajectory = traj
    action_client.send_goal(cmd)

# Move to a target joint position using the RRT planner
def move_to(action_client: ActionClient, model: RobotModel, target_position: np.array):
    scene = PlanningScene(model)
    start = current_state(model).get_joint_group_positions("manipulator")
    traj = pro_rrt.plan_trajectory_to_joint_goal("manipulator", start, target_position, scene)
    follow_trajectory(action_client, traj)

# Move to a target end effector pose using IK
def move_to_pose(action_client: ActionClient, model: RobotModel, target_pose: np.array):

    # Create target
    target = kinematics.PoseTarget()
    target.tip_link = model.get_link_model("grasp_link")
    target.root_pose_tip = target_pose 

    # Get current joint values as seed
    model_group = model.get_joint_model_group("manipulator")
    state = current_state(model)
    seed = state.get_joint_group_positions(model_group.name)

    # Create validation function that always returns True
    def always_valid(solution):
        return True

    print("Solving IK")
    solution = kinematics.solve_ik(
        state,
        model_group,
        [target],
        seed,
        timeout=1.0,  # 1 second timeout
        validation_fn=always_valid,
    )
    print(f"IK solved, moving to solution {solution}")
    move_to(action_client, model, solution)

def main():
    rclpy.init()
    node = Node("moveit_pro_py_demo")
    node.create_subscription(JointState, '/joint_states', joint_state_callback, 10)
    action_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print("Waiting for action server")
    action_client.wait_for_server()

    # Wait a moment to start receiving joint states
    time.sleep(1)

    urdf_path = "lab_sim.urdf"
    srdf_path = "lab_sim.srdf"

    print("Loading robot model")
    robot_model = RobotModel(urdf_path, srdf_path)
    # Set acceleration limits since these aren't specified in the URDF
    robot_model.set_group_acceleration_bounds("manipulator", [-2] * 7, [2] * 7)

    initial_state = current_state(robot_model)
    initial_pose = initial_state.get_global_link_transform("grasp_link")

    # Generate a linear path in the X and Z direction
    print("Generating linear path")
    linear_path = [initial_pose, (initial_pose+translation_matrix(1.0, 0.0, -0.2))]
    cartesian_waypoints = cartesian.path_ik(current_state(robot_model), robot_model.get_joint_model_group("manipulator"), ["grasp_link"], linear_path)

    print("Creating trajectory from waypoints")
    traj = create_trajectory_message_from_waypoints(robot_model.get_joint_model_group("manipulator"), cartesian_waypoints)

    print("Executing cartesian trajectory")
    follow_trajectory(action_client, traj)

    print("Moving to pose goal")
    move_to_pose(action_client, robot_model, initial_pose)

    print("Done!")

    thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()