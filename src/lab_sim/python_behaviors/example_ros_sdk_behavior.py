#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""An example Behavior that talks to ROS and uses the MoveIt Pro Python SDK.

Nothing here is special to Python Behaviors — it is ordinary `rclpy` and `moveit_pro` code.
That is the point: a Behavior can use the same libraries as any other Python program in the
MoveIt Pro environment, and editing it takes effect on the next Objective run.
"""

import os
import tempfile
import time
from typing import List

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from _behavior_math import joint_positions

from moveit_pro import RobotModel, RobotState
from moveit_pro_python_behavior import Behavior, InputPort, NodeStatus, OutputPort

# The node whose parameters carry the robot description. The Objective server is the Behavior's own
# process, and it is the node that serves both the URDF and the SRDF over the parameter service.
ROBOT_DESCRIPTION_NODE = "/objective_server_node"


class ComputeToolPose(Behavior):
    """Report the tool pose for a set of joint positions, via ROS and the MoveIt Pro Python SDK.

    Reads the robot description from the running stack over ROS, builds a kinematic model with
    the MoveIt Pro Python SDK, and computes forward kinematics for the requested link. The ROS
    call is awaited across ticks rather than blocked on, so the rest of the tree keeps running.
    """

    subcategory = "Python Examples"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort(
                "joint_state",
                dict,
                description="A JointState or RobotJointState, e.g. from RetrieveWaypoint.",
            ),
            InputPort(
                "joint_group_name",
                str,
                default="manipulator",
                description="The joint group the positions belong to.",
            ),
            InputPort(
                "link_name",
                str,
                default="",
                description="Link to report. Defaults to the last link of the joint group.",
            ),
            InputPort(
                "timeout",
                float,
                default=10.0,
                description="How long to wait for the robot description, in seconds.",
            ),
            OutputPort(
                "tool_pose",
                "geometry_msgs/msg/PoseStamped",
                description="Pose of the link, in the model frame.",
            ),
        ]

    def on_start(self):
        # A private context keeps this node out of the way of any other rclpy code in the process.
        self._context = Context()
        rclpy.init(context=self._context)
        self._node = rclpy.create_node(
            f"python_behavior_{id(self):x}", context=self._context
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)

        self._client = self._node.create_client(
            GetParameters, f"{ROBOT_DESCRIPTION_NODE}/get_parameters"
        )
        self._request = None
        self._deadline = time.monotonic() + self.get_input("timeout")
        return self.on_running()

    def on_running(self):
        if time.monotonic() > self._deadline:
            self._shut_down()
            self.publish_failure(
                f"Timed out reading the robot description from {ROBOT_DESCRIPTION_NODE}."
            )
            return NodeStatus.FAILURE

        # Spin with a zero timeout so a tick never blocks the tree.
        self._executor.spin_once(timeout_sec=0.0)

        if self._request is None:
            if not self._client.service_is_ready():
                return NodeStatus.RUNNING
            self._request = self._client.call_async(
                GetParameters.Request(
                    names=["robot_description", "robot_description_semantic"]
                )
            )
            return NodeStatus.RUNNING

        if not self._request.done():
            return NodeStatus.RUNNING

        response = self._request.result()
        self._shut_down()
        if (
            response is None
            or len(response.values) != 2
            or not response.values[0].string_value
        ):
            self.publish_failure(
                f"{ROBOT_DESCRIPTION_NODE} returned no robot description."
            )
            return NodeStatus.FAILURE

        return self._report_pose(
            response.values[0].string_value, response.values[1].string_value
        )

    def on_halt(self):
        self._shut_down()

    def _report_pose(self, urdf: str, srdf: str) -> NodeStatus:
        """Build the model with the SDK and report where the link ends up."""
        # RobotModel reads from disk, and what ROS handed us is XML text.
        written: List[str] = []
        try:
            urdf_path = _write_temporary(urdf, ".urdf")
            written.append(urdf_path)
            srdf_path = _write_temporary(srdf, ".srdf")
            written.append(srdf_path)
            model = RobotModel(urdf_path, srdf_path)
            group_name = self.get_input("joint_group_name")
            if not model.has_joint_model_group(group_name):
                self.publish_failure(
                    f"'{group_name}' is not a joint group of this robot. Available groups: "
                    f"{', '.join(model.joint_model_group_names)}."
                )
                return NodeStatus.FAILURE
            group = model.get_joint_model_group(group_name)

            positions = joint_positions(self.get_input("joint_state"))
            missing = [
                joint
                for joint in group.active_joint_model_names
                if joint not in positions
            ]
            if missing:
                # Substituting zeros would produce a confidently wrong pose, which is worse than
                # no pose at all for anything that plans against it.
                self.publish_failure(
                    f"The joint state does not cover '{group_name}'. Missing: {', '.join(missing)}."
                )
                return NodeStatus.FAILURE
            ordered = [positions[joint] for joint in group.active_joint_model_names]

            state = RobotState(model)
            # Joints outside this group keep their default values, so a robot whose tool hangs off
            # a torso or lift reports a pose for the default configuration of those joints.
            state.set_to_default_values()
            state.set_joint_group_positions(group_name, ordered)
            state.update()

            link_name = (
                self.get_input("link_name")
                or group.eef_name
                or group.link_model_names[-1]
            )
            transform = state.get_global_link_transform(link_name)
            position = [float(value) for value in transform[:3, 3]]

            self.log_info(
                f"{link_name} is at "
                f"x={position[0]:.4f} y={position[1]:.4f} z={position[2]:.4f} "
                f"in frame {model.model_frame}."
            )
            self.set_output(
                "tool_pose",
                {
                    "header": {"frame_id": model.model_frame},
                    "pose": {
                        "position": {
                            "x": position[0],
                            "y": position[1],
                            "z": position[2],
                        },
                        # All four components together: a partly-specified quaternion is rejected,
                        # because leaving one out silently yields a non-unit rotation.
                        "orientation": _quaternion_from(transform[:3, :3]),
                    },
                },
            )
            return NodeStatus.SUCCESS
        finally:
            for path in written:
                os.unlink(path)

    def _shut_down(self) -> None:
        """Release the ROS node and context. Safe to call more than once."""
        context = getattr(self, "_context", None)
        if context is None:
            return
        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.shutdown(context=context)
        self._context = None


def _quaternion_from(rotation) -> dict:
    """Convert a rotation matrix to the ``x, y, z, w`` quaternion a PoseStamped carries."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = [
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
            0.25 * scale,
        ]
    else:
        # Pivot on the largest diagonal entry, which keeps the divisor away from zero.
        axis = int(np.argmax([rotation[0][0], rotation[1][1], rotation[2][2]]))
        first, second = (axis + 1) % 3, (axis + 2) % 3
        scale = 2.0 * np.sqrt(
            1.0
            + rotation[axis][axis]
            - rotation[first][first]
            - rotation[second][second]
        )
        quaternion = [
            0.0,
            0.0,
            0.0,
            (rotation[second][first] - rotation[first][second]) / scale,
        ]
        quaternion[axis] = 0.25 * scale
        quaternion[first] = (rotation[first][axis] + rotation[axis][first]) / scale
        quaternion[second] = (rotation[second][axis] + rotation[axis][second]) / scale

    norm = float(np.linalg.norm(quaternion))
    return {
        "x": quaternion[0] / norm,
        "y": quaternion[1] / norm,
        "z": quaternion[2] / norm,
        "w": quaternion[3] / norm,
    }


def _write_temporary(text: str, suffix: str) -> str:
    """Write ``text`` to a temporary file and return its path."""
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=suffix, delete=False, encoding="utf-8"
    ) as handle:
        handle.write(text)
        return handle.name
