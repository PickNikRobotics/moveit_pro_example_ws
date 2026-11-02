# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The arm backend defaults to mock and wires the real RobStride driver correctly."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

# Motor id -> (actuator model, kp, kd). RS06 drives the three base joints and
# RS00 the wrist and gripper; the model selects the motor's CAN quantization
# ranges, so a wrong one rescales every position, velocity and effort silently.
# Arm kp/kd start from Seeed's values; jaw damping is increased for fast opening.
MOTORS = {
    "joint1": ("1", "RS06", "50.0", "3.0"),
    "joint2": ("2", "RS06", "150.0", "10.0"),
    "joint3": ("3", "RS06", "150.0", "10.0"),
    "joint4": ("4", "RS00", "50.0", "5.0"),
    "joint5": ("5", "RS00", "50.0", "4.0"),
    "joint6": ("6", "RS00", "50.0", "4.0"),
    "joint7": ("7", "RS00", "12.0", "0.5"),
}


def expand(**mappings):
    path = (
        Path(get_package_share_directory("rebot_bench_base_config"))
        / "description/rebot.urdf.xacro"
    )
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


def params(element):
    return {p.get("name"): p.text for p in element.findall("param")}


def test_default_and_explicit_mock_have_only_generic_system():
    for mappings in ({}, {"hardware_interface": "mock"}):
        model = expand(**mappings)
        assert [p.text for p in model.findall("ros2_control/hardware/plugin")] == [
            "mock_components/GenericSystem"
        ]
        assert len(model.findall("ros2_control/joint")) == 7
        # Mock carries no motor hardware: no CAN parameters, no gpio blocks.
        assert model.findall("ros2_control/gpio") == []
        assert params(model.find("ros2_control/hardware")) == {
            "calculate_dynamics": "true"
        }


def test_unsupported_selector_fails_expansion():
    # A silently plugin-less ros2_control block would load and then do nothing.
    with pytest.raises(xacro.XacroException):
        expand(hardware_interface="bogus")


def test_real_selects_the_robstride_driver_with_the_no_brakes_settings():
    hardware = expand(hardware_interface="real").find("ros2_control/hardware")
    assert [p.text for p in hardware.findall("plugin")] == [
        "robstride_hardware_interface/RobstrideHardware"
    ]
    assert params(hardware) == {
        "master_id": "253",
        "number_of_joints": "7",
        # The arm has no brakes, so a clean stop must hold the pose. The hold is
        # only real while the motors' own CAN_TIMEOUT watchdog is off, because
        # deactivation closes the bus: the two settings are one decision.
        "hold_torque_on_deactivate": "true",
        "motor_can_timeout_ms": "0",
        # Losing one joint leaves the rest holding rather than dropping the arm.
        "freeze_on_joint_loss": "true",
        "torque_enable": "true",
        "thermal_cutoff_c": "100.0",
        "thermal_clear_c": "75.0",
    }


def test_real_declares_one_motor_per_joint_matched_by_id():
    control = expand(hardware_interface="real").find("ros2_control")
    joints = control.findall("joint")
    gpios = control.findall("gpio")
    assert len(joints) == 7
    assert len(gpios) == 7

    for joint in joints:
        motor_id, model, kp, kd = MOTORS[joint.get("name")]
        assert params(joint) == {"id": motor_id}
        # Both modes retain position-only commands by default. Arm effort
        # feedforward is a separate, explicit opt-in.
        assert [c.get("name") for c in joint.findall("command_interface")] == [
            "position"
        ]
        # The driver exports only what is declared, and
        # joint_state_broadcaster asks for position and velocity.
        assert [s.get("name") for s in joint.findall("state_interface")] == [
            "position",
            "velocity",
        ]
        # on_init matches a joint to its gpio by id and refuses to start
        # without one.
        gpio = next(g for g in gpios if params(g)["ID"] == motor_id)
        assert params(gpio) == {
            "type": "robstride",
            "ID": motor_id,
            "actuator_type": model,
            "can_interface": "can0",
            # Impedance (motion) by default on every joint.
            "control_mode": "motion",
            "kp": kp,
            "kd": kd,
        }


def test_arm_control_mode_switches_joint1_to_6_and_never_the_gripper():
    control = expand(hardware_interface="real", arm_control_mode="position_csp")
    modes = {
        params(g)["ID"]: params(g)["control_mode"]
        for g in control.find("ros2_control").findall("gpio")
    }
    assert modes == {**{str(i): "position_csp" for i in range(1, 7)}, "7": "motion"}
    with pytest.raises(xacro.XacroException):
        expand(hardware_interface="real", arm_control_mode="torque")


@pytest.mark.parametrize("hardware", ["mock", "real", "mujoco"])
@pytest.mark.parametrize("mode", ["motion", "position_csp"])
@pytest.mark.parametrize("effort", ["false", "true"])
def test_effort_command_is_opt_in_real_motion_arm_only(hardware, mode, effort):
    model = expand(
        hardware_interface=hardware,
        arm_control_mode=mode,
        arm_effort_command=effort,
    )
    efforts = {
        joint.get("name")
        for joint in model.findall("ros2_control/joint")
        if joint.find("command_interface[@name='effort']") is not None
    }
    expected = (
        {f"joint{i}" for i in range(1, 7)}
        if (hardware, mode, effort) == ("real", "motion", "true")
        else set()
    )
    assert efforts == expected
    # Opting in must not alter any motor gains, modes or hardware parameters.
    default = expand(hardware_interface=hardware, arm_control_mode=mode)
    assert [params(g) for g in model.findall("ros2_control/gpio")] == [
        params(g) for g in default.findall("ros2_control/gpio")
    ]
    for added in model.findall("ros2_control/joint"):
        for interface in added.findall("command_interface[@name='effort']"):
            added.remove(interface)

    def normalized(element):
        return (
            element.tag,
            element.attrib,
            (element.text or "").strip(),
            [normalized(child) for child in element],
        )

    assert normalized(model) == normalized(default)


def test_gravity_controller_is_inactive_with_zero_output_defaults(tmp_path):
    import yaml
    from moveit_studio_utils_py.system_config import load_system_config

    config = load_system_config("rebot_bench_base_config", tmp_path)[0]
    assert (
        "gravity_compensation_controller"
        in config.ros2_control["controllers_inactive_at_startup"]
    )
    assert (
        "gravity_compensation_controller"
        not in config.ros2_control["controllers_active_at_startup"]
    )
    urdf_params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    assert urdf_params["hardware_interface"] == "mock"
    assert urdf_params["arm_effort_command"] == "false"
    share = Path(get_package_share_directory("rebot_bench_base_config"))
    control = yaml.safe_load(
        (share / "config/control/rebot.ros2_control.yaml").read_text()
    )
    gravity = control["gravity_compensation_controller"]["ros__parameters"]
    assert control["controller_manager"]["ros__parameters"][
        "gravity_compensation_controller"
    ]["type"] == ("kdl_gravity_compensation_controller/GravityCompensationController")
    assert gravity["joints"] == [f"joint{i}" for i in range(1, 7)]
    assert gravity["gains"] == [0.0] * 6
    assert gravity["offsets"] == [0.0] * 6
    assert gravity["payload_mass"] == 0.0
    model = expand()
    assert model.find(f"link[@name='{gravity['root_link']}']") is not None
    assert model.find(f"link[@name='{gravity['tip_link']}']") is not None


def test_shipped_config_runs_joint1_to_6_in_position_csp(tmp_path):
    from moveit_studio_utils_py.system_config import load_system_config

    config = load_system_config("rebot_bench_base_config", tmp_path)[0]
    urdf_params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    control = expand(**{**urdf_params, "hardware_interface": "real"})
    modes = {
        params(g)["ID"]: params(g)["control_mode"]
        for g in control.find("ros2_control").findall("gpio")
    }
    assert modes == {**{str(i): "position_csp" for i in range(1, 7)}, "7": "motion"}


def test_real_branch_follows_the_can_interface_and_torque_arguments():
    control = expand(
        hardware_interface="real", can_interface="can1", torque_enable="false"
    ).find("ros2_control")
    # torque_enable=false is the first-session dry run: readable but limp.
    assert params(control.find("hardware"))["torque_enable"] == "false"
    assert {params(g)["can_interface"] for g in control.findall("gpio")} == {"can1"}


@pytest.mark.parametrize("hardware_interface", ["mock", "real", "mujoco"])
def test_commanded_joints_keep_the_measured_motor_frame_limits(hardware_interface):
    # These are approved motor-frame bounds, including conservative J2/J3
    # home margins. They are not a measurement of negative hard stops or a
    # guarantee of collision-free motion; command enforcement is driver-specific.
    expected = {
        "joint1": (-2.530727415392, 2.530727415392),
        "joint2": (-0.02, 3.543018381548),
        "joint3": (-0.01, 4.171336912266),
        "joint4": (-1.850049007114, 1.500983156715),
        "joint5": (-1.762782544514, 1.884955592154),
        "joint6": (-3.054326190990, 3.054326190990),
        "joint7": (0.0, 5.410520681182422),
    }
    model = expand(hardware_interface=hardware_interface)
    commanded = {j.get("name") for j in model.findall("ros2_control/joint")}
    assert commanded == (set() if hardware_interface == "mujoco" else set(expected))

    limits = {
        joint.get("name"): joint.find("limit")
        for joint in model.findall("joint")
        if joint.get("name") in expected
    }
    for name, (lower, upper) in expected.items():
        assert float(limits[name].get("lower")) == pytest.approx(lower)
        assert float(limits[name].get("upper")) == pytest.approx(upper)
        # Every joint spans less than a full turn, which is what makes the
        # driver's power-cycle unwrapping read the same angle after a reboot.
        assert upper - lower < 2 * 3.141592653589793


def test_mujoco_leaves_hardware_and_optical_frames_to_the_simulation():
    # rebot_bench_sim supplies the MuJoCo ros2_control block, and the
    # simulator publishes the optical frames from its camera sites; a URDF
    # copy of either would give the joints two hardware owners or the frames
    # two TF parents.
    model = expand(hardware_interface="mujoco")
    assert model.findall("ros2_control") == []
    links = {link.get("name") for link in model.findall("link")}
    assert "d435_color_optical_frame" not in links
    assert "scene_camera_color_optical_frame" not in links
    assert {"d435_link", "scene_camera_link"} <= links
    for mappings in ({}, {"hardware_interface": "real"}):
        links = {link.get("name") for link in expand(**mappings).findall("link")}
        assert {"d435_color_optical_frame", "scene_camera_color_optical_frame"} <= links


def test_base_and_scene_camera_placements_default_to_world_and_follow_arguments():
    def origin(model, joint):
        element = model.find(f"joint[@name='{joint}']/origin")
        return element.get("xyz"), element.get("rpy")

    model = expand()
    assert origin(model, "world_to_base") == ("0 0 0", "0 0 0")
    assert origin(model, "scene_camera_mount_joint") == ("0 0 0", "0 0 0")
    model = expand(
        world_to_base_xyz="0.1 -0.6 0",
        world_to_base_rpy="0 0 1.57",
        scene_camera_xyz="0.6 -0.25 0.7",
        scene_camera_rpy="0 1.2 3.1",
    )
    assert origin(model, "world_to_base") == ("0.1 -0.6 0", "0 0 1.57")
    assert origin(model, "scene_camera_mount_joint") == ("0.6 -0.25 0.7", "0 1.2 3.1")


def test_jaw_gains_can_be_tuned_without_changing_arm_mode():
    root = expand(
        hardware_interface="real",
        arm_control_mode="position_csp",
        jaw_kp="10",
        jaw_kd="0.7",
    )
    motors = {g.get("name"): params(g) for g in root.findall("ros2_control/gpio")}
    assert motors["joint7_motor"]["kp"] == "10"
    assert motors["joint7_motor"]["kd"] == "0.7"
    assert motors["joint7_motor"]["control_mode"] == "motion"
    assert motors["joint4_motor"]["control_mode"] == "position_csp"
