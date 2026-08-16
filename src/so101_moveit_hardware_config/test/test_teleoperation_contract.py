"""Tests for the SO-101 teleoperation controller contract."""

from pathlib import Path
import xml.etree.ElementTree as ET


OBJECTIVE_PATH = Path(__file__).parents[1] / "objectives" / "request_teleoperation.xml"
JTC_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
JTC_NAME = "joint_trajectory_controller"
JTC_PIPELINE = "jtc"


def _objective_root() -> ET.Element:
    return ET.parse(OBJECTIVE_PATH).getroot()


def _port_defaults(root: ET.Element) -> dict[str, str]:
    model = root.find("./TreeNodesModel/SubTree[@ID='Request Teleoperation']")
    assert model is not None
    return {
        port.attrib["name"]: port.attrib["default"]
        for port in model
        if port.tag.endswith("_port") and "default" in port.attrib
    }


def test_velocity_scale_is_seeded_before_parallel_teleoperation() -> None:
    root = _objective_root()
    sequence = root.find(
        "./BehaviorTree[@ID='Request Teleoperation']/Control[@ID='Sequence']"
    )
    assert sequence is not None

    children = list(sequence)
    assert children[0].attrib == {"ID": "Script", "code": "teleop_mode := 0"}
    assert children[1].attrib == {
        "ID": "Script",
        "code": "velocity_scale_factor := 0.25",
    }
    assert children[2].attrib["ID"] == "Parallel"

    teleoperate_action = sequence.find(".//Action[@ID='DoTeleoperateAction']")
    assert teleoperate_action is not None
    assert teleoperate_action.attrib["velocity_scale_factor"] == (
        "{velocity_scale_factor}"
    )


def test_all_planned_teleoperation_motion_uses_seeded_scale_and_jtc_ports() -> None:
    root = _objective_root()
    expected_controller_attributes = {
        "Interpolate to Joint State": "controller_action_server",
        "Move to Pose": "controller_action_name",
        "Move to Joint State": "controller_action_server",
    }

    for subtree_id, action_attribute in expected_controller_attributes.items():
        subtree = root.find(f".//SubTree[@ID='{subtree_id}']")
        assert subtree is not None
        assert subtree.attrib["velocity_scale_factor"] == "{velocity_scale_factor}"
        assert subtree.attrib["controller_names"] == (
            "{joint_trajectory_controller_name}"
        )
        assert subtree.attrib[action_attribute] == "{controller_action_server}"
        assert subtree.attrib["execution_pipeline"] == "{execution_pipeline}"


def test_so101_defaults_planned_teleoperation_to_jtc() -> None:
    root = _objective_root()
    defaults = _port_defaults(root)

    assert defaults["controller_action_server"] == JTC_ACTION
    assert defaults["joint_trajectory_controller_name"] == JTC_NAME
    assert defaults["execution_pipeline"] == JTC_PIPELINE

    objective_text = OBJECTIVE_PATH.read_text()
    assert "joint_trajectory_admittance_controller" not in objective_text
    assert "RetrieveJointStateParameter" not in objective_text
    assert len(root.findall(".//Action[@ID='RetrieveRobotStateParameter']")) == 2
