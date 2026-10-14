"""The release rests the grasped cube on red without a drop."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

OBJECTIVES = Path(__file__).resolve().parents[1] / "objectives"


def test_release_pose_places_the_grasped_cube_on_red_without_a_drop():
    root = ET.parse(OBJECTIVES / "rebot_check_stack_reach.xml")
    release = root.find('.//Action[@ID="TransformPose"][@output_pose="{place}"]')
    x, y, z = map(float, release.get("translation_xyz").split(";"))
    tilt = math.radians(5)
    # A 50 mm cube centred at 25 mm is grasped with the tip at 35 mm.
    grasp_offset = 0.025 - 0.035
    assert abs(x - grasp_offset * math.sin(tilt)) < 1e-8
    assert y == 0
    cube_half_height = 0.025 * (math.cos(tilt) + math.sin(tilt))
    assert abs(z + grasp_offset * math.cos(tilt) - cube_half_height) < 1e-8
