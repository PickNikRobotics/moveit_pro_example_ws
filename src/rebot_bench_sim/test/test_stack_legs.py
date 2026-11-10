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
"""The release rests the grasped cube on red without a drop."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

OBJECTIVES = Path(__file__).resolve().parents[1] / "objectives"


def test_release_pose_places_the_grasped_cube_on_red_without_a_drop():
    root = ET.parse(OBJECTIVES / "rebot_plan_place.xml")
    release = root.find('.//Action[@ID="TransformPose"][@output_pose="{place}"]')
    x, y, z = map(float, release.get("translation_xyz").split(";"))
    tilt = math.radians(5)
    # A 50 mm cube centred at 25 mm is grasped with the tip at 35 mm.
    grasp_offset = 0.025 - 0.035
    assert abs(x - grasp_offset * math.sin(tilt)) < 1e-8
    assert y == 0
    cube_half_height = 0.025 * (math.cos(tilt) + math.sin(tilt))
    assert abs(z + grasp_offset * math.cos(tilt) - cube_half_height) < 1e-8
