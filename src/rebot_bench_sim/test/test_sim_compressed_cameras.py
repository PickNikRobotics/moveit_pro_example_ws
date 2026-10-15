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
"""The twin's compressed camera topics decode back to the rendered image."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge


def test_compressed_image_matches_the_raw_frame_and_its_header():
    script = Path(get_package_prefix("rebot_bench_sim")) / "lib/rebot_bench_sim"
    spec = importlib.util.spec_from_file_location(
        "sim_compressed_cameras", script / "sim_compressed_cameras.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rgb = np.full((48, 64, 3), (200, 30, 10), np.uint8)
    bridge = CvBridge()
    image = bridge.cv2_to_imgmsg(rgb, "rgb8")
    image.header.frame_id = "scene_camera_color_optical_frame"
    image.header.stamp.sec = 7
    compressed = module.compress(image, bridge, 95)
    assert compressed.header == image.header
    assert compressed.format == "rgb8; jpeg compressed bgr8"
    decoded = cv2.imdecode(np.frombuffer(compressed.data, np.uint8), cv2.IMREAD_COLOR)
    assert np.abs(decoded[..., ::-1].astype(int) - rgb).max() < 8
