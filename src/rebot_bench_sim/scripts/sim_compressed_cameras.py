#!/usr/bin/env python3
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
"""Publish JPEG CompressedImage topics beside the simulated cameras' raw ones.

The real cameras publish image_raw/compressed through compressed_image_transport;
the twin's image has no compressed transport plugin, so this mirrors those topics
for the episode recorder's compressed training config.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

CAMERAS = ("/scene_camera/color/image_raw", "/wrist_mounted_camera/color/image_raw")


def compress(image: Image, bridge: CvBridge, quality: int) -> CompressedImage:
    """JPEG-encode an RGB/BGR image as compressed_image_transport does."""
    ok, data = cv2.imencode(
        ".jpg", bridge.imgmsg_to_cv2(image, "bgr8"), [cv2.IMWRITE_JPEG_QUALITY, quality]
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return CompressedImage(
        header=image.header,
        format=f"{image.encoding}; jpeg compressed bgr8",
        data=data.tobytes(),
    )


class CompressedCameras(Node):
    def __init__(self):
        super().__init__("sim_compressed_cameras")
        quality = self.declare_parameter("jpeg_quality", 95).value
        bridge = CvBridge()
        self._keep = []
        for topic in CAMERAS:
            publisher = self.create_publisher(
                CompressedImage, topic + "/compressed", qos_profile_sensor_data
            )
            self._keep.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda image, p=publisher: p.publish(
                        compress(image, bridge, quality)
                    ),
                    qos_profile_sensor_data,
                )
            )


def main():
    rclpy.init()
    rclpy.spin(CompressedCameras())


if __name__ == "__main__":
    main()
