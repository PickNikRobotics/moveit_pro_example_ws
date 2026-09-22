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

"""Feeds both filtered lidar scans to AMCL on one topic, one scan per message.

AMCL takes a single scan topic, and the two lidars each cover 270 deg, so the
localizer needs both of them to see all the way around the base. Relaying them
onto /scan_localization gives it both without ever putting two scans -- and so
two instants -- inside one message.

/scan_front_filtered and /scan_rear_filtered are republished untouched, so the
costmaps and slam_toolbox that read them directly are unaffected.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


SOURCE_TOPICS = ("/scan_front_filtered", "/scan_rear_filtered")
LOCALIZATION_TOPIC = "/scan_localization"


class ScanLocalizationRelay(Node):
    """Republishes every filtered scan, from either lidar, onto one topic."""

    def __init__(self):
        super().__init__("scan_localization_relay")

        self.publisher = self.create_publisher(
            LaserScan, LOCALIZATION_TOPIC, qos_profile_sensor_data
        )
        self.subscriptions_by_topic = {
            topic: self.create_subscription(
                LaserScan, topic, self._relay, qos_profile_sensor_data
            )
            for topic in SOURCE_TOPICS
        }

    def _relay(self, scan):
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanLocalizationRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
