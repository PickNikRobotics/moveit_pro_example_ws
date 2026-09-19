#!/usr/bin/env python3
"""Seed covariances shared between the A/B driver and the analysis tools.

`navloop_ab.py` (which needs rclpy and runs inside the runtime container) writes the rescue
covariance; `armsumm.py` (which must stay importable with nothing but the standard library)
reads it back to tell a rescue apart from an Objective's own re-seed. Both import it from here
so widening the rescue seed cannot silently reclassify rescue events as an experiment arm.
"""

# Deliberately a covariance no Objective arm uses, so the two populations stay distinguishable
# in a recording. If you change it, both sides follow automatically.
RESCUE_XY, RESCUE_YAW = 0.0100, 0.0009

# Float equality on values that survive a round trip through a ROS message and JSON.
COV_TOL = 1e-6
