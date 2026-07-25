#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Helper module for the example Behaviors.

Files whose name starts with ``_`` are not scanned for Behaviors, so a directory can hold
shared code that the Behaviors next to it import normally. Give these a distinctive name:
Python resolves its own built-in modules first, so a helper called ``_statistics`` or
``queue`` would be shadowed by the standard library.
"""

from typing import Dict, List, Sequence

import numpy as np


def largest_absolute_value(values: Sequence[float]) -> float:
    """Return the largest absolute value in ``values``, or 0.0 if it is empty."""
    if len(values) == 0:
        return 0.0
    return float(np.max(np.abs(np.asarray(values, dtype=float))))


def joint_positions(message: Dict) -> Dict[str, float]:
    """Extract ``{joint name: position}`` from a JointState or RobotJointState dict.

    An ``any`` port hands over whatever the blackboard holds, and the two message types that
    carry joint positions in MoveIt Pro differ by one level of nesting: ``RetrieveWaypoint``
    outputs a ``RobotJointState``, which wraps a ``sensor_msgs/JointState`` under ``joint_state``.
    """
    joint_state = message.get("joint_state", message) or {}
    names: List[str] = joint_state.get("name") or []
    positions: List[float] = joint_state.get("position") or []
    return {name: float(position) for name, position in zip(names, positions)}
