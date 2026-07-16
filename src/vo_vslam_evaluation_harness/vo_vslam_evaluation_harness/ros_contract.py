# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""ROS-independent serialization of diagnostic confidence v0."""

from __future__ import annotations

import math

from .core import TrackingResult


def diagnostic_values(
    result: TrackingResult, unsynchronized_messages: int, adapter_name: str
) -> dict[str, str]:
    """Convert measured tracking health to stable diagnostic key/value strings."""
    values = {
        "adapter": adapter_name,
        "tracking_state": result.state.value,
        "failure_reason": result.failure_reason,
        "covariance_status": "unknown",
        "unsynchronized_messages": str(unsynchronized_messages),
        "keypoints_left": str(result.keypoints_left),
        "keypoints_right": str(result.keypoints_right),
        "stereo_matches": str(result.stereo_matches),
        "temporal_matches": str(result.temporal_matches),
        "pnp_points": str(result.pnp_points),
        "pnp_solved": str(result.pnp_inliers > 0).lower(),
        "pnp_inliers": str(result.pnp_inliers),
        "pnp_inlier_ratio": f"{result.pnp_inlier_ratio:.6f}",
    }
    if math.isfinite(result.residual_mean_px):
        values["reprojection_residual_mean_px"] = f"{result.residual_mean_px:.6f}"
    if math.isfinite(result.residual_p95_px):
        values["reprojection_residual_p95_px"] = f"{result.residual_p95_px:.6f}"
    for stage, latency_ms in result.stage_latency_ms.items():
        values[f"latency_{stage}_ms"] = f"{latency_ms:.3f}"
    return values
