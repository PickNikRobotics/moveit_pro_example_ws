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

"""Unit tests for the dependency-light VLA benchmark report helpers."""

import unittest

from benchmark_vla_inference import (
    health_url,
    nearest_rank_percentile,
    render_html,
    summarize,
    validate_response,
)


class TestNearestRankPercentile(unittest.TestCase):
    def test_uses_nearest_rank(self) -> None:
        self.assertEqual(nearest_rank_percentile([4.0, 1.0, 3.0, 2.0], 95), 4.0)
        self.assertEqual(nearest_rank_percentile([4.0, 1.0, 3.0, 2.0], 50), 2.0)

    def test_rejects_empty_sample(self) -> None:
        with self.assertRaisesRegex(ValueError, "empty"):
            nearest_rank_percentile([], 95)


class TestValidateResponse(unittest.TestCase):
    def test_accepts_rectangular_finite_chunk(self) -> None:
        self.assertEqual(
            validate_response({"action_chunk": [[1.0, 2.0], [3.0, 4.0]], "dt": 0.1}),
            (2, 2, 0.1),
        )

    def test_rejects_ragged_chunk(self) -> None:
        with self.assertRaisesRegex(ValueError, "rectangular"):
            validate_response({"action_chunk": [[1.0], [2.0, 3.0]], "dt": 0.1})

    def test_rejects_non_finite_action(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_response({"action_chunk": [[float("nan")]], "dt": 0.1})

    def test_rejects_non_positive_dt(self) -> None:
        with self.assertRaisesRegex(ValueError, "positive"):
            validate_response({"action_chunk": [[1.0]], "dt": 0.0})


class TestHealthUrl(unittest.TestCase):
    def test_preserves_scheme_and_authority(self) -> None:
        self.assertEqual(
            health_url("http://127.0.0.1:8973/infer"),
            "http://127.0.0.1:8973/health",
        )

    def test_rejects_an_unexpected_path(self) -> None:
        with self.assertRaisesRegex(ValueError, "exactly"):
            health_url("http://127.0.0.1:8973/other")


class TestReport(unittest.TestCase):
    def _report(self) -> dict:
        samples = [
            {
                "index": 0,
                "seconds": 1.0,
                "chunk_steps": 50,
                "action_width": 8,
                "dt": 0.1,
                "action_sha256": "a" * 64,
            },
            {
                "index": 1,
                "seconds": 1.5,
                "chunk_steps": 50,
                "action_width": 8,
                "dt": 0.1,
                "action_sha256": "b" * 64,
            },
        ]
        health = {
            "status": "ready",
            "device": "cuda",
            "accelerator": "rocm",
            "torch_version": "2.10.0+rocm7.2.2",
            "warmup": {"realtime_budget_seconds": 2.5},
        }
        return {
            "created_at": "2026-08-15T00:00:00+00:00",
            "health": health,
            "request": {
                "state_dim": 8,
                "camera_names": ["scene", "wrist"],
                "task": "stack <blue>",
            },
            "samples": samples,
            "summary": summarize(samples, health),
        }

    def test_summary_relates_median_to_realtime_budget(self) -> None:
        summary = self._report()["summary"]
        self.assertEqual(summary["median_seconds"], 1.25)
        self.assertEqual(summary["median_budget_margin_seconds"], 1.25)
        self.assertTrue(summary["median_within_realtime_budget"])
        self.assertEqual(summary["unique_action_digests"], 2)

    def test_html_is_static_and_escapes_request_text(self) -> None:
        rendered = render_html(self._report())
        self.assertIn("MoveIt Pro VLA inference benchmark", rendered)
        self.assertIn("rocm", rendered)
        self.assertIn("stack &lt;blue&gt;", rendered)
        self.assertNotIn("stack <blue>", rendered)


if __name__ == "__main__":
    unittest.main()
