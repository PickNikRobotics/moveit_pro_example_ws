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

"""Compare bracket lookahead modes on a reset factory_sim instance."""

import argparse
import json
from pathlib import Path
import re
import statistics
import time

from moveit_studio_agent_msgs.msg import Log

from moveit_studio_sdk_msgs.action import DoObjectiveSequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile


class Benchmark(Node):
    """Run bounded simulation Objectives and retain their timing messages."""

    def __init__(self) -> None:
        """Initialize the Objective client and timing subscription."""
        super().__init__("bracket_lookahead_benchmark")
        self.client = ActionClient(self, DoObjectiveSequence, "/do_objective")
        self.messages: list[dict] = []
        self.subscription = self.create_subscription(
            Log, "/failure_reason_collector", self.on_log, QoSProfile(depth=100)
        )

    def on_log(self, message: Log) -> None:
        """Retain receipt time and the Runtime's original diagnostic text."""
        self.messages.append({"wall_s": time.monotonic(), "message": message.message})

    def run(
        self,
        goal: DoObjectiveSequence.Goal,
        timeout_s: float,
        *,
        expected_cycles: int | None = None,
    ) -> dict:
        """Execute one Objective; cancel and report failure on timeout."""
        if not self.client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("The factory_sim /do_objective action is unavailable.")
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        if not sent.done() or sent.result() is None or not sent.result().accepted:
            raise RuntimeError("The factory_sim Objective goal was not accepted.")
        handle = sent.result()
        result = handle.get_result_async()
        start = time.monotonic()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout_s)
        if not result.done():
            canceled = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, canceled, timeout_sec=10.0)
            rclpy.spin_until_future_complete(self, result, timeout_sec=15.0)
            raise RuntimeError(
                f"Objective exceeded its {timeout_s:g} s timeout; cancellation requested."
            )
        response = result.result()
        duration_s = time.monotonic() - start
        diagnostics_complete = expected_cycles is None
        if (
            expected_cycles is not None
            and response.status == 4
            and response.result.error_code.val == 1
        ):
            diagnostics_complete = self.wait_for_cycle_diagnostics(expected_cycles)
        return {
            "status": response.status,
            "error_code": response.result.error_code.val,
            "error_message": response.result.error_message,
            "duration_s": duration_s,
            "cycle_diagnostics_complete": diagnostics_complete,
        }

    def wait_for_cycle_diagnostics(
        self, expected_cycles: int, timeout_s: float = 2.0
    ) -> bool:
        """Drain queued cycle labels after an action result, within a bounded wait."""
        deadline = time.monotonic() + timeout_s
        while True:
            received = sum(
                "StopwatchEnd Info: CYCLE:" in entry["message"]
                for entry in self.messages
            )
            if received >= expected_cycles:
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))


def cycle_times(messages: list[dict]) -> list[float]:
    """Read the Runtime's paired stopwatch duration and cycle label."""
    values: list[float] = []
    pending: float | None = None
    for entry in messages:
        match = re.search(
            r"StopwatchEnd Info: Time elapsed: ([0-9.eE+-]+)s", entry["message"]
        )
        if match:
            pending = float(match.group(1))
        elif "StopwatchEnd Info: CYCLE: bracket lookahead" in entry["message"]:
            if pending is None:
                raise RuntimeError(
                    "A cycle timing label has no preceding stopwatch duration."
                )
            values.append(pending)
            pending = None
        elif "StopwatchEnd Info:" in entry["message"]:
            pending = None
    return values


def timing_summary(runs: list[dict], parallel: bool) -> dict:
    """Separate cold, preparation-opportunity and terminal cycles."""
    selected = [run for run in runs if run["parallel"] == parallel]
    middle = [value for run in selected for value in run["cycle_times_s"][1:-1]]
    final = [run["cycle_times_s"][-1] for run in selected if run["cycle_times_s"]]
    first = [run["cycle_times_s"][0] for run in selected if run["cycle_times_s"]]
    summary = {
        "runs": len(selected),
        "steady_preparation_cycles": len(middle),
        "first_cycle_median_s": statistics.median(first) if first else None,
        "terminal_cycle_median_s": statistics.median(final) if final else None,
        "batch_median_s": (
            statistics.median([run["duration_s"] for run in selected])
            if selected
            else None
        ),
        "steady_preparation_median_s": statistics.median(middle) if middle else None,
        "steady_preparation_max_s": max(middle) if middle else None,
    }
    return summary


def main() -> None:
    """Reset the Simulator between identical sequential and parallel runs."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument(
        "--mode", choices=["both", "sequential", "parallel"], default="both"
    )
    args = parser.parse_args()
    if args.runs < 1 or not 1 <= args.cycles <= 3:
        parser.error(
            "runs must be positive; the factory scene contains at most three bracket picks"
        )
    rclpy.init()
    node = Benchmark()
    report: dict = {"config": "factory_sim", "runs": [], "complete": False}
    modes = [False, True] if args.mode == "both" else [args.mode == "parallel"]
    try:
        for repeat in range(args.runs):
            for parallel in modes:
                reset = node.run(
                    DoObjectiveSequence.Goal(objective_name="Reset MuJoCo Sim"), 30.0
                )
                if reset["error_code"] != 1:
                    raise RuntimeError(
                        f'Simulation reset failed: {reset["error_message"]}'
                    )
                node.messages.clear()
                xml = (
                    '<root BTCPP_format="4" main_tree_to_execute="Bracket Benchmark">'
                    '<BehaviorTree ID="Bracket Benchmark">'
                    '<SubTree ID="Pick and Place Brackets with Lookahead" '
                    f'parallel="{str(parallel).lower()}" '
                    f'num_cycles="{args.cycles}" fallback_count="{{fallback_count}}" />'
                    "</BehaviorTree></root>"
                )
                result = node.run(
                    DoObjectiveSequence.Goal(objective_xml_string=xml),
                    360.0,
                    expected_cycles=args.cycles,
                )
                record = {
                    "repeat": repeat + 1,
                    "parallel": parallel,
                    **result,
                    "messages": list(node.messages),
                }
                report["runs"].append(record)
                args.output.write_text(json.dumps(report, indent=2) + "\n")
                print(
                    json.dumps(
                        {
                            key: value
                            for key, value in record.items()
                            if key != "messages"
                        }
                    ),
                    flush=True,
                )
                if result["error_code"] != 1 or result["status"] != 4:
                    raise RuntimeError(
                        f'Bracket cycle failed: {result["error_message"]}'
                    )
                record["cycle_times_s"] = cycle_times(node.messages)
                record["fallback_count"] = sum(
                    "HANDOFF: bracket pick requires planning from the current state."
                    in entry["message"]
                    for entry in node.messages
                )
                record["accepted_candidates"] = sum(
                    "HANDOFF: bracket pick candidate accepted." in entry["message"]
                    for entry in node.messages
                )
                record["setup_cleanup_transport_s"] = result["duration_s"] - sum(
                    record["cycle_times_s"]
                )
                if len(record["cycle_times_s"]) != args.cycles:
                    raise RuntimeError(
                        "Cycle timing count differs from completed bracket count."
                    )
        report["complete"] = True
        report["summary"] = {}
        for parallel in modes:
            report["summary"]["parallel" if parallel else "sequential"] = (
                timing_summary(report["runs"], parallel)
            )
    finally:
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
