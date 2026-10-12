# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Compare the original factory application and isolated optimization profiles."""

import argparse
import json
from pathlib import Path
import re
import statistics

from benchmark_parallel_pick import Benchmark
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
import rclpy


def profile_summary(runs: list[dict], profile: str) -> dict:
    """Summarize complete batches and comparable middle cycles for one profile."""
    selected = [run for run in runs if run["profile"] == profile]
    if not selected:
        raise ValueError("No completed runs for profile: " + profile)
    return {
        "runs": len(selected),
        "batch_median_s": statistics.median([run["duration_s"] for run in selected]),
        "middle_cycle_median_s": statistics.median(
            [run["cycle_times_s"][1] for run in selected]
        ),
        "cycle_sum_median_s": statistics.median(
            [sum(run["cycle_times_s"]) for run in selected]
        ),
    }


def main() -> None:
    """Run reset three-bracket batches through each selected scheduling profile."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--profiles",
        nargs="+",
        choices=["original", "lookahead", "near_jig", "cached"],
        default=["original", "near_jig", "cached"],
    )
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("runs must be positive")
    report = {"runs": [], "complete": False}
    rclpy.init()
    node = Benchmark()
    try:
        for repeat in range(args.runs):
            for profile in args.profiles:
                reset = node.run(
                    DoObjectiveSequence.Goal(objective_name="Reset MuJoCo Sim"), 30
                )
                if reset["error_code"] != 1:
                    raise RuntimeError("Simulator reset failed.")
                node.messages.clear()
                if profile == "original":
                    goal = DoObjectiveSequence.Goal(
                        objective_name="Pick and Place Brackets from Left Bin"
                    )
                else:
                    near = "true" if profile in ["near_jig", "cached"] else "false"
                    cached = "true" if profile == "cached" else "false"
                    flags = f' near_jig_wait="{near}" cache_transfers="{cached}"'
                    xml = (
                        '<root BTCPP_format="4" main_tree_to_execute="Timing comparison">'
                        '<BehaviorTree ID="Timing comparison">'
                        '<SubTree ID="Pick and Place Brackets with Lookahead" '
                        f'parallel="true" num_cycles="3"{flags} />'
                        "</BehaviorTree></root>"
                    )
                    goal = DoObjectiveSequence.Goal(objective_xml_string=xml)
                result = node.run(goal, 360)
                times = []
                pending = None
                for msg in node.messages:
                    match = re.search(
                        r"StopwatchEnd Info: Time elapsed: ([0-9.eE+-]+)s",
                        msg["message"],
                    )
                    if match:
                        pending = float(match.group(1))
                    elif "StopwatchEnd Info:" in msg["message"]:
                        if "CYCLE:" in msg["message"] and pending is not None:
                            times.append(pending)
                        pending = None
                record = {
                    "repeat": repeat + 1,
                    "profile": profile,
                    **result,
                    "cycle_times_s": times,
                    "messages": list(node.messages),
                }
                report["runs"].append(record)
                args.output.write_text(json.dumps(report, indent=2) + "\n")
                print(
                    json.dumps({k: v for k, v in record.items() if k != "messages"}),
                    flush=True,
                )
                if (
                    result["status"] != 4
                    or result["error_code"] != 1
                    or len(times) != 3
                ):
                    raise RuntimeError(
                        "A three-bracket batch did not complete all cycles."
                    )
        report["complete"] = True
        report["summary"] = {
            profile: profile_summary(report["runs"], profile)
            for profile in args.profiles
        }
    finally:
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
