"""Render a pytest JUnit xunit.xml to a nicer HTML report.

Tests are grouped by their parent directory. Per-test ROS log slices come
from `<xunit.parent>/ros_logs/rosout.ndjson` (one JSON object per line,
captured from the /rosout topic by lab_sim/test/conftest.py).
"""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import html
import json
import os.path
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


# ---------- formatting helpers ----------


def format_duration(seconds: float) -> str:
    if seconds < 60:
        return f"{seconds:.1f}s"
    minutes, secs = divmod(seconds, 60)
    if minutes < 60:
        return f"{int(minutes)}m {secs:.0f}s"
    hours, minutes = divmod(int(minutes), 60)
    return f"{hours}h {minutes}m {secs:.0f}s"


def format_timestamp(iso: str) -> str:
    if not iso:
        return ""
    try:
        t = dt.datetime.fromisoformat(iso)
        return t.strftime("%b %d, %Y · %H:%M:%S")
    except ValueError:
        return iso


def parse_iso_to_unix(iso: str) -> float | None:
    if not iso:
        return None
    try:
        t = dt.datetime.fromisoformat(iso)
        if t.tzinfo is None:
            t = t.replace(tzinfo=dt.timezone.utc)
        return t.timestamp()
    except ValueError:
        return None


# ---------- test name + objective resolution ----------


def split_path(test_name: str) -> tuple[str, str, str]:
    """Return (parent_dir, filename, full). Handles non-path test names too."""
    m = re.search(r"\[([^\]]+)\]", test_name)
    full = m.group(1) if m else test_name
    if "/" not in full:
        return "", full, full
    parent, fname = os.path.split(full)
    return parent, fname, full


_RE_EXAMPLE_WS = re.compile(
    r"^/__w/([^/]+)/\1/install/([^/]+)/share/\2/objectives/(.+)$"
)
_RE_OVERLAY = re.compile(r"^/opt/overlay_ws/install/([^/]+)/share/\1/objectives/(.+)$")

_EXAMPLE_WS_ROOTS = [
    Path.home() / "code" / "moveit_pro_example_ws",
]
# Also resolve via the renderer's CWD so the GitHub Actions render-report job
# (which checks out the repo at $GITHUB_WORKSPACE and runs the renderer from
# there) can find objective XMLs without needing the hardcoded local path above.
_EXAMPLE_WS_ROOTS.append(Path.cwd())
_MOVEIT_PRO_INSTALL_ROOT = Path.home() / "code" / "moveit_pro" / ".colcon" / "install"
_MOVEIT_PRO_LOCAL = Path.home() / "code" / "moveit_pro"
_CONTAINER_PREFIX = "/opt/overlay_ws"


def find_local_xml(ci_path: str) -> Path | None:
    m = _RE_EXAMPLE_WS.match(ci_path)
    if m:
        pkg, rel = m.group(2), m.group(3)
        for root in _EXAMPLE_WS_ROOTS:
            candidate = root / "src" / pkg / "objectives" / rel
            if candidate.exists():
                return candidate
        return None
    m = _RE_OVERLAY.match(ci_path)
    if m:
        pkg, rel = m.group(1), m.group(2)
        install_link = (
            _MOVEIT_PRO_INSTALL_ROOT / pkg / "share" / pkg / "objectives" / rel
        )
        if install_link.is_symlink():
            target = os.readlink(install_link)
            if target.startswith(_CONTAINER_PREFIX + "/"):
                candidate = _MOVEIT_PRO_LOCAL / target[len(_CONTAINER_PREFIX) + 1 :]
                if candidate.exists():
                    return candidate
        elif install_link.exists():
            return install_link
    return None


_OBJECTIVE_NAME_CACHE: dict[str, str | None] = {}


def extract_objective_name(ci_path: str) -> str | None:
    if ci_path in _OBJECTIVE_NAME_CACHE:
        return _OBJECTIVE_NAME_CACHE[ci_path]
    local = find_local_xml(ci_path)
    name = None
    if local is not None:
        try:
            tree = ET.parse(local)
            root = tree.getroot()
            if "main_tree_to_execute" in root.attrib:
                name = root.attrib["main_tree_to_execute"]
            if name is None:
                for tnm in root.iter("TreeNodesModel"):
                    for sub in tnm.findall("SubTree"):
                        if "ID" in sub.attrib:
                            name = sub.attrib["ID"]
                            break
                    if name:
                        break
            if name is None:
                for bt in root.iter("BehaviorTree"):
                    if "ID" in bt.attrib:
                        name = bt.attrib["ID"]
                        break
        except (ET.ParseError, OSError):
            pass
    _OBJECTIVE_NAME_CACHE[ci_path] = name
    return name


# ---------- ROS log loading + slicing ----------

# launch.log line formats — two patterns. Order matters: the leveled form is
# more specific (requires `]:` after the second bracket) and must be tried
# first, otherwise the passthrough regex would mis-capture `[LEVEL]` as the
# process name.
#
# Leveled (launch's own events):  `<ts> [LEVEL] [name]: msg`
_RE_LAUNCH_LEVELED = re.compile(
    r"^(?P<ts>\d+(?:\.\d+)?)\s+\[(?P<level>[A-Z]+)\]\s+\[(?P<name>[^\]]+)\]:\s+(?P<msg>.*)$"
)
# Passthrough (stdout/stderr from launched processes):  `<ts> [name-N] msg`
_RE_LAUNCH_PASSTHROUGH = re.compile(
    r"^(?P<ts>\d+(?:\.\d+)?)\s+\[(?P<name>[^\]]+)\]\s+(?P<msg>.*)$"
)
# Strip the launch sequence suffix (`-14` → ``) so passthrough lines align
# with /rosout entries from the same node.
_RE_LAUNCH_SEQ_SUFFIX = re.compile(r"-\d+$")

# Many ROS executables follow the convention `<name>_node_main` (the binary
# in `lib/<pkg>/`), while the rcl logger publishes /rosout under the bare
# `<name>_node`. Strip the `_main` suffix so launch.log passthrough entries
# align with /rosout entries from the same node. Anchor to `_node_main$` so
# we don't accidentally strip `_main` from binaries that legitimately end
# in `_main` without `_node` (uncommon, but possible).
_RE_EXEC_MAIN_SUFFIX = re.compile(r"(_node)_main$")

# rcutils' default stream formatter prepends `[LEVEL] [timestamp] ` to every
# message written to stdout — what ros2 launch then aggregates into
# launch.log. Bare /rosout messages have no such prefix, so without
# stripping it the same logical line appears twice in the report (rosout +
# launch.log) and the (level, node, msg) dedup misses both copies. The
# timestamp bracket varies (unix float, ISO wall time, with or without
# milliseconds) — match `[<anything that isn't ]>]`. Optional ANSI color
# codes wrap each bracket when rcutils' tty colorization is on.
_RE_RCUTILS_PREFIX = re.compile(
    r"^\s*"
    r"(?:\x1b\[\d+m)?\[(?:DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\](?:\x1b\[\d+m)?"
    r"\s*"
    r"(?:\x1b\[\d+m)?\[[^\]]+\](?:\x1b\[\d+m)?"
    r"\s*:?\s*"
)

# A passthrough line is the raw stdout/stderr of a launched process. ROS code
# often prints its own `[LEVEL]` bracket inside that text (sometimes with ANSI
# color codes around it). Extract that level when present so the renderer can
# style the entry correctly instead of dumping it as INFO.
_RE_PASSTHROUGH_EMBED_LEVEL = re.compile(
    r"^\s*(?:\x1b\[\d+m)?\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]"
)

# When a passthrough line has no embedded level (stderr from a dying process,
# uncaught exception text, etc.) but its content matches one of these markers,
# treat it as ERROR. Without this, boot-crash diagnostics like
# `Aborted (Signal sent by tkill())` or `what(): Failed to load RobotModel`
# get dumped at INFO severity and disappear when the user filters to errors.
# Markers split into two groups:
#   - LINE_START: anchored to the start of the message text. Avoids false
#     positives like "Operation Aborted by user" or a docstring mention of
#     "Traceback" being elevated to ERROR.
#   - SUBSTRING: stable enough as substrings (highly specific, unlikely to
#     appear in benign log content).
_PASSTHROUGH_ERROR_LINE_START = (
    "Aborted",
    "Segmentation fault",
    "Stack trace",
    "Traceback",
    "terminate called",
)
_PASSTHROUGH_ERROR_SUBSTRING = (
    "SIGSEGV",
    "SIGABRT",
    "what():",
)


def _classify_passthrough(msg: str) -> str:
    m = _RE_PASSTHROUGH_EMBED_LEVEL.match(msg)
    if m:
        level = m.group(1).upper()
        return "WARN" if level == "WARNING" else level
    stripped = msg.lstrip()
    for marker in _PASSTHROUGH_ERROR_LINE_START:
        if stripped.startswith(marker):
            return "ERROR"
    for marker in _PASSTHROUGH_ERROR_SUBSTRING:
        if marker in msg:
            return "ERROR"
    return "INFO"


def load_rosout_ndjson(ndjson_path: Path) -> list[tuple[float, str, str, str]]:
    """Read structured /rosout log entries from rosout.ndjson.

    The file is produced by lab_sim/test/conftest.py via a session-scoped
    subscriber to /rosout. Each line is one JSON object: {ts, level, node, msg}.
    Going through /rosout (a published rcl_interfaces/msg/Log topic with a
    stable schema) instead of regex-parsing rcutils file-logger output
    insulates the renderer from rcutils' textual format, which has no
    stability contract.
    """
    if not ndjson_path.is_file():
        return []
    out: list[tuple[float, str, str, str]] = []
    try:
        with open(ndjson_path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                    out.append(
                        (
                            float(entry["ts"]),
                            str(entry.get("level", "")).upper(),
                            str(entry.get("node", "")),
                            str(entry.get("msg", "")),
                        )
                    )
                except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                    continue
    except OSError:
        return []
    out.sort(key=lambda x: x[0])
    return out


def load_launch_logs(logs_dir: Path) -> list[tuple[float, str, str, str]]:
    """Read every <logs_dir>/<dated>/launch.log file produced by ros2 launch.

    Secondary source alongside /rosout. Critical for diagnosing boot
    failures where a node crashes before publishing anything to /rosout —
    the stack trace and SIGSEGV/SIGABRT signal lines live in launch.log's
    stdout/stderr passthrough only. Format is best-effort; lines that
    don't match either regex (e.g. raw stderr without a timestamp prefix
    from a child process) are dropped.
    """
    if not logs_dir.is_dir():
        return []
    out: list[tuple[float, str, str, str]] = []
    for launch_log in sorted(logs_dir.glob("*/launch.log")):
        try:
            with open(launch_log, encoding="utf-8", errors="replace") as f:
                for line in f:
                    m = _RE_LAUNCH_LEVELED.match(line)
                    if m:
                        try:
                            out.append(
                                (
                                    float(m["ts"]),
                                    m["level"].upper(),
                                    m["name"],
                                    m["msg"].rstrip(),
                                )
                            )
                        except ValueError:
                            pass
                        continue
                    m2 = _RE_LAUNCH_PASSTHROUGH.match(line)
                    if m2:
                        try:
                            name = _RE_LAUNCH_SEQ_SUFFIX.sub("", m2["name"])
                            name = _RE_EXEC_MAIN_SUFFIX.sub(r"\1", name)
                            msg = m2["msg"].rstrip()
                            # Classify *before* stripping the rcutils prefix
                            # — _classify_passthrough reads the embedded
                            # `[LEVEL]` token to assign severity.
                            level = _classify_passthrough(msg)
                            msg = _RE_RCUTILS_PREFIX.sub("", msg)
                            out.append(
                                (
                                    float(m2["ts"]),
                                    level,
                                    name,
                                    msg,
                                )
                            )
                        except ValueError:
                            pass
        except OSError:
            continue
    return out


def slice_log(
    all_lines: list[tuple[float, str, str, str]],
    starts: list[float],
    t_start: float,
    t_end: float,
) -> list[tuple[float, str, str, str]]:
    lo = bisect.bisect_left(starts, t_start)
    hi = bisect.bisect_right(starts, t_end)
    return all_lines[lo:hi]


def classify(tc: ET.Element) -> str:
    if tc.find("failure") is not None:
        return "failed"
    if tc.find("error") is not None:
        return "error"
    if tc.find("skipped") is not None:
        return "skipped"
    return "passed"


# ---------- main ----------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Render a pytest JUnit xunit.xml to an HTML report.",
    )
    parser.add_argument("input", type=Path, help="Path to xunit.xml")
    parser.add_argument("output", type=Path, help="Path to write HTML report")
    parser.add_argument(
        "rosout_ndjson",
        type=Path,
        nargs="?",
        default=None,
        help="Path to rosout.ndjson "
        "(defaults to <input.parent>/ros_logs/rosout.ndjson)",
    )
    args = parser.parse_args()

    in_path = args.input
    out_path = args.output
    rosout_path = (
        args.rosout_ndjson
        if args.rosout_ndjson is not None
        else in_path.parent / "ros_logs" / "rosout.ndjson"
    )

    # Parse defensively: a truncated upload or a runner that crashed before
    # pytest wrote any results will yield malformed/empty XML. The render-report
    # job is `needs:` for the sticky-comment job, so a crash here loses the
    # comment exactly when it would be most useful. Degrade gracefully instead.
    try:
        tree = ET.parse(in_path)
    except ET.ParseError as exc:
        print(f"render_report: could not parse {in_path}: {exc}", file=sys.stderr)
        sys.exit(0)
    root = tree.getroot()
    suite = root.find("testsuite") if root.tag == "testsuites" else root
    if suite is None:
        print(
            f"render_report: no <testsuite> in {in_path}; nothing to render",
            file=sys.stderr,
        )
        sys.exit(0)

    timestamp = suite.attrib.get("timestamp", "")
    hostname = suite.attrib.get("hostname", "")
    total_time = float(suite.attrib.get("time", 0))
    suite_unix = parse_iso_to_unix(timestamp)

    # Load ROS logs (file logger + launch-wrapped stdout) once.
    # Primary source: structured /rosout NDJSON captured by conftest. Secondary
    # source: launch.log stdout/stderr passthrough, critical for boot failures
    # where a node SIGABRTs before it publishes anything to /rosout.
    ros_lines = sorted(
        load_rosout_ndjson(rosout_path) + load_launch_logs(rosout_path.parent),
        key=lambda x: x[0],
    )
    ros_starts = [entry[0] for entry in ros_lines]

    groups: dict[str, list[dict]] = defaultdict(list)
    counts = {"passed": 0, "failed": 0, "error": 0, "skipped": 0}
    cumulative_time = 0.0
    first_non_skipped_seen = False

    for tc in suite.findall("testcase"):
        status = classify(tc)
        counts[status] += 1
        parent, fname, full = split_path(tc.attrib.get("name", ""))
        time_s = float(tc.attrib.get("time", 0))

        # Compute per-test log window. Pad it generously — pytest's <testcase
        # time> only covers the test call, not fixture setup/teardown, so the
        # cumulative-time cursor drifts a second or two per test against the
        # log file's wall-clock timestamps. Without padding, very-short tests
        # (~0.5s failures from BT-load errors) miss their own error line.
        pad_before = 2.0
        pad_after = 5.0
        t_start = (suite_unix + cumulative_time) if suite_unix else None
        t_end = (suite_unix + cumulative_time + time_s) if suite_unix else None
        # Skipped tests don't actually consume sim time, so don't advance the
        # cumulative cursor for them.
        if status != "skipped":
            cumulative_time += time_s

        # The first non-skipped test absorbs any pre-suite log entries
        # (e.g., a backend node crashing during launch.bringup before the
        # suite timestamp was emitted). Without this, boot-failure stack
        # traces in launch.log fall outside every test's window and get
        # rendered as "No ROS log lines in this test's time window."
        extend_back_to: float | None = None
        if (
            t_start is not None
            and status != "skipped"
            and not first_non_skipped_seen
            and ros_starts
            and ros_starts[0] < t_start
        ):
            extend_back_to = ros_starts[0]
        if status != "skipped":
            first_non_skipped_seen = True

        if t_start is not None and ros_lines:
            # extend_back_to is a widen-only override: take whichever is earlier
            # so the first-test window never gets narrower than any other test's.
            window_start = t_start - pad_before
            if extend_back_to is not None:
                window_start = min(window_start, extend_back_to)
            log_slice = slice_log(
                ros_lines, ros_starts, window_start, t_end + pad_after
            )
        else:
            log_slice = []

        warn_n = sum(1 for _, lvl, _, _ in log_slice if lvl == "WARN")
        err_n = sum(1 for _, lvl, _, _ in log_slice if lvl in ("ERROR", "FATAL"))

        groups[parent].append(
            {
                "status": status,
                "fname": fname,
                "full": full,
                "objective_name": extract_objective_name(full),
                "time": time_s,
                "t_start": t_start,
                "t_end": t_end,
                "log_slice": log_slice,
                "warn_n": warn_n,
                "err_n": err_n,
            }
        )

    status_rank = {"failed": 0, "error": 1, "passed": 2, "skipped": 3}
    for tests in groups.values():
        tests.sort(key=lambda r: (status_rank[r["status"]], -r["time"], r["fname"]))

    def group_key(item):
        parent, tests = item
        fails = sum(1 for t in tests if t["status"] in ("failed", "error"))
        return (-fails, parent)

    sorted_groups = sorted(groups.items(), key=group_key)
    total = sum(counts.values())

    executed_times = [
        t["time"]
        for tests in groups.values()
        for t in tests
        if t["status"] != "skipped"
    ]
    avg_exec_time = (
        (sum(executed_times) / len(executed_times)) if executed_times else 0.0
    )

    def status_badge(s: str) -> str:
        symbols = {"passed": "✓", "failed": "✕", "error": "!", "skipped": "−"}
        return f'<span class="badge badge-{s}">{symbols[s]} {s}</span>'

    def render_log_slice(log_slice: list[tuple[float, str, str, str]]) -> str:
        if not log_slice:
            return '<div class="logs-empty">No ROS log lines in this test\'s time window.</div>'
        # Fold every duplicate (level, node, msg) in the slice down to one
        # entry, anchored at the first occurrence's timestamp, with a count
        # badge for repeats. Strict-consecutive collapsing misses cases where
        # a retry loop's messages interleave with other nodes' output — group
        # collapsing handles those.
        seen: dict[tuple[str, str, str], list] = {}
        ordered_keys: list[tuple[str, str, str]] = []
        for ts, lvl, node, msg in log_slice:
            key = (lvl, node, msg)
            if key in seen:
                seen[key][1] += 1
            else:
                seen[key] = [ts, 1]
                ordered_keys.append(key)
        collapsed: list[tuple[float, str, str, str, int]] = [
            (seen[k][0], k[0], k[1], k[2], seen[k][1]) for k in ordered_keys
        ]

        rows: list[str] = []
        anchor = collapsed[0][0]
        for ts, lvl, node, msg, count in collapsed:
            rel = ts - anchor
            level_class = (
                lvl.lower()
                if lvl in ("ERROR", "FATAL", "WARN", "INFO", "DEBUG")
                else "info"
            )
            count_badge = (
                f' <span class="log-count">×{count}</span>' if count > 1 else ""
            )
            rows.append(
                f'<div class="log-line log-{level_class}">'
                f'<span class="log-ts">+{rel:6.2f}s</span>'
                f'<span class="log-level">{lvl}</span>'
                f'<span class="log-node">{html.escape(node)}</span>'
                f'<span class="log-msg">{html.escape(msg)}{count_badge}</span>'
                f"</div>"
            )
        return f'<div class="logs">{"".join(rows)}</div>'

    section_parts: list[str] = []
    test_idx = 0
    for parent, tests in sorted_groups:
        g_fails = sum(1 for t in tests if t["status"] in ("failed", "error"))
        g_passes = sum(1 for t in tests if t["status"] == "passed")
        g_skips = sum(1 for t in tests if t["status"] == "skipped")
        chips: list[str] = []
        if g_fails:
            chips.append(f'<span class="chip chip-failed">{g_fails} fail</span>')
        if g_passes:
            chips.append(f'<span class="chip chip-passed">{g_passes} pass</span>')
        if g_skips:
            chips.append(f'<span class="chip chip-skipped">{g_skips} skip</span>')

        rows_html: list[str] = []
        for r in tests:
            has_logs = bool(r["log_slice"])
            details_html = (
                f'<tr class="details" id="d{test_idx}" hidden>'
                f'<td colspan="5">{render_log_slice(r["log_slice"])}</td></tr>'
            )
            click = (
                f"onclick=\"document.getElementById('d{test_idx}').hidden = "
                f"!document.getElementById('d{test_idx}').hidden\""
            )
            cursor = 'style="cursor: pointer"'

            # Replace the pytest-fail message with a log summary.
            if has_logs:
                pieces: list[str] = []
                if r["err_n"]:
                    pieces.append(
                        f'<span class="sum-err">{r["err_n"]} error{"s" if r["err_n"] != 1 else ""}</span>'
                    )
                if r["warn_n"]:
                    pieces.append(
                        f'<span class="sum-warn">{r["warn_n"]} warning{"s" if r["warn_n"] != 1 else ""}</span>'
                    )
                info_n = sum(1 for _, lvl, _, _ in r["log_slice"] if lvl == "INFO")
                if info_n:
                    pieces.append(f'<span class="sum-info">{info_n} info</span>')
                msg_cell = " · ".join(pieces)
            elif r["status"] == "skipped":
                msg_cell = '<span class="msg-skipped">skipped</span>'
            else:
                msg_cell = '<span class="msg-empty">no logs</span>'

            obj_cell = (
                f'<td class="obj-name">{html.escape(r["objective_name"])}</td>'
                if r["objective_name"]
                else '<td class="obj-name obj-missing">—</td>'
            )
            search_field = html.escape(
                (r["objective_name"] or "") + " " + r["fname"]
            ).lower()
            rows_html.append(
                f'<tr data-status="{r["status"]}" data-name="{search_field}" {click} {cursor}>'
                f'<td class="status-cell">{status_badge(r["status"])}</td>'
                f"{obj_cell}"
                f'<td class="fname">{html.escape(r["fname"])}</td>'
                f'<td class="time">{r["time"]:.1f}s</td>'
                f'<td class="msg">{msg_cell}</td>'
                f"</tr>{details_html}"
            )
            test_idx += 1

        header_class = (
            "group-failed"
            if g_fails
            else ("group-skipped" if g_skips and not g_passes else "group-passed")
        )
        section_parts.append(
            f'<section class="group {header_class}">'
            f'<div class="group-header" onclick="this.parentElement.classList.toggle(\'collapsed\')">'
            f'<div class="group-left">'
            f'<span class="chevron">▾</span>'
            f'<code class="group-path" title="{html.escape(parent)}">'
            f'{html.escape(parent or "(no parent)")}</code>'
            f"</div>"
            f'<div class="group-chips">{"".join(chips)}</div>'
            f"</div>"
            f'<table><tbody>{"".join(rows_html)}</tbody></table>'
            f"</section>"
        )

    pass_rate_denom = counts["passed"] + counts["failed"] + counts["error"]
    pass_rate = (counts["passed"] / pass_rate_denom * 100) if pass_rate_denom else 0

    # Default to the "Failed" filter when failures exist (post-mortem is the
    # primary use case for this report). When every test passed, "Failed"
    # would render an empty list, so default to "All" instead.
    has_failures = counts["failed"] + counts["error"] > 0
    default_filter = "failed" if has_failures else "all"
    all_active = "" if has_failures else " active"
    failed_active = " active" if has_failures else ""

    html_out = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>MoveIt Pro Objective Integration Tests</title>
<link rel="icon" href="https://docs.picknik.ai/img/favicon.ico" type="image/x-icon">
<style>
  * {{ box-sizing: border-box; }}
  body {{
    margin: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    color: #1f2328;
    background: #f6f8fa;
    line-height: 1.5;
  }}
  header {{ background: #fff; border-bottom: 1px solid #d0d7de; padding: 20px 32px; }}
  h1 {{ margin: 0 0 4px; font-size: 20px; font-weight: 600; }}
  .meta {{ color: #57606a; font-size: 13px; }}
  .meta span + span:before {{ content: " · "; }}
  main {{ max-width: 1800px; margin: 24px auto; padding: 0 32px; width: 95%; }}
  .stats {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
    gap: 12px;
    margin-bottom: 24px;
  }}
  .stat {{
    background: #fff; border: 1px solid #d0d7de; border-radius: 8px;
    padding: 16px; text-align: center;
  }}
  .stat-count {{ font-size: 28px; font-weight: 600; }}
  .stat-label {{ font-size: 12px; text-transform: uppercase; letter-spacing: 0.5px; color: #57606a; margin-top: 4px; }}
  .stat-passed .stat-count {{ color: #1a7f37; }}
  .stat-failed .stat-count {{ color: #cf222e; }}
  .stat-skipped .stat-count {{ color: #6e7781; }}
  .stat-total .stat-count {{ color: #1f2328; }}
  .controls {{
    background: #fff; border: 1px solid #d0d7de; border-radius: 8px;
    padding: 12px 16px; margin-bottom: 16px;
    display: flex; gap: 8px; align-items: center; flex-wrap: wrap;
  }}
  .filter-btn {{
    border: 1px solid #d0d7de; background: #f6f8fa;
    padding: 4px 10px; border-radius: 6px; font-size: 12px; cursor: pointer; color: #24292f;
  }}
  .filter-btn.active {{ background: #1f2328; color: #fff; border-color: #1f2328; }}
  .filter-btn:hover {{ background: #eaeef2; }}
  .filter-btn.active:hover {{ background: #1f2328; }}
  input[type=search] {{
    flex: 1; min-width: 200px;
    border: 1px solid #d0d7de; border-radius: 6px;
    padding: 5px 10px; font-size: 13px; font-family: inherit;
  }}
  .toggle {{
    display: inline-flex; align-items: center; gap: 6px;
    font-size: 12px; color: #57606a; cursor: pointer; user-select: none;
    padding: 4px 10px; border: 1px solid #d0d7de; border-radius: 6px; background: #f6f8fa;
  }}
  .toggle:hover {{ background: #eaeef2; }}
  .toggle input {{ margin: 0; }}
  body.hide-info .log-info, body.hide-info .log-debug {{ display: none; }}
  .group {{ background: #fff; border: 1px solid #d0d7de; border-radius: 8px; margin-bottom: 16px; overflow: hidden; }}
  .group-failed  {{ border-left: 4px solid #cf222e; }}
  .group-passed  {{ border-left: 4px solid #1a7f37; }}
  .group-skipped {{ border-left: 4px solid #d0d7de; }}
  .group-header {{
    padding: 10px 16px; background: #f6f8fa; border-bottom: 1px solid #d0d7de;
    display: flex; justify-content: space-between; align-items: center;
    gap: 16px; flex-wrap: wrap; cursor: pointer; user-select: none;
  }}
  .group-header:hover {{ background: #eaeef2; }}
  .group-left {{ display: flex; align-items: center; gap: 8px; min-width: 0; flex: 1; }}
  .chevron {{
    flex-shrink: 0; color: #57606a; font-size: 10px;
    transition: transform 0.15s ease; display: inline-block; width: 12px;
  }}
  .group.collapsed .chevron {{ transform: rotate(-90deg); }}
  .group.collapsed table {{ display: none; }}
  .group.collapsed {{ border-bottom-left-radius: 8px; border-bottom-right-radius: 8px; }}
  .group.collapsed .group-header {{ border-bottom: none; }}
  .group-path {{ font-family: ui-monospace, "SF Mono", Menlo, monospace; font-size: 12px; color: #57606a; word-break: break-all; }}
  .group-chips {{ display: flex; gap: 6px; flex-shrink: 0; }}
  .chip {{ display: inline-block; padding: 1px 8px; border-radius: 10px; font-size: 11px; font-weight: 600; }}
  .chip-passed  {{ background: #dafbe1; color: #1a7f37; }}
  .chip-failed  {{ background: #ffebe9; color: #cf222e; }}
  .chip-skipped {{ background: #eaeef2; color: #6e7781; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  td {{ padding: 6px 16px; border-bottom: 1px solid #eaeef2; }}
  tr:last-child td {{ border-bottom: none; }}
  tr:hover {{ background: #f6f8fa; }}
  tr.details:hover {{ background: #fff; }}
  tr.details td {{ padding: 0; background: #fafbfc; }}

  /* Log slice rendering */
  .logs {{
    background: #0d1117; color: #c9d1d9;
    font-family: ui-monospace, "SF Mono", Menlo, monospace; font-size: 11px;
    max-height: 720px; overflow-y: auto; padding: 0;
  }}
  .log-line {{
    display: grid;
    grid-template-columns: 64px 60px 220px 1fr;
    gap: 12px; padding: 2px 16px;
    border-left: 3px solid transparent;
  }}
  .log-line:hover {{ background: rgba(255,255,255,0.04); }}
  .log-ts    {{ color: #6e7681; text-align: right; }}
  .log-level {{ font-weight: 600; }}
  .log-node  {{ color: #79c0ff; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
  .log-msg   {{ color: #c9d1d9; white-space: pre-wrap; word-break: break-word; }}
  .log-count {{
    display: inline-block; margin-left: 8px;
    background: rgba(255,255,255,0.12); color: #c9d1d9;
    padding: 0 6px; border-radius: 8px;
    font-size: 10px; font-weight: 600;
  }}
  .log-error, .log-fatal {{
    background: rgba(207, 34, 46, 0.18);
    border-left-color: #ff7b72;
  }}
  .log-error .log-level, .log-fatal .log-level {{ color: #ff7b72; }}
  .log-warn {{
    background: rgba(187, 128, 9, 0.16);
    border-left-color: #d29922;
  }}
  .log-warn  .log-level {{ color: #d29922; }}
  .log-info  .log-level {{ color: #56d364; }}
  .log-debug {{ color: #6e7681; }}
  .log-debug .log-level {{ color: #6e7681; }}
  .logs-empty {{
    padding: 16px; color: #6e7781; font-size: 12px; background: #fafbfc;
    font-style: italic;
  }}

  .status-cell {{ width: 110px; text-align: center; }}
  .badge {{ display: inline-block; padding: 2px 8px; border-radius: 12px; font-size: 11px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.4px; white-space: nowrap; }}
  .badge-passed  {{ background: #dafbe1; color: #1a7f37; }}
  .badge-failed  {{ background: #ffebe9; color: #cf222e; }}
  .badge-error   {{ background: #ffebe9; color: #cf222e; }}
  .badge-skipped {{ background: #eaeef2; color: #6e7781; }}
  .fname {{ font-family: ui-monospace, "SF Mono", Menlo, monospace; font-size: 12px; color: #6e7781; }}
  .obj-name {{ font-weight: 500; color: #0969da; }}
  .obj-missing {{ color: #8c959f; font-weight: 400; }}
  .time {{ font-family: ui-monospace, "SF Mono", Menlo, monospace; font-size: 12px; color: #57606a; text-align: right; width: 70px; }}
  .msg {{ color: #57606a; font-size: 12px; }}
  .msg-skipped {{ color: #8c959f; font-style: italic; }}
  .msg-empty {{ color: #8c959f; font-style: italic; }}
  .sum-err  {{ color: #cf222e; font-weight: 600; }}
  .sum-warn {{ color: #9a6700; font-weight: 600; }}
  .sum-info {{ color: #6e7781; }}
  .more {{ color: #0969da; font-weight: 500; }}
  .group.hidden {{ display: none; }}
</style>
</head>
<body>
<header>
  <h1>MoveIt Pro Objective Integration Tests</h1>
  <div class="meta">
    <span>{total} tests</span>
    <span>{format_duration(total_time)} total</span>
    <span>{html.escape(format_timestamp(timestamp))}</span>
    <span>{html.escape(hostname)}</span>
    <span>{len(ros_lines):,} log lines indexed (rosout + launch)</span>
  </div>
</header>
<main>
  <div class="stats">
    <div class="stat stat-total"><div class="stat-count">{total}</div><div class="stat-label">Total objective tests</div></div>
    <div class="stat stat-passed"><div class="stat-count">{counts["passed"]}</div><div class="stat-label">Objectives passed</div></div>
    <div class="stat stat-failed"><div class="stat-count">{counts["failed"] + counts["error"]}</div><div class="stat-label">Objectives failed</div></div>
    <div class="stat stat-skipped"><div class="stat-count">{counts["skipped"]}</div><div class="stat-label">Objectives skipped</div></div>
    <div class="stat stat-total"><div class="stat-count">{avg_exec_time:.1f}s</div><div class="stat-label">Avg test time</div></div>
    <div class="stat stat-total"><div class="stat-count">{pass_rate:.0f}%</div><div class="stat-label">Objective pass rate</div></div>
  </div>
  <div class="controls">
    <button class="filter-btn{all_active}" data-filter="all">All</button>
    <button class="filter-btn{failed_active}" data-filter="failed">Failed</button>
    <button class="filter-btn" data-filter="passed">Passed</button>
    <button class="filter-btn" data-filter="skipped">Skipped</button>
    <input type="search" id="search" placeholder="Filter by name…">
    <label class="toggle"><input type="checkbox" id="show-info" checked> Show INFO/DEBUG</label>
  </div>
  {''.join(section_parts)}
</main>
<script>
  const buttons = document.querySelectorAll('.filter-btn');
  const search = document.getElementById('search');
  let currentFilter = '{default_filter}';

  function apply() {{
    const q = search.value.toLowerCase();
    document.querySelectorAll('.group').forEach(group => {{
      let visible = 0;
      group.querySelectorAll('tbody tr:not(.details)').forEach(row => {{
        const status = row.dataset.status;
        const name = row.dataset.name;
        const matchFilter = currentFilter === 'all' || (currentFilter === 'failed' ? (status === 'failed' || status === 'error') : status === currentFilter);
        const matchSearch = !q || name.includes(q);
        const show = matchFilter && matchSearch;
        row.style.display = show ? '' : 'none';
        const details = row.nextElementSibling;
        if (details && details.classList.contains('details')) {{
          if (!show) {{ details.hidden = true; }}
          details.style.display = show ? '' : 'none';
        }}
        if (show) visible++;
      }});
      group.classList.toggle('hidden', visible === 0);
    }});
  }}

  buttons.forEach(b => b.addEventListener('click', () => {{
    buttons.forEach(x => x.classList.remove('active'));
    b.classList.add('active');
    currentFilter = b.dataset.filter;
    apply();
  }}));
  search.addEventListener('input', apply);

  const showInfo = document.getElementById('show-info');
  // Sync the body's hide-info class to the checkbox state on initial render
  // (and on subsequent change). When checked, full per-objective context is
  // visible by default — INFO/DEBUG/WARN/ERROR/FATAL — which is what we want
  // when triaging a failing test.
  document.body.classList.toggle('hide-info', !showInfo.checked);
  showInfo.addEventListener('change', () => {{
    document.body.classList.toggle('hide-info', !showInfo.checked);
  }});

  // Apply the default filter ("Failed") to the test rows on initial load.
  // Without this, the "Failed" button is visually active but the apply()
  // logic that hides non-matching rows hasn't run yet, so skipped/passed
  // rows stay visible until the user clicks a filter button.
  apply();
</script>
</body>
</html>
"""
    out_path.write_text(html_out, encoding="utf-8")
    print(f"Wrote {out_path} ({len(html_out):,} bytes)")


if __name__ == "__main__":
    main()
