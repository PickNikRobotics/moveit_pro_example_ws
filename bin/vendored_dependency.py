#!/usr/bin/env python3
"""Report how far each vendored dependency has drifted from its upstream branch."""

from dataclasses import dataclass
from pathlib import Path
import argparse
import json
import os
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlparse
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_workspace_dependencies as validator  # noqa: E402

COMPARE_TIMEOUT_SECONDS = 30
COMPARE_MAX_RESPONSE_BYTES = 16 * 1024 * 1024
TABLE_HEADER = ("Vendored source", "Pinned commit", "Upstream branch", "Commits behind")


@dataclass(frozen=True)
class Drift:
    """Drift of one vendored source relative to its pinned upstream branch."""

    source: str
    commit: str = ""
    branch: str = ""
    behind: int | None = None
    error: str | None = None


def fetch_drift(
    repository: str, commit: str, branch: str
) -> tuple[int | None, str | None]:
    """Return how many commits the upstream branch has beyond the pinned commit."""
    repository_path = urlparse(repository).path.removeprefix("/").removesuffix(".git")
    basehead = f"{quote(commit, safe='')}...{quote(branch, safe='')}"
    url = f"https://api.github.com/repos/{repository_path}/compare/{basehead}"
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "moveit-pro-workspace-vendored-dependency",
        "X-GitHub-Api-Version": "2022-11-28",
    }
    if token := os.environ.get("GITHUB_TOKEN"):
        headers["Authorization"] = f"Bearer {token}"
    try:
        with urlopen(
            Request(url, headers=headers), timeout=COMPARE_TIMEOUT_SECONDS
        ) as response:
            payload = response.read(COMPARE_MAX_RESPONSE_BYTES + 1)
    except (HTTPError, URLError, OSError, TimeoutError, ValueError) as error:
        return None, f"could not compare against upstream branch {branch}: {error}"
    if len(payload) > COMPARE_MAX_RESPONSE_BYTES:
        return None, "upstream compare response exceeds the size limit"
    try:
        data = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None, "upstream compare response is malformed"
    if not isinstance(data, dict):
        return None, "upstream compare response is malformed"
    ahead_by = data.get("ahead_by")
    behind_by = data.get("behind_by")
    # bool is an int subclass, so check the exact type: a count must be a number.
    if type(ahead_by) is not int or type(behind_by) is not int:
        return None, "upstream compare response is malformed"
    if behind_by:
        return None, f"pinned commit {commit} is not on upstream branch {branch}"
    return ahead_by, None


def collect_drift() -> list[Drift]:
    """Look up drift for every vendored source that has a valid manifest."""
    manifests, discovery_errors = validator.discover_vendoring_manifests()
    rows = [
        Drift(source="src/external_dependencies", error=e) for e in discovery_errors
    ]
    for manifest_path in manifests:
        source = manifest_path.parent.relative_to(validator.REPOSITORY_ROOT).as_posix()
        if manifest_errors := validator.validate_vendor_manifest(manifest_path):
            rows.append(Drift(source=source, error="; ".join(manifest_errors)))
            continue
        upstream = validator.parse_vendor_manifest(manifest_path)["upstream"]
        if not isinstance(upstream, dict):
            rows.append(Drift(source=source, error="has invalid upstream metadata"))
            continue
        repository = str(upstream["repository"])
        commit = str(upstream["commit"])
        branch = str(upstream["branch"])
        behind, error = fetch_drift(repository, commit, branch)
        rows.append(Drift(source, commit, branch, behind, error))
    rows.sort(key=lambda row: (row.behind is None, -(row.behind or 0), row.source))
    return rows


def escape_markdown_cell(cell: str) -> str:
    """Keep a literal backslash or pipe from being read as table syntax."""
    return cell.replace("\\", "\\\\").replace("|", "\\|")


def render_table(rows: list[Drift], *, markdown: bool) -> str:
    """Render drift rows as an aligned text table or a GitHub Markdown table."""
    body = [
        (
            row.source.removeprefix("src/external_dependencies/"),
            row.commit[:9] or "-",
            row.branch or "-",
            "?" if row.behind is None else str(row.behind),
        )
        for row in rows
    ]
    if markdown:
        lines = [
            "## Vendored dependency drift",
            "",
            "| " + " | ".join(TABLE_HEADER) + " |",
            "|---|---|---|---:|",
        ]
        lines.extend(
            "| " + " | ".join(escape_markdown_cell(cell) for cell in cells) + " |"
            for cells in body
        )
        return "\n".join(lines) + "\n"
    widths = [
        max(len(cells[column]) for cells in (TABLE_HEADER, *body))
        for column in range(len(TABLE_HEADER))
    ]
    lines = []
    for cells in (TABLE_HEADER, *body):
        *left, right = cells
        padded = [cell.ljust(width) for cell, width in zip(left, widths[:-1])]
        lines.append("  ".join([*padded, right.rjust(widths[-1])]).rstrip())
    return "\n".join(lines) + "\n"


def status(*, markdown: bool) -> int:
    rows = collect_drift()
    write_github_output(rows)
    if not rows:
        print("No vendored dependencies found.")
        return 0
    print(render_table(rows, markdown=markdown), end="")
    failures = [row for row in rows if row.error is not None]
    for row in failures:
        print(f"ERROR: {row.source} {row.error}", file=sys.stderr)
    return 1 if failures else 0


def write_github_output(rows: list[Drift]) -> None:
    """Expose the verdict to a GitHub Actions step when GITHUB_OUTPUT is set."""
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        return
    resolved = [row for row in rows if row.behind is not None]
    drifted = any(row.behind for row in resolved)
    with open(output_path, "a", encoding="utf-8") as output:
        output.write(f"drifted={str(drifted).lower()}\n")
        output.write(f"resolved={len(resolved)}\n")
        output.write(f"unresolved={len(rows) - len(resolved)}\n")


def main(argv: list[str] | None = None) -> int:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    subparsers = argument_parser.add_subparsers(dest="command", required=True)
    status_parser = subparsers.add_parser(
        "status",
        help="report how many upstream commits each vendored source is behind",
    )
    status_parser.add_argument(
        "--markdown",
        action="store_true",
        help="emit a GitHub Markdown table, for example for GITHUB_STEP_SUMMARY",
    )
    arguments = argument_parser.parse_args(argv)
    return status(markdown=arguments.markdown)


if __name__ == "__main__":
    sys.exit(main())
