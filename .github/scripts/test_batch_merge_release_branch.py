"""Exercise release-merge PR replacement using the workflow's actual shell steps.

Ported from moveit_pro, with its combined merge/create step mocked locally.
"""

import json
import os
import subprocess
from pathlib import Path

import pytest
import yaml

WORKFLOW = (
    Path(__file__).resolve().parents[1] / "workflows/batch-merge-release-branch.yaml"
)
REPOSITORY = "PickNikRobotics/moveit_pro_example_ws"


def pull(number: int, branch: str = "v10.2", **overrides) -> dict:
    """Build a representative GitHub pull-request API response."""
    return {
        "number": number,
        "state": "open",
        "title": f"Merge {branch} into main",
        "base": {"ref": "main"},
        "head": {
            "ref": f"merge-{branch}-main-{number}",
            "repo": {"full_name": REPOSITORY},
        },
        **overrides,
    }


@pytest.fixture
def run_workflow(tmp_path):
    """Run merge/creation and cleanup with fake GitHub and Git commands, but real jq."""
    steps = yaml.safe_load(WORKFLOW.read_text())["jobs"]["batch-merge-release-branch"][
        "steps"
    ]
    create = next(step for step in steps if step.get("id") == "merge")
    cleanup = next(
        step for step in steps if step["name"] == "Close superseded merge pull requests"
    )
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    gh = bin_dir / "gh"
    gh.write_text(
        "#!/usr/bin/env python3\n"
        "import json, os, sys\n"
        "args = sys.argv[1:]\n"
        "with open(os.environ['CALLS'], 'a') as log:\n"
        "    log.write(json.dumps(args) + '\\n')\n"
        "if args[:2] == ['pr', 'create']:\n"
        "    if os.environ['FAIL'] == 'create': sys.exit(1)\n"
        "    print('https://github.com/' + os.environ['GITHUB_REPOSITORY'] + '/pull/100')\n"
        "elif args[0] == 'api':\n"
        "    print(os.environ['PAGES'])\n"
        "    if os.environ['FAIL'] == 'list': sys.exit(1)\n"
        "elif args[:2] == ['pr', 'close']:\n"
        "    if os.environ['FAIL'] == 'close': sys.exit(1)\n"
        "    if os.environ['FAIL'] == 'second-close' and args[2] == '2': sys.exit(1)\n"
        "else: sys.exit(2)\n"
    )
    gh.chmod(0o755)
    git = bin_dir / "git"
    git.write_text(
        "#!/usr/bin/env bash\n"
        'if [ "$1" = merge-tree ]; then\n'
        '  if [ "$FAIL" = merge-tree ]; then exit 2; fi\n'
        '  if [ "$FAIL" = noop ]; then echo main-tree; else echo merged-tree; fi\n'
        'elif [ "$1" = rev-parse ]; then echo main-tree\n'
        'elif [ "$1" = merge ] && [ "$FAIL" = conflict ]; then exit 1\n'
        'elif [ "$1" = push ] && [ "$FAIL" = push ]; then exit 1\n'
        "fi\n"
        "exit 0\n"
    )
    git.chmod(0o755)

    def run(pages: list[list[dict]], fail: str = "", branch: str = "v10.2"):
        calls_file = tmp_path / "calls"
        calls_file.write_text("")
        output = tmp_path / "output"
        output.write_text("")
        env = {
            **os.environ,
            "PATH": f"{bin_dir}:{os.environ['PATH']}",
            "CALLS": str(calls_file),
            "FAIL": fail,
            "PAGES": "\n".join(json.dumps(page) for page in pages),
            "GITHUB_OUTPUT": str(output),
            "GITHUB_STEP_SUMMARY": str(tmp_path / "summary"),
            "GITHUB_REPOSITORY": REPOSITORY,
            "RELEASE_BRANCH": branch,
        }
        result = subprocess.run(
            ["bash", "-e", "-o", "pipefail", "-c", create["run"]],
            env=env,
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        # Mirror the cleanup step's implicit success() and explicit skip condition.
        if result.returncode == 0 and "skip=true" not in output.read_text():
            env["NEW_PR_NUMBER"] = output.read_text().strip().removeprefix("number=")
            result = subprocess.run(
                ["bash", "-e", "-o", "pipefail", "-c", cleanup["run"]],
                env=env,
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
        calls = [json.loads(line) for line in calls_file.read_text().splitlines()]
        return result, calls

    return run


@pytest.mark.parametrize("failure", ["merge-tree", "push", "create", "list", "close"])
def test_failures_fail_the_step(run_workflow, failure) -> None:
    """Failed merge, push, creation or listing cannot close existing PRs."""
    result, calls = run_workflow([[pull(1)]], fail=failure)
    assert result.returncode != 0
    if failure != "close":
        assert not any(call[:2] == ["pr", "close"] for call in calls)


def test_cleanup_stops_after_a_partial_close_failure(run_workflow) -> None:
    result, calls = run_workflow([[pull(1), pull(2), pull(3)]], fail="second-close")
    assert result.returncode != 0
    assert [call[2] for call in calls if call[:2] == ["pr", "close"]] == ["1", "2"]


def test_no_changes_leave_existing_prs_open(run_workflow) -> None:
    result, calls = run_workflow([[pull(1)]], fail="noop")
    assert result.returncode == 0, result.stderr
    assert calls == []


def test_no_open_prs_only_creates_replacement(run_workflow) -> None:
    result, calls = run_workflow([[]])
    assert result.returncode == 0, result.stderr
    assert not any(call[:2] == ["pr", "close"] for call in calls)


def test_unrelated_closed_and_newer_prs_stay_untouched(run_workflow) -> None:
    """Only older open same-source workflow PRs are eligible, never manual or fork PRs."""
    candidates = [
        pull(100),
        pull(101),
        pull(1, "v10.1"),
        pull(2, state="closed"),
        pull(3, base={"ref": "v10.2"}),
        pull(4, title="Manual merge"),
        pull(
            5,
            head={
                "ref": "merge-v10.2-main-5",
                "repo": {"full_name": "fork/moveit_pro_example_ws"},
            },
        ),
        pull(6, head={"ref": "manual-branch", "repo": {"full_name": REPOSITORY}}),
        pull(
            7,
            head={"ref": "merge-v10.2-main-7-extra", "repo": {"full_name": REPOSITORY}},
        ),
        pull(8, head={"ref": "merge-v10.2-main-", "repo": {"full_name": REPOSITORY}}),
        pull(9, head={"ref": "merge-v10.2-main-9", "repo": None}),
    ]
    result, calls = run_workflow([candidates])
    assert result.returncode == 0, result.stderr
    assert not any(call[:2] == ["pr", "close"] for call in calls)


@pytest.mark.parametrize("branch", ["v10.2", "feature/nested.branch"])
@pytest.mark.parametrize("merge_result", ["", "conflict"])
def test_regression_closes_all_older_same_source_prs_after_creation(
    run_workflow, branch, merge_result
) -> None:
    """Both title variants across API pages receive the exact supersession comment."""
    result, calls = run_workflow(
        [
            [pull(99, branch)],
            [
                pull(
                    1,
                    branch,
                    title=f"Merge {branch} into main (conflicts resolved by Claude)",
                )
            ],
        ],
        branch=branch,
        fail=merge_result,
    )
    assert result.returncode == 0, result.stderr
    assert calls[0][:2] == ["pr", "create"]
    assert calls[0][calls[0].index("--repo") + 1] == REPOSITORY
    assert calls[1] == [
        "api",
        "--paginate",
        f"repos/{REPOSITORY}/pulls?state=open&base=main&per_page=100",
    ]
    assert calls[2:] == [
        [
            "pr",
            "close",
            str(number),
            "--repo",
            REPOSITORY,
            "--comment",
            "Superseded by #100",
        ]
        for number in [99, 1]
    ]


def test_cleanup_is_gated_on_successful_creation_and_nonempty_merge() -> None:
    workflow = yaml.safe_load(WORKFLOW.read_text())
    steps = workflow["jobs"]["batch-merge-release-branch"]["steps"]
    create = next(step for step in steps if step.get("id") == "merge")
    cleanup = next(
        step for step in steps if step["name"] == "Close superseded merge pull requests"
    )
    assert steps.index(create) < steps.index(cleanup)
    assert cleanup["if"] == "steps.merge.outputs.skip != 'true'"
    assert cleanup["env"]["NEW_PR_NUMBER"] == "${{ steps.merge.outputs.number }}"
    assert not create.get("continue-on-error", False)
    assert workflow["concurrency"] == {
        "group": "batch-merge-release-branch",
        "cancel-in-progress": False,
    }
