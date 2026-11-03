"""Regression tests for the vendored refresh branch guard and head recheck."""

import json
import os
import re
import subprocess
import textwrap
from pathlib import Path

from pytest import MonkeyPatch, mark

WORKFLOW_PATH = Path(__file__).resolve().parents[2] / ".github/workflows/ci.yaml"
REPOSITORY = "PickNikRobotics/moveit_pro_example_ws"
SOURCE = "franka_config"
BRANCH = f"vendored-refresh/{SOURCE}"
HEAD = "0123456789abcdef0123456789abcdef01234567"
REFRESH_AUTHOR = "vendored_dependency.py <noreply@picknik.ai>"
OTHER_AUTHOR = "Shane Copenhagen <shane.copenhagen@picknik.ai>"
GUARD = "Check the refresh branch for commits by others"
RECHECK = "Recheck the refresh branch head"
COMPARE_QUERY = (
    r'"\(.total_commits) \(.commits | length)", '
    r'(.commits[].commit | .author, .committer | "\(.name) <\(.email)>")'
)


def refresh_job() -> str:
    """Read the refresh job from the production workflow, not a script copy."""
    return (
        WORKFLOW_PATH.read_text(encoding="utf-8")
        .split("  vendored-refresh:\n", 1)[1]
        .split("\n  validate_objectives:", 1)[0]
    )


def run_refresh_step(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
    step_name: str,
    *,
    head: str | None = HEAD,
    compare: str = "0 0\n",
    ref_error: str = "",
    compare_error: str = "",
    expected_head: str = "",
) -> tuple[subprocess.CompletedProcess[str], dict[str, str], str]:
    """Execute a workflow step with canned API responses and real Bash utilities."""
    job = refresh_job()
    step = job.split(f"      - name: {step_name}\n", 1)[1]
    script = textwrap.dedent(
        re.split(r"\n      [-#]", step.split("        run: |\n", 1)[1], maxsplit=1)[0]
    )
    author = re.search(r"^      REFRESH_AUTHOR: (.+)$", job, re.MULTILINE)
    assert author is not None

    runner = tmp_path / "runner"
    runner.mkdir()
    output = tmp_path / "output"
    summary = tmp_path / "summary"
    output.touch()
    summary.touch()
    ref_error = ref_error or ("gh: Not Found (HTTP 404)" if head is None else "")
    responses = {
        f"repos/{REPOSITORY}/git/ref/heads/{BRANCH}": {
            "query": ".object.sha",
            "stdout": head or "",
            "stderr": ref_error,
            "status": 1 if ref_error else 0,
        },
        f"repos/{REPOSITORY}/compare/main...{head}": {
            "query": COMPARE_QUERY,
            "stdout": compare,
            "stderr": compare_error,
            "status": 1 if compare_error else 0,
        },
    }
    response_file = tmp_path / "responses.json"
    response_file.write_text(json.dumps(responses), encoding="utf-8")
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_gh = fake_bin / "gh"
    fake_gh.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env python3
            import json
            import os
            import sys
            from pathlib import Path

            arguments = sys.argv[1:]
            responses = json.loads(Path(os.environ["GH_RESPONSES"]).read_text())
            response = responses[arguments[1]]
            assert arguments == ["api", arguments[1], "--jq", response["query"]], arguments
            print(response["stdout"], end="")
            print(response["stderr"], file=sys.stderr, end="")
            sys.exit(response["status"])
            """
        ),
        encoding="utf-8",
    )
    fake_gh.chmod(0o755)
    for key, value in {
        "PATH": f"{fake_bin}{os.pathsep}{os.environ['PATH']}",
        "GH_RESPONSES": str(response_file),
        "GH_TOKEN": "test-token",
        "GITHUB_REPOSITORY": REPOSITORY,
        "SOURCE": SOURCE,
        "REFRESH_BRANCH": BRANCH,
        "REFRESH_AUTHOR": author.group(1),
        "EXPECTED_HEAD": expected_head,
        "RUNNER_TEMP": str(runner),
        "GITHUB_OUTPUT": str(output),
        "GITHUB_STEP_SUMMARY": str(summary),
    }.items():
        monkeypatch.setenv(key, value)
    result = subprocess.run(
        ["bash", "-e", "-o", "pipefail", "-c", script],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
        timeout=10,
    )
    outputs = dict(line.split("=", 1) for line in output.read_text().splitlines())
    return result, outputs, summary.read_text()


@mark.parametrize(
    "author,committer",
    [
        (OTHER_AUTHOR, REFRESH_AUTHOR),
        (REFRESH_AUTHOR, OTHER_AUTHOR),
        (OTHER_AUTHOR, OTHER_AUTHOR),
        (REFRESH_AUTHOR, "vendored_dependency.py <shane.copenhagen@picknik.ai>"),
    ],
    ids=["other-author", "other-committer", "both-identities", "other-email"],
)
def test_refresh_guard_skips_commits_with_other_identities(
    tmp_path: Path, monkeypatch: MonkeyPatch, author: str, committer: str
) -> None:
    """Keep hand-made commits, including an amend that retains the refresh author."""
    compare = f"2 2\n{REFRESH_AUTHOR}\n{REFRESH_AUTHOR}\n{author}\n{committer}\n"
    result, outputs, summary = run_refresh_step(
        tmp_path, monkeypatch, GUARD, compare=compare
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert outputs == {"head": HEAD, "skip": "true"}
    assert "::warning::" in result.stdout
    assert f"Delete {BRANCH} to resume weekly refreshes" in result.stdout
    assert SOURCE in summary
    assert BRANCH in summary
    for identity in {author, committer} - {REFRESH_AUTHOR}:
        assert identity in result.stdout
        assert identity in summary


@mark.parametrize("commits", [0, 1, 2])
def test_refresh_guard_allows_only_refresh_identities(
    tmp_path: Path, monkeypatch: MonkeyPatch, commits: int
) -> None:
    """Permit bot-only history and an existing branch with no commits ahead of main."""
    compare = f"{commits} {commits}\n" + f"{REFRESH_AUTHOR}\n" * (2 * commits)
    result, outputs, summary = run_refresh_step(
        tmp_path, monkeypatch, GUARD, compare=compare
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert outputs == {"head": HEAD, "skip": "false"}
    assert summary == ""
    assert "::warning::" not in result.stdout


def test_refresh_guard_allows_a_missing_branch(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """A deleted or not-yet-created branch permits refresh without a comparison."""
    result, outputs, summary = run_refresh_step(
        tmp_path,
        monkeypatch,
        GUARD,
        head=None,
        compare_error="The comparison must not run for a missing branch",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert outputs == {"skip": "false"}
    assert summary == ""


@mark.parametrize("total,listed", [(251, 250), (1, 0)])
def test_refresh_guard_skips_incomplete_comparisons(
    tmp_path: Path, monkeypatch: MonkeyPatch, total: int, listed: int
) -> None:
    """Do not overwrite a branch when an unlisted commit could have another identity."""
    compare = f"{total} {listed}\n" + f"{REFRESH_AUTHOR}\n" * (2 * listed)
    result, outputs, summary = run_refresh_step(
        tmp_path, monkeypatch, GUARD, compare=compare
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert outputs == {"head": HEAD, "skip": "true"}
    assert "::warning::" in result.stdout
    assert f"{BRANCH} has {total} commits, more than the compare API lists" in summary


@mark.parametrize(
    "endpoint,status", [("ref", 401), ("ref", 502), ("compare", 404), ("compare", 502)]
)
def test_refresh_guard_fails_closed_on_api_errors(
    tmp_path: Path, monkeypatch: MonkeyPatch, endpoint: str, status: int
) -> None:
    """Only a missing branch is safe; failed reads or comparisons must stop the job."""
    error = f"gh: API request failed (HTTP {status})"
    result, outputs, summary = run_refresh_step(
        tmp_path,
        monkeypatch,
        GUARD,
        ref_error=error if endpoint == "ref" else "",
        compare_error=error if endpoint == "compare" else "",
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "skip" not in outputs
    assert "::error::" in result.stdout
    assert error in result.stdout
    assert summary == ""


@mark.parametrize(
    "expected_head,head,skip",
    [
        (HEAD, HEAD, "false"),
        ("", None, "false"),
        (HEAD, "f" * 40, "true"),
        (HEAD, None, "true"),
        ("", HEAD, "true"),
    ],
    ids=["unchanged", "still-missing", "moved", "deleted", "created"],
)
def test_refresh_recheck_skips_only_when_the_branch_head_changed(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
    expected_head: str,
    head: str | None,
    skip: str,
) -> None:
    """Detect a push, deletion, or creation during the update before opening the PR."""
    result, outputs, summary = run_refresh_step(
        tmp_path, monkeypatch, RECHECK, head=head, expected_head=expected_head
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert outputs == {"skip": skip}
    if skip == "true":
        assert "::warning::" in result.stdout
        assert f"{BRANCH} moved during the refresh" in summary
        assert SOURCE in summary
    else:
        assert summary == ""
        assert "::warning::" not in result.stdout


@mark.parametrize("status", [401, 502])
def test_refresh_recheck_fails_closed_on_api_errors(
    tmp_path: Path, monkeypatch: MonkeyPatch, status: int
) -> None:
    """An unreadable branch must not permit PR creation or a force-push."""
    error = f"gh: API request failed (HTTP {status})"
    result, outputs, summary = run_refresh_step(
        tmp_path, monkeypatch, RECHECK, ref_error=error, expected_head=HEAD
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert outputs == {}
    assert "::error::" in result.stdout
    assert error in result.stdout
    assert summary == ""


def test_refresh_workflow_gates_updates_and_pr_writes_on_the_guards() -> None:
    """A skip suppresses checkout, refresh, token generation, and the PR-writing action."""
    steps = re.split(r"^      - ", refresh_job(), flags=re.MULTILINE)[1:]
    assert steps[0].startswith(f"name: {GUARD}\n")
    assert "\n        id: guard\n" in steps[0]
    for step in steps[1:]:
        condition = re.search(r"^        if: (.+)$", step, re.MULTILINE)
        assert condition is not None, step
        guard = (
            "recheck"
            if "actions/create-github-app-token@" in step
            or "peter-evans/create-pull-request@" in step
            else "guard"
        )
        assert condition.group(1) == f"steps.{guard}.outputs.skip == 'false'"
    recheck = next(step for step in steps if step.startswith(f"name: {RECHECK}\n"))
    assert "\n        id: recheck\n" in recheck
    assert "EXPECTED_HEAD: ${{ steps.guard.outputs.head }}\n" in recheck
    pr = next(step for step in steps if "peter-evans/create-pull-request@" in step)
    branch = "vendored-refresh/${{ matrix.source.name }}"
    assert f"REFRESH_BRANCH: {branch}\n" in refresh_job()
    assert f"branch: {branch}\n" in pr
    assert "author: ${{ env.REFRESH_AUTHOR }}\n" in pr
    assert "committer: ${{ env.REFRESH_AUTHOR }}\n" in pr
