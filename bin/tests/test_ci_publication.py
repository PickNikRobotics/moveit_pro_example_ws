"""Contracts for jobs that share the gh-pages branch."""

from pathlib import Path
import re
import json
import os
import subprocess
import textwrap

from pytest import mark

ROOT = Path(__file__).resolve().parents[2]


def test_pages_writers_share_a_repository_wide_lock():
    """Different PRs must not push the same branch concurrently."""
    groups = []
    for filename, job in (
        ("ci.yaml", "publish-and-comment"),
        ("cleanup-pr-reports.yaml", "cleanup-reports"),
    ):
        workflow = (ROOT / ".github/workflows" / filename).read_text()
        section = workflow.split(f"  {job}:", 1)[1]
        match = re.search(r"group: (.+)", section)
        assert match is not None
        group = match.group(1)
        groups.append(group)
        assert "${{" not in group
        assert "cancel-in-progress: false" in section
        assert "queue: max" in section
    assert groups[0] == groups[1]


@mark.parametrize("outcome", ["success", "failure", "cancelled", "skipped"])
def test_report_comment_links_only_published_pages(outcome):
    workflow = (ROOT / ".github/workflows/ci.yaml").read_text()
    step = workflow.split("      - name: Post PR comment\n", 1)[1].split(
        "\n  ensure-no-ssh-in-gitmodules:", 1
    )[0]
    script = textwrap.dedent(step.split("          script: |\n", 1)[1])
    harness = """
const context = {repo: {owner: 'Example', repo: 'workspace'}, runId: 123, issue: {number: 894}};
const github = {rest: {issues: {createComment: async ({body}) => console.log(body)}}};
(async () => { SCRIPT })().catch(e => {console.error(e); process.exit(1);});
""".replace(
        "SCRIPT", script
    )
    result = subprocess.run(
        ["node", "-e", harness],
        capture_output=True,
        text=True,
        check=True,
        env={
            **os.environ,
            "BASE": "pr-894/run-123",
            "DEPLOY_OUTCOME": outcome,
            "STATUS_MAP": json.dumps(
                {"lab_sim": {"jazzy": {"status": "passed", "passed": 31}}}
            ),
        },
    )
    assert "31 passed" in result.stdout
    assert ("github.io" in result.stdout) is (outcome == "success")
    if outcome != "success":
        assert "https://github.com/Example/workspace/actions/runs/123" in result.stdout


def test_drift_issue_only_reports_failures_with_issue_scoped_token():
    workflow = (ROOT / ".github/workflows/ci.yaml").read_text()
    job = workflow.split("  upstream-drift-issue:", 1)[1].split(
        "\n  validate_objectives:", 1
    )[0]
    assert "always() && needs.verify-upstream-snapshots.result == 'failure'" in job
    assert "permission-issues: write" in job
