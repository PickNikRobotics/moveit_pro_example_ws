"""Tests for the vendored dependency drift report."""

import importlib.util
import io
import json
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request

from pytest import CaptureFixture, MonkeyPatch, mark

MODULE_PATH = Path(__file__).resolve().parents[1] / "vendored_dependency.py"
MODULE_SPEC = importlib.util.spec_from_file_location("vendored_dependency", MODULE_PATH)
assert MODULE_SPEC is not None
assert MODULE_SPEC.loader is not None
tool = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(tool)

REPOSITORY = "https://github.com/example/repository.git"
COMMIT = "0123456789abcdef0123456789abcdef01234567"


class FakeResponse(io.BytesIO):
    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def mock_compare(monkeypatch: MonkeyPatch, payload: object) -> list[Request]:
    """Replace urlopen with a canned compare response and record the requests."""
    requests: list[Request] = []

    def fake_urlopen(request: Request, **_: object) -> FakeResponse:
        requests.append(request)
        return FakeResponse(json.dumps(payload).encode())

    monkeypatch.setattr(tool, "urlopen", fake_urlopen)
    return requests


def test_fetch_drift_compares_pin_against_branch(monkeypatch: MonkeyPatch) -> None:
    """Ask GitHub for the commits on the branch that the pinned commit lacks."""
    monkeypatch.setenv("GITHUB_TOKEN", "token")
    requests = mock_compare(monkeypatch, {"ahead_by": 4, "behind_by": 0})
    assert tool.fetch_drift(REPOSITORY, COMMIT, "main") == (4, None)
    assert requests[0].full_url == (
        f"https://api.github.com/repos/example/repository/compare/{COMMIT}...main"
    )
    assert requests[0].get_header("Authorization") == "Bearer token"


def test_fetch_drift_encodes_branch_names(monkeypatch: MonkeyPatch) -> None:
    """A slash in a branch name must not read as a path separator."""
    requests = mock_compare(monkeypatch, {"ahead_by": 0, "behind_by": 0})
    assert tool.fetch_drift(REPOSITORY, COMMIT, "release/1.0") == (0, None)
    assert requests[0].full_url.endswith(f"/compare/{COMMIT}...release%2F1.0")


def test_fetch_drift_reports_pin_off_branch(monkeypatch: MonkeyPatch) -> None:
    """A pin with commits the branch lacks is not on that branch."""
    mock_compare(monkeypatch, {"ahead_by": 4, "behind_by": 2})
    behind, error = tool.fetch_drift(REPOSITORY, COMMIT, "main")
    assert behind is None
    assert error == f"pinned commit {COMMIT} is not on upstream branch main"


def test_fetch_drift_reports_http_failure(monkeypatch: MonkeyPatch) -> None:
    """Surface the HTTP status so a missing branch or bad token is recognizable."""

    def fail(request: Request, **_: object) -> FakeResponse:
        raise HTTPError(request.full_url, 404, "Not Found", {}, None)  # type: ignore[arg-type]

    monkeypatch.setattr(tool, "urlopen", fail)
    behind, error = tool.fetch_drift(REPOSITORY, COMMIT, "main")
    assert behind is None
    assert error is not None
    assert "main" in error
    assert "404" in error


def test_fetch_drift_rejects_oversized_response(monkeypatch: MonkeyPatch) -> None:
    """Stop reading past the response cap instead of parsing an unbounded body."""
    monkeypatch.setattr(tool, "COMPARE_MAX_RESPONSE_BYTES", 4)
    mock_compare(monkeypatch, {"ahead_by": 4, "behind_by": 0})
    assert tool.fetch_drift(REPOSITORY, COMMIT, "main") == (
        None,
        "upstream compare response exceeds the size limit",
    )


@mark.parametrize(
    "payload",
    [
        {"ahead_by": "many"},
        {"ahead_by": True, "behind_by": False},
        {"ahead_by": 4},
        ["not", "a", "dict"],
    ],
)
def test_fetch_drift_rejects_malformed_payload(
    monkeypatch: MonkeyPatch, payload: object
) -> None:
    mock_compare(monkeypatch, payload)
    assert tool.fetch_drift(REPOSITORY, COMMIT, "main") == (
        None,
        "upstream compare response is malformed",
    )


def mock_manifests(
    monkeypatch: MonkeyPatch, tmp_path: Path, drift: dict[str, int | str]
) -> None:
    """Create one manifest per source and answer drift lookups from a table."""
    manifests: list[Path] = []
    for source in drift:
        root = tmp_path / "src" / "external_dependencies" / source
        root.mkdir(parents=True)
        (root / "UPSTREAM.yaml").write_text(
            "upstream:\n"
            f"  repository: {REPOSITORY}\n"
            f"  commit: {COMMIT}\n"
            f"  branch: {source}-branch\n"
            "vendored_paths:\n  - description\n",
            encoding="utf-8",
        )
        manifests.append(root / "UPSTREAM.yaml")
    monkeypatch.setattr(tool.validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(
        tool.validator, "discover_vendoring_manifests", lambda: (manifests, [])
    )
    monkeypatch.setattr(tool.validator, "validate_vendor_manifest", lambda _: [])

    def fake_fetch(*arguments: str) -> tuple[int | None, str | None]:
        result = drift[arguments[2].removesuffix("-branch")]
        return (result, None) if isinstance(result, int) else (None, result)

    monkeypatch.setattr(tool, "fetch_drift", fake_fetch)


def test_status_lists_most_drifted_first(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    mock_manifests(monkeypatch, tmp_path, {"alpha": 2, "beta": 89, "gamma": 0})
    assert tool.main(["status"]) == 0
    assert capsys.readouterr().out == (
        "Vendored source  Pinned commit  Upstream branch  Commits behind\n"
        "beta             012345678      beta-branch                  89\n"
        "alpha            012345678      alpha-branch                  2\n"
        "gamma            012345678      gamma-branch                  0\n"
    )


def test_status_markdown_is_a_step_summary(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    mock_manifests(monkeypatch, tmp_path, {"alpha": 2})
    assert tool.main(["status", "--markdown"]) == 0
    assert capsys.readouterr().out == (
        "## Vendored dependency drift\n\n"
        "| Vendored source | Pinned commit | Upstream branch | Commits behind |\n"
        "|---|---|---|---:|\n"
        "| alpha | 012345678 | alpha-branch | 2 |\n"
    )


def test_status_markdown_escapes_table_syntax(
    monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """A pipe or backslash in a cell must not add columns to the summary table."""
    monkeypatch.setattr(
        tool,
        "collect_drift",
        lambda: [tool.Drift("src/external_dependencies/a|b", COMMIT, "rel\\x|y", 2)],
    )
    assert tool.main(["status", "--markdown"]) == 0
    assert capsys.readouterr().out.splitlines()[-1] == (
        "| a\\|b | 012345678 | rel\\\\x\\|y | 2 |"
    )


def test_status_writes_github_output_when_requested(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """A workflow keys on these outputs rather than parsing the rendered table."""
    output = tmp_path / "github_output"
    monkeypatch.setenv("GITHUB_OUTPUT", str(output))
    mock_manifests(monkeypatch, tmp_path, {"alpha": 0, "beta": 3, "gamma": "boom"})
    assert tool.main(["status"]) == 1
    capsys.readouterr()
    assert output.read_text() == "drifted=true\nresolved=2\nunresolved=1\n"
    output.unlink()
    mock_manifests(monkeypatch, tmp_path / "second", {"alpha": 0})
    assert tool.main(["status"]) == 0
    assert output.read_text() == "drifted=false\nresolved=1\nunresolved=0\n"


def test_status_reports_discovery_and_manifest_errors(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """Broken manifests take the error column instead of hiding behind the table."""
    mock_manifests(monkeypatch, tmp_path, {"alpha": 1})
    manifest = tmp_path / "src" / "external_dependencies" / "alpha" / "UPSTREAM.yaml"
    scalar = tmp_path / "src" / "external_dependencies" / "scalar" / "UPSTREAM.yaml"
    scalar.parent.mkdir()
    scalar.write_text("vendored_paths:\n  - description\n")
    real_parse = tool.validator.parse_vendor_manifest
    monkeypatch.setattr(
        tool.validator,
        "parse_vendor_manifest",
        lambda path: {"upstream": "nope"} if path == scalar else real_parse(path),
    )
    discovery_error = (
        "vendored source has no UPSTREAM.yaml: src/external_dependencies/empty"
    )
    monkeypatch.setattr(
        tool.validator,
        "discover_vendoring_manifests",
        lambda: ([manifest, scalar], [discovery_error]),
    )
    monkeypatch.setattr(
        tool.validator,
        "validate_vendor_manifest",
        lambda path: ["bad manifest"] if path == manifest else [],
    )
    assert tool.main(["status"]) == 1
    captured = capsys.readouterr()
    assert captured.err == (
        f"ERROR: src/external_dependencies {discovery_error}\n"
        "ERROR: src/external_dependencies/alpha bad manifest\n"
        "ERROR: src/external_dependencies/scalar has invalid upstream metadata\n"
    )
    assert captured.out.count("?") == 3


def test_status_with_no_vendored_sources(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """An empty inventory still reports its verdict to a workflow."""
    output = tmp_path / "github_output"
    monkeypatch.setenv("GITHUB_OUTPUT", str(output))
    monkeypatch.setattr(
        tool.validator, "discover_vendoring_manifests", lambda: ([], [])
    )
    assert tool.main(["status"]) == 0
    assert capsys.readouterr().out == "No vendored dependencies found.\n"
    assert output.read_text() == "drifted=false\nresolved=0\nunresolved=0\n"


def test_status_fails_when_a_lookup_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """A failed lookup still prints the table but exits non-zero, last in the table."""
    mock_manifests(monkeypatch, tmp_path, {"alpha": 2, "beta": "boom"})
    assert tool.main(["status"]) == 1
    captured = capsys.readouterr()
    assert captured.out.splitlines()[-1].startswith("beta")
    assert captured.out.splitlines()[-1].endswith("?")
    assert captured.err == "ERROR: src/external_dependencies/beta boom\n"


# --- update -----------------------------------------------------------------

GIT_IDENTITY = {
    "GIT_AUTHOR_NAME": "test",
    "GIT_AUTHOR_EMAIL": "test@example.com",
    "GIT_COMMITTER_NAME": "test",
    "GIT_COMMITTER_EMAIL": "test@example.com",
}


def git(repository: Path, *arguments: str) -> str:
    import os
    import subprocess

    return subprocess.run(
        ["git", "-c", "commit.gpgsign=false", "-C", str(repository), *arguments],
        check=True,
        capture_output=True,
        text=True,
        env={**os.environ, **GIT_IDENTITY},
    ).stdout.strip()


def make_upstream(
    tmp_path: Path, *, second_first_line: str = "a", drop_docs: bool = False
) -> tuple[Path, str, str]:
    """Build an upstream repo with two commits; return (path, old sha, new sha)."""
    upstream = tmp_path / "upstream"
    upstream.mkdir()
    git(upstream, "init", "--quiet", "--initial-branch=main")
    git(upstream, "config", "uploadpack.allowFilter", "true")
    (upstream / "description").mkdir()
    (upstream / "description" / "model.txt").write_text("a\nb\nc\n")
    (upstream / "description" / "extra.txt").write_text("extra\n")
    (upstream / "description" / "two words.txt").write_text("spaced\n")
    (upstream / "LICENSE").write_text("MIT\n")
    (upstream / "unrelated.txt").write_text("not vendored\n")
    (upstream / "docs").mkdir()
    (upstream / "docs" / "guide.txt").write_text("guide\n")
    git(upstream, "add", "--all")
    git(upstream, "commit", "--quiet", "--message", "first")
    old = git(upstream, "rev-parse", "HEAD")
    if drop_docs:
        git(upstream, "rm", "--quiet", "-r", "docs")
    (upstream / "description" / "model.txt").write_text(
        f"{second_first_line}\nb\nc\nd\n"
    )
    (upstream / "description" / "extra.txt").write_text("extra changed\n")
    (upstream / "description" / "two words.txt").write_text("spaced changed\n")
    git(upstream, "add", "--all")
    git(upstream, "commit", "--quiet", "--message", "second")
    new = git(upstream, "rev-parse", "HEAD")
    return upstream, old, new


def make_workspace(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
    upstream: Path,
    old: str,
    *,
    pristine: bool = False,
    vendor_docs: bool = False,
    commit_suffix: str = "",
) -> Path:
    """Vendor the first upstream commit, by default with one edit and one prune."""
    root = tmp_path / "ws" / "src" / "external_dependencies" / "source"
    (root / "description").mkdir(parents=True)
    (root / "LICENSE").write_text("MIT\n")
    if pristine:
        (root / "description" / "model.txt").write_text("a\nb\nc\n")
        (root / "description" / "extra.txt").write_text("extra\n")
        (root / "description" / "two words.txt").write_text("spaced\n")
        modified = ""
    else:
        (root / "description" / "model.txt").write_text("A\nb\nc\n")
        modified = "modified_paths:\n  - description/model.txt\n"
    vendored = "vendored_paths:\n  - LICENSE\n  - description\n"
    if vendor_docs:
        (root / "docs").mkdir()
        (root / "docs" / "guide.txt").write_text("guide\n")
        vendored += "  - docs\n"
    manifest = root / "UPSTREAM.yaml"
    manifest.write_text(
        "upstream:\n"
        f"  repository: {upstream}\n"
        f"  commit: {old}{commit_suffix}\n"
        "  branch: main\n" + vendored + modified,
        encoding="utf-8",
    )
    monkeypatch.setattr(tool.validator, "REPOSITORY_ROOT", tmp_path / "ws")
    monkeypatch.setattr(
        tool.validator, "discover_vendoring_manifests", lambda: ([manifest], [])
    )
    monkeypatch.setattr(tool.validator, "validate_vendor_manifest", lambda _: [])
    return root


def test_update_reapplies_local_edits_and_bumps_pin(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, new = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "source"]) == 0
    assert (root / "description" / "model.txt").read_text() == "A\nb\nc\nd\n"
    assert not (root / "description" / "extra.txt").exists()
    # Pruned upstream-modified file with a space in its name: the modify/delete
    # resolution must see the whole path, not two fragments.
    assert not (root / "description" / "two words.txt").exists()
    assert not (root / "unrelated.txt").exists()
    assert f"  commit: {new}\n" in (root / "UPSTREAM.yaml").read_text()
    out = capsys.readouterr().out
    assert (
        f"Refreshed src/external_dependencies/source from {old[:9]} to {new[:9]} on main."
        in out
    )
    assert "Re-applied local modifications to 3 files." in out
    assert "ledger matches" in out


def test_update_to_explicit_commit_and_noop_when_current(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, new = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "source", "--to", old]) == 0
    assert (
        capsys.readouterr().out
        == f"src/external_dependencies/source already pins {old} on main.\n"
    )
    assert (root / "description" / "model.txt").read_text() == "A\nb\nc\n"
    assert tool.main(["update", "source", "--to", new]) == 0
    assert f"  commit: {new}\n" in (root / "UPSTREAM.yaml").read_text()


def test_update_leaves_conflict_markers_and_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, new = make_upstream(tmp_path, second_first_line="z")
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "source"]) == 1
    model = (root / "description" / "model.txt").read_text()
    assert "<<<<<<<" in model
    assert "A\n" in model
    assert "z\n" in model
    assert f"  commit: {new}\n" in (root / "UPSTREAM.yaml").read_text()
    out = capsys.readouterr().out
    assert "Conflicts while re-applying local modifications:" in out
    assert "  src/external_dependencies/source/description/model.txt" in out
    assert (
        "to discard: git restore --source=HEAD --staged --worktree -- "
        "src/external_dependencies/source && git clean -fd -- "
        "src/external_dependencies/source" in out
    )


def test_update_reports_stale_ledger(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """Upstream adopting a local edit makes modified_paths overstate the changes."""
    upstream, old, new = make_upstream(tmp_path, second_first_line="A")
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "source"]) == 1
    out = capsys.readouterr().out
    assert "ledger needs updating" in out
    assert "lists an unchanged upstream path as modified: description/model.txt" in out


def test_update_unknown_source_lists_known_ones(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "nope"]) == 1
    assert (
        capsys.readouterr().err
        == "ERROR: no vendored source named nope; known sources: source\n"
    )


def test_update_refuses_lfs_pointers(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    (root / "description" / "mesh.stl").write_bytes(
        b"version https://git-lfs.github.com/spec/v1\noid sha256:"
        + b"0" * 64
        + b"\nsize 12\n"
    )
    assert tool.main(["update", "source"]) == 1
    assert "git lfs pull" in capsys.readouterr().err


def test_update_pristine_copy_just_moves_the_pin(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, new = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old, pristine=True)
    assert tool.main(["update", "source"]) == 0
    assert (root / "description" / "model.txt").read_text() == "a\nb\nc\nd\n"
    assert (root / "description" / "extra.txt").read_text() == "extra changed\n"
    assert f"  commit: {new}\n" in (root / "UPSTREAM.yaml").read_text()
    out = capsys.readouterr().out
    assert "Re-applied" not in out
    assert "ledger matches" in out


def test_update_warns_when_upstream_dropped_a_vendored_path(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, new = make_upstream(tmp_path, drop_docs=True)
    root = make_workspace(tmp_path, monkeypatch, upstream, old, vendor_docs=True)
    assert tool.main(["update", "source"]) == 1
    out = capsys.readouterr().out
    assert (
        f"WARNING: upstream {new[:9]} no longer has docs; remove it from vendored_paths or restore it."
        in out
    )
    assert (root / "docs" / "guide.txt").exists()
    assert "omits a modified upstream path: docs/guide.txt" in out


def test_update_rejects_unresolvable_target(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "source", "--to", "deadbeef"]) == 1
    assert (
        capsys.readouterr().err
        == "ERROR: could not resolve deadbeef on upstream branch main\n"
    )
    assert f"  commit: {old}\n" in (root / "UPSTREAM.yaml").read_text()


def test_update_requires_an_exact_commit_line(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """A commit line the parser tolerates but the rewrite cannot match must fail up front."""
    upstream, old, _ = make_upstream(tmp_path)
    make_workspace(tmp_path, monkeypatch, upstream, old, commit_suffix=" ")
    assert tool.main(["update", "source"]) == 1
    assert "must contain exactly one" in capsys.readouterr().err


def test_update_reports_git_failures_and_timeouts(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    import subprocess

    upstream, old, _ = make_upstream(tmp_path)
    make_workspace(tmp_path, monkeypatch, upstream, old)
    command = [
        "git",
        "-c",
        "commit.gpgsign=false",
        "-C",
        "/tmp/x",
        "sparse-checkout",
        "set",
        "--",
        "/a",
    ]

    def fail(*_: object) -> int:
        raise subprocess.CalledProcessError(
            128, command, stderr="warning: x\nfatal: boom\n"
        )

    monkeypatch.setattr(tool, "refresh", fail)
    assert tool.main(["update", "source"]) == 1
    assert (
        capsys.readouterr().err
        == "ERROR: git sparse-checkout set failed: fatal: boom\n"
    )

    def hang(*_: object) -> int:
        raise subprocess.TimeoutExpired(command, 300)

    monkeypatch.setattr(tool, "refresh", hang)
    assert tool.main(["update", "source"]) == 1
    assert (
        capsys.readouterr().err
        == "ERROR: git sparse-checkout set timed out after 300 seconds\n"
    )


def test_update_rejects_option_like_target(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    make_workspace(tmp_path, monkeypatch, upstream, old)
    assert tool.main(["update", "source", "--to=--foo"]) == 1
    assert (
        capsys.readouterr().err
        == "ERROR: could not resolve --foo on upstream branch main\n"
    )


def test_update_refuses_to_leave_lfs_pointers_behind(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """An unsmudged checkout must not pass as a refresh; the validator cannot tell."""
    upstream, old, new = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    calls: list[int] = []

    def pointers_after_copy(_: Path) -> list[Path]:
        calls.append(1)
        return [] if len(calls) == 1 else [Path("description/mesh.stl")]

    monkeypatch.setattr(tool, "lfs_pointers", pointers_after_copy)
    assert tool.main(["update", "source"]) == 1
    err = capsys.readouterr().err
    assert "unsmudged Git LFS pointers (description/mesh.stl)" in err
    assert "git restore --source=HEAD" in err
    assert f"  commit: {new}\n" in (root / "UPSTREAM.yaml").read_text()


def test_update_reports_copy_failures_with_recovery(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    make_workspace(tmp_path, monkeypatch, upstream, old)

    def fail(*_: object) -> int:
        raise OSError("No space left on device")

    monkeypatch.setattr(tool, "refresh", fail)
    assert tool.main(["update", "source"]) == 1
    assert capsys.readouterr().err == (
        "ERROR: No space left on device. src/external_dependencies/source may be "
        "partially rewritten; to discard: git restore --source=HEAD --staged "
        "--worktree -- src/external_dependencies/source && git clean -fd -- "
        "src/external_dependencies/source\n"
    )


def test_update_reports_fetch_failure(
    tmp_path: Path, monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    manifest = root / "UPSTREAM.yaml"
    manifest.write_text(
        manifest.read_text().replace(str(upstream), str(tmp_path / "gone"))
    )
    assert tool.main(["update", "source"]) == 1
    err = capsys.readouterr().err
    assert err.startswith(
        f"ERROR: could not fetch upstream branch main from {tmp_path / 'gone'}: "
    )
    assert (root / "description" / "model.txt").read_text() == "A\nb\nc\n"


def test_resolve_manifest_accepts_paths(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    upstream, old, _ = make_upstream(tmp_path)
    root = make_workspace(tmp_path, monkeypatch, upstream, old)
    manifest = root / "UPSTREAM.yaml"
    assert tool.resolve_manifest("source") == manifest
    assert tool.resolve_manifest("src/external_dependencies/source") == manifest
    assert tool.resolve_manifest(str(root)) == manifest
    assert tool.resolve_manifest(str(manifest)) == manifest
