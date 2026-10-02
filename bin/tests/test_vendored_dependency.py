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
