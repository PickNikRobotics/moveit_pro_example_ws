"""Tests for the workspace dependency policy validator."""

import hashlib
import os
import importlib.util
from pathlib import Path
import subprocess
from urllib.request import Request

from pytest import CaptureFixture, MonkeyPatch, fixture, mark, raises

MODULE_PATH = Path(__file__).resolve().parents[1] / "validate_workspace_dependencies.py"
MODULE_SPEC = importlib.util.spec_from_file_location(
    "validate_workspace_dependencies", MODULE_PATH
)
assert MODULE_SPEC is not None
assert MODULE_SPEC.loader is not None
validator = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(validator)

VALID_MANIFEST = """\
upstream:
  repository: https://github.com/example/repository.git
  commit: 0123456789abcdef0123456789abcdef01234567
  branch: main
vendored_paths:
  - description
pruning_notes: []
notes:
  - Test fixture.
"""


def mock_valid_upstream_metadata(monkeypatch: MonkeyPatch) -> None:
    """Provide one bounded retained blob for tests focused on later Git phases."""
    monkeypatch.setattr(
        validator,
        "fetch_upstream_tree_metadata",
        lambda *_: (
            [{"path": "description/model.txt", "type": "blob", "size": 1}],
            None,
        ),
    )


def validate_manifest(
    tmp_path: Path, content: str, *, create_vendored_path: bool = True
) -> list[str]:
    """Validate a temporary vendoring manifest and return its errors."""
    if create_vendored_path:
        (tmp_path / "description").mkdir()
    manifest = tmp_path / "UPSTREAM.yaml"
    manifest.write_text(content, encoding="utf-8")
    return validator.validate_vendor_manifest(manifest)


def test_valid_manifest_passes(tmp_path: Path) -> None:
    """Accept a complete manifest whose retained path exists."""
    assert validate_manifest(tmp_path, VALID_MANIFEST) == []


def test_comment_only_manifest_fails(tmp_path: Path) -> None:
    """Reject a manifest containing no metadata."""
    errors = validate_manifest(
        tmp_path, "# repository:\n# commit:\n# vendored_paths:\n# pruning_notes:\n"
    )
    assert "must contain an upstream mapping" in errors[0]


def test_short_commit_fails(tmp_path: Path) -> None:
    """Reject an upstream revision that is not a full commit SHA."""
    errors = validate_manifest(
        tmp_path,
        VALID_MANIFEST.replace("0123456789abcdef0123456789abcdef01234567", "0123456"),
    )
    assert any("full lowercase Git commit SHA" in error for error in errors)


def test_missing_branch_fails(tmp_path: Path) -> None:
    """Reject a manifest without the upstream branch."""
    errors = validate_manifest(tmp_path, VALID_MANIFEST.replace("  branch: main\n", ""))
    assert any("identify the upstream branch" in error for error in errors)


def test_duplicate_upstream_key_fails(tmp_path: Path) -> None:
    """Reject duplicate keys that could conceal provenance metadata."""
    errors = validate_manifest(
        tmp_path,
        VALID_MANIFEST.replace(
            "  branch: main\n",
            "  commit: ffffffffffffffffffffffffffffffffffffffff\n  branch: main\n",
        ),
    )
    assert "duplicate upstream key: commit" in errors[0]


def test_unknown_top_level_key_fails(tmp_path: Path) -> None:
    """Reject top-level keys outside the manifest schema."""
    errors = validate_manifest(tmp_path, VALID_MANIFEST + "unexpected: []\n")
    assert "unknown top-level key: unexpected" in errors[0]


def test_non_https_repository_fails(tmp_path: Path) -> None:
    """Reject repository URLs that are not immutable HTTPS sources."""
    errors = validate_manifest(
        tmp_path,
        VALID_MANIFEST.replace(
            "https://github.com/example/repository.git",
            "git@github.com:example/repository.git",
        ),
    )
    assert any("repository URL" in error for error in errors)


@mark.parametrize(
    "repository",
    [
        "https://user:password@github.com/example/repository.git",
        "https://github.com/example/repository.git?access_token=secret",
        "https://github.com/example/repository.git#fragment",
        "https://github.com:8443/example/repository.git",
        "https://localhost/example/repository.git",
        "https://example.com/example/repository.git",
        "https://github.com:invalid/example/repository.git",
    ],
)
def test_unsafe_repository_url_fails(tmp_path: Path, repository: str) -> None:
    """Reject repository URLs that could redirect or disclose CI network traffic."""
    errors = validate_manifest(
        tmp_path,
        VALID_MANIFEST.replace("https://github.com/example/repository.git", repository),
    )
    assert any("repository URL" in error for error in errors)


@mark.parametrize(
    "branch",
    [
        "main\nforged",
        "+refs/heads/main",
        "main:other",
        "main..other",
        "main@{upstream}",
    ],
)
def test_unsafe_upstream_branch_fails(branch: str) -> None:
    """Reject branch names that can alter refspecs or forge diagnostics."""
    errors = validator.validate_upstream(
        Path("UPSTREAM.yaml"),
        {
            "repository": "https://github.com/example/repository.git",
            "commit": "0123456789abcdef0123456789abcdef01234567",
            "branch": branch,
        },
    )
    assert any("identify the upstream branch" in error for error in errors)


def test_empty_vendored_paths_fails(tmp_path: Path) -> None:
    """Reject provenance metadata that retains no source paths."""
    errors = validate_manifest(
        tmp_path,
        VALID_MANIFEST.replace(
            "vendored_paths:\n  - description", "vendored_paths: []"
        ),
    )
    assert any("at least one vendored path" in error for error in errors)


def test_missing_vendored_path_fails(tmp_path: Path) -> None:
    """Reject a retained path that is absent from the snapshot."""
    errors = validate_manifest(tmp_path, VALID_MANIFEST, create_vendored_path=False)
    assert any("missing vendored path" in error for error in errors)


def test_vendored_path_outside_source_root_fails(tmp_path: Path) -> None:
    """Reject a retained path that escapes the vendored source root."""
    errors = validate_manifest(
        tmp_path, VALID_MANIFEST.replace("  - description", "  - ../description")
    )
    assert any(
        "vendored_paths contains a non-normalized path: ../description" in error
        for error in errors
    )


def test_source_root_as_vendored_path_fails(tmp_path: Path) -> None:
    """Reject the source root itself as an imprecise retained path."""
    errors = validate_manifest(
        tmp_path, VALID_MANIFEST.replace("  - description", "  - .")
    )
    assert any("concrete vendored paths" in error for error in errors)


def test_vendored_path_outside_snapshot_fails_offline(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject snapshot boundary violations before upstream network validation."""
    (tmp_path / "description").mkdir()
    (tmp_path / "snapshot").mkdir()
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "vendored_paths:\n", "snapshot_path: snapshot\nvendored_paths:\n"
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    assert validator.validate_vendor_manifest(manifest_path) == [
        "UPSTREAM.yaml vendored path is outside snapshot_path: description"
    ]


def test_non_normalized_snapshot_path_fails_offline(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a snapshot boundary containing traversal before network validation."""
    (tmp_path / "snapshot" / "description").mkdir(parents=True)
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "vendored_paths:\n  - description",
            "snapshot_path: snapshot/../snapshot\n"
            "vendored_paths:\n  - snapshot/description",
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    assert any(
        "snapshot_path must be a normalized relative path" in error
        for error in validator.validate_vendor_manifest(manifest_path)
    )


@mark.parametrize(
    ("value", "allow_root", "expected"),
    [
        ("description", False, True),
        ("snapshot/description", False, True),
        (".", True, True),
        (".", False, False),
        ("/outside", False, False),
        ("C:/outside", False, False),
        ("C:outside", False, False),
        ("snapshot\\description", False, False),
        ("snapshot/*", False, False),
        ("snapshot/?", False, False),
        ("snapshot/[abc]", False, False),
        ("snapshot/]", False, False),
        ("snapshot//description", False, False),
        ("snapshot/./description", False, False),
        ("snapshot/../description", False, False),
    ],
)
def test_manifest_path_normalization_is_portable(
    value: str, allow_root: bool, expected: bool
) -> None:
    """Apply identical manifest path semantics on POSIX and Windows hosts."""
    assert (
        validator.is_normalized_manifest_path(value, allow_root=allow_root) is expected
    )


@mark.parametrize(
    ("metadata", "key"),
    [
        ("vendored_paths:\n  - snapshot/../other", "vendored_paths"),
        (
            "vendored_paths:\n  - snapshot/description\n"
            "modified_paths:\n  - snapshot/../other",
            "modified_paths",
        ),
        (
            "vendored_paths:\n  - snapshot/description\n"
            "apache_paths:\n  - snapshot/../other",
            "apache_paths",
        ),
        (
            "vendored_paths:\n  - snapshot/description\n"
            "apache_paths:\n  - snapshot/description\n"
            "apache_excluded_paths:\n  - snapshot/../other",
            "apache_excluded_paths",
        ),
    ],
)
def test_non_normalized_retained_path_fails_offline(
    tmp_path: Path, monkeypatch: MonkeyPatch, metadata: str, key: str
) -> None:
    """Reject traversal in every retained-path ledger before network validation."""
    (tmp_path / "snapshot" / "description").mkdir(parents=True)
    (tmp_path / "other").mkdir()
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "vendored_paths:\n  - description",
            f"snapshot_path: snapshot\n{metadata}",
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    assert any(
        f"{key} contains a non-normalized path: snapshot/../other" in error
        for error in validator.validate_vendor_manifest(manifest_path)
    )


def test_non_normalized_manifest_skips_upstream_fetch(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Stop before network validation when offline path validation fails."""
    (tmp_path / "snapshot").mkdir()
    (tmp_path / "other").mkdir()
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "vendored_paths:\n  - description",
            "snapshot_path: snapshot\n" "vendored_paths:\n  - snapshot/../other",
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    def unexpected_fetch(*_: object) -> list[str]:
        raise AssertionError(
            "upstream fetch must not run after offline validation fails"
        )

    monkeypatch.setattr(validator, "fetch_and_validate_upstream", unexpected_fetch)

    errors = validator.validate_vendored_roots([manifest_path], verify_upstream=True)
    assert any("contains a non-normalized path" in error for error in errors)


@mark.parametrize("unsafe_component", ["*", "?", "[abc]", "]"])
def test_sparse_pattern_metacharacter_skips_upstream_fetch(
    tmp_path: Path, monkeypatch: MonkeyPatch, unsafe_component: str
) -> None:
    """Reject sparse-pattern syntax before metadata or Git network operations."""
    (tmp_path / "snapshot" / unsafe_component).mkdir(parents=True)
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "vendored_paths:\n  - description",
            "snapshot_path: snapshot\n"
            f"vendored_paths:\n  - snapshot/{unsafe_component}",
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    fetch_called = False

    def track_fetch(_: Path, __: object) -> list[str]:
        nonlocal fetch_called
        fetch_called = True
        return []

    monkeypatch.setattr(validator, "fetch_and_validate_upstream", track_fetch)

    errors = validator.validate_vendored_roots([manifest_path], verify_upstream=True)
    assert (
        not fetch_called
    ), "upstream fetch must not run after offline validation fails"
    assert errors == [
        "UPSTREAM.yaml vendored_paths contains a non-normalized path: "
        f"snapshot/{unsafe_component}"
    ]


def test_retained_apache_material_requires_declared_boundary(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Require path-level Apache metadata for mixed-license retained snapshots."""
    (tmp_path / "description").mkdir()
    (tmp_path / "description" / "LICENSE.txt").write_text(
        "Apache License\n", encoding="utf-8"
    )
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    assert validator.validate_vendor_manifest(manifest_path) == [
        "UPSTREAM.yaml retains Apache-licensed material but does not declare apache_paths"
    ]


def test_retained_license_symlink_fails_without_following_target(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a license symlink rather than dereferencing an external target."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    external_license = tmp_path / "external-license"
    external_license.write_text("Apache License\n", encoding="utf-8")
    (source_root / "description" / "LICENSE.txt").symlink_to(external_license)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    assert validator.validate_vendor_manifest(manifest_path) == [
        "source/UPSTREAM.yaml retained license file must not be a symlink: "
        "description/LICENSE.txt"
    ]


def test_apache_license_does_not_hide_later_license_symlink(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Inspect every retained license entry after detecting Apache content."""
    source_root = tmp_path / "source"
    description = source_root / "description"
    description.mkdir(parents=True)
    (description / "LICENSE-A").write_text("Apache License\n", encoding="utf-8")
    target = description / "license-target"
    target.write_text("conditional terms\n", encoding="utf-8")
    (description / "LICENSE-Z").symlink_to(target)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "notes:\n",
            "modified_paths:\n"
            + modified_entry(
                "description/LICENSE-Z",
                f"symlink:{os.readlink(description / 'LICENSE-Z')}".encode(),
            ).rstrip("\n")
            + "\n"
            "apache_paths:\n"
            "  - description\n"
            "apache_excluded_paths:\n"
            "  - description/LICENSE-Z\n"
            "notes:\n",
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    expected_errors = [
        "source/UPSTREAM.yaml retained license file must not be a symlink: "
        "description/LICENSE-Z"
    ]
    assert validator.validate_vendor_manifest(manifest_path) == expected_errors

    upstream = tmp_path / "upstream"
    upstream.mkdir()
    assert validator.validate_upstream_snapshot(manifest_path, upstream) == (
        expected_errors
    )


def test_absolute_retained_symlink_fails_offline(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject an absolute retained symlink before any upstream network access."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    (source_root / "description" / "escape").symlink_to("/etc/passwd")
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    assert validator.validate_vendor_manifest(manifest_path) == [
        "source/UPSTREAM.yaml retained symlink has an unsafe target: "
        "description/escape -> /etc/passwd"
    ]


def test_cyclic_declared_vendored_path_fails_offline(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a cycle when vendored_paths names the symlink itself."""
    source_root = tmp_path / "source"
    source_root.mkdir()
    (source_root / "description").symlink_to("description")
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    assert validator.validate_vendor_manifest(manifest_path) == [
        "source/UPSTREAM.yaml vendored path could not be resolved: description"
    ]


@mark.parametrize(
    ("target", "expected_detail"),
    [
        ("../../outside", "has an unsafe target"),
        ("missing.txt", "target is missing"),
        ("directory", "target is not a file"),
        ("link", "target could not be resolved"),
    ],
)
def test_unsafe_retained_symlink_fails_offline(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
    target: str,
    expected_detail: str,
) -> None:
    """Reject traversal, broken, and directory links in retained content."""
    source_root = tmp_path / "source"
    (source_root / "description" / "directory").mkdir(parents=True)
    (source_root / "description" / "link").symlink_to(target)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    errors = validator.validate_vendor_manifest(manifest_path)

    assert len(errors) == 1
    assert expected_detail in errors[0]
    assert f"description/link -> {target}" in errors[0]


def test_confined_retained_file_symlink_passes_offline(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Accept a normalized retained symlink to a file within the source root."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    (source_root / "description" / "model.txt").write_text("model", encoding="utf-8")
    (source_root / "description" / "alias").symlink_to("model.txt")
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    assert validator.validate_vendor_manifest(manifest_path) == []


def test_escaping_upstream_symlink_fails_comparison(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject an escaping symlink materialized from a pinned upstream tree."""
    candidate = tmp_path / "candidate"
    upstream = tmp_path / "upstream"
    (candidate / "description").mkdir(parents=True)
    (upstream / "description").mkdir(parents=True)
    (candidate / "description" / "model.txt").write_text("model", encoding="utf-8")
    (upstream / "description" / "escape").symlink_to("/etc/passwd")
    manifest_path = candidate / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    assert validator.validate_upstream_snapshot(manifest_path, upstream) == [
        "candidate/UPSTREAM.yaml retained symlink has an unsafe target: "
        "description/escape -> /etc/passwd"
    ]


def test_malformed_indentation_fails(tmp_path: Path) -> None:
    """Reject malformed YAML indentation instead of misparsing metadata."""
    errors = validate_manifest(
        tmp_path, VALID_MANIFEST.replace("  commit:", "   commit:")
    )
    assert "unsupported indentation" in errors[0]


def test_missing_notes_fails(tmp_path: Path) -> None:
    """Reject provenance metadata without explanatory notes."""
    errors = validate_manifest(
        tmp_path, VALID_MANIFEST.replace("notes:\n  - Test fixture.\n", "")
    )
    assert any("must contain a notes list" in error for error in errors)


def test_empty_notes_fails(tmp_path: Path) -> None:
    """Reject an empty provenance-notes list."""
    errors = validate_manifest(
        tmp_path, VALID_MANIFEST.replace("notes:\n  - Test fixture.", "notes: []")
    )
    assert any("at least one provenance note" in error for error in errors)


def test_missing_modified_path_fails(tmp_path: Path) -> None:
    """Reject a modification ledger that references an absent path."""
    manifest = VALID_MANIFEST.replace(
        "notes:\n",
        "modified_paths:\n  - missing_file.txt sha256:" + "0" * 64 + "\nnotes:\n",
    )
    errors = validate_manifest(tmp_path, manifest)
    assert any("missing modified path" in error for error in errors)


def modified_entry(declared_path: str, _content: bytes = b"") -> str:
    """Render a modified_paths entry."""
    return f"  - {declared_path}\n"


def upstream_comparison_errors(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
    *,
    candidate_content: bytes,
    upstream_content: bytes,
    modified: bool = False,
    outside_content: bytes | None = None,
    apache_license: bool = False,
    apache_excluded: bool = False,
    lfs_tracked: bool = False,
) -> list[str]:
    """Compare a temporary candidate manifest with an upstream snapshot."""
    if lfs_tracked:
        (tmp_path / ".gitattributes").write_text(
            "*.txt filter=lfs diff=lfs merge=lfs -text\n", encoding="utf-8"
        )
    candidate = tmp_path / "candidate"
    upstream = tmp_path / "upstream"
    (candidate / "description").mkdir(parents=True)
    (upstream / "description").mkdir(parents=True)
    (candidate / "description" / "model.txt").write_bytes(candidate_content)
    (upstream / "description" / "model.txt").write_bytes(upstream_content)
    if outside_content is not None:
        (candidate / "outside.txt").write_bytes(outside_content)
    manifest = VALID_MANIFEST
    if apache_license:
        (candidate / "description" / "LICENSE").write_text(
            "Apache License\n", encoding="utf-8"
        )
        (upstream / "LICENSE").write_text("Apache License\n", encoding="utf-8")
        (upstream / "model.txt").write_bytes(upstream_content)
        (upstream / "description" / "model.txt").unlink()
        manifest = manifest.replace(
            "vendored_paths:\n", "snapshot_path: description\nvendored_paths:\n"
        )
        manifest = manifest.replace(
            "notes:\n", "apache_paths:\n  - description\nnotes:\n"
        )
        if apache_excluded:
            manifest = manifest.replace(
                "notes:\n",
                "apache_excluded_paths:\n  - description/model.txt\nnotes:\n",
            )
    if modified:
        manifest = manifest.replace(
            "notes:\n",
            "modified_paths:\n"
            + modified_entry("description/model.txt", candidate_content)
            + "notes:\n",
        )
    manifest_path = candidate / "UPSTREAM.yaml"
    manifest_path.write_text(manifest, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    return validator.validate_upstream_snapshot(manifest_path, upstream)


def test_undeclared_upstream_modification_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a retained file that differs without a modification-ledger entry."""
    errors = upstream_comparison_errors(
        tmp_path,
        monkeypatch,
        candidate_content=b"changed",
        upstream_content=b"original",
    )
    assert any(
        "omits a modified upstream path: description/model.txt" in error
        for error in errors
    )


def test_stale_modified_path_fails(tmp_path: Path, monkeypatch: MonkeyPatch) -> None:
    """Reject a modification-ledger entry whose file matches upstream."""
    errors = upstream_comparison_errors(
        tmp_path,
        monkeypatch,
        candidate_content=b"same",
        upstream_content=b"same",
        modified=True,
    )
    assert any(
        "lists an unchanged upstream path as modified" in error for error in errors
    )


def test_modified_apache_path_without_change_notice_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Require a PickNik change notice on modified upstream Apache files."""
    errors = upstream_comparison_errors(
        tmp_path,
        monkeypatch,
        candidate_content=b"changed",
        upstream_content=b"original",
        modified=True,
        apache_license=True,
    )
    assert errors == [
        "candidate/UPSTREAM.yaml modified Apache-licensed path lacks a PickNik "
        "change notice: description/model.txt"
    ]


def test_apache_snapshot_rejects_unclassified_modified_path(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Require every modification in an Apache-bearing snapshot to be classified."""
    candidate = tmp_path / "candidate"
    upstream = tmp_path / "upstream"
    (candidate / "description").mkdir(parents=True)
    upstream.mkdir()
    (candidate / "description" / "LICENSE").write_text(
        "Apache License\n", encoding="utf-8"
    )
    (candidate / "description" / "model.txt").write_text("changed", encoding="utf-8")
    (candidate / "description" / "harmless.txt").write_text("same", encoding="utf-8")
    (upstream / "LICENSE").write_text("Apache License\n", encoding="utf-8")
    (upstream / "model.txt").write_text("original", encoding="utf-8")
    (upstream / "harmless.txt").write_text("same", encoding="utf-8")
    manifest = VALID_MANIFEST.replace(
        "vendored_paths:\n", "snapshot_path: description\nvendored_paths:\n"
    ).replace(
        "notes:\n",
        "modified_paths:\n"
        + modified_entry("description/model.txt", b"changed")
        + "apache_paths:\n"
        "  - description/harmless.txt\n"
        "notes:\n",
    )
    manifest_path = candidate / "UPSTREAM.yaml"
    manifest_path.write_text(manifest, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    expected_error = (
        "candidate/UPSTREAM.yaml modified path is not classified by apache_paths "
        "or apache_excluded_paths: description/model.txt"
    )

    assert validator.validate_vendor_manifest(manifest_path) == [expected_error]
    assert validator.validate_upstream_snapshot(manifest_path, upstream) == [
        expected_error
    ]


def test_modified_apache_path_with_change_notice_passes(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Accept a modified upstream Apache file carrying the required notice."""
    assert (
        upstream_comparison_errors(
            tmp_path,
            monkeypatch,
            candidate_content=b"# Modified by PickNik Inc., 2026.\nchanged",
            upstream_content=b"original",
            modified=True,
            apache_license=True,
        )
        == []
    )


def test_modified_path_excluded_from_apache_boundary_passes(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Do not apply Apache notices to a documented mixed-license exception."""
    assert (
        upstream_comparison_errors(
            tmp_path,
            monkeypatch,
            candidate_content=b"changed",
            upstream_content=b"original",
            modified=True,
            apache_license=True,
            apache_excluded=True,
        )
        == []
    )


def test_retained_file_outside_vendored_paths_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a retained file outside the manifest's vendored boundaries."""
    errors = upstream_comparison_errors(
        tmp_path,
        monkeypatch,
        candidate_content=b"same",
        upstream_content=b"same",
        outside_content=b"undeclared",
    )
    assert any(
        "retains a file outside vendored_paths: outside.txt" in error
        for error in errors
    )


def test_lfs_pointer_matches_upstream_binary(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Treat an LFS pointer as the digest of its upstream binary content."""
    upstream_content = b"binary content"
    object_digest = hashlib.sha256(upstream_content).hexdigest()
    lfs_pointer = (
        "version https://git-lfs.github.com/spec/v1\n"
        f"oid sha256:{object_digest}\n"
        f"size {len(upstream_content)}\n"
    ).encode()
    assert (
        upstream_comparison_errors(
            tmp_path,
            monkeypatch,
            candidate_content=lfs_pointer,
            upstream_content=upstream_content,
            lfs_tracked=True,
        )
        == []
    )


def test_untracked_lfs_pointer_shape_is_not_trusted(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject pointer-shaped bytes at a path .gitattributes does not track.

    Git smudges a real LFS file to its content before the validator runs. So
    pointer text at an untracked path is an ordinary file shaped like a pointer,
    and trusting its embedded oid would let hand-authored text stand in for
    upstream content it never matched.
    """
    upstream_content = b"binary content"
    forged_pointer = (
        "version https://git-lfs.github.com/spec/v1\n"
        f"oid sha256:{hashlib.sha256(upstream_content).hexdigest()}\n"
        f"size {len(upstream_content)}\n"
    ).encode()
    errors = upstream_comparison_errors(
        tmp_path,
        monkeypatch,
        candidate_content=forged_pointer,
        upstream_content=upstream_content,
        lfs_tracked=False,
    )
    assert any("omits a modified upstream path" in error for error in errors)


def test_lfs_pointer_size_must_match_upstream_binary(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject an LFS pointer whose declared size does not match pinned content."""
    upstream_content = b"binary content"
    object_digest = hashlib.sha256(upstream_content).hexdigest()
    lfs_pointer = (
        "version https://git-lfs.github.com/spec/v1\n"
        f"oid sha256:{object_digest}\n"
        f"size {len(upstream_content) + 1}\n"
    ).encode()

    errors = upstream_comparison_errors(
        tmp_path,
        monkeypatch,
        candidate_content=lfs_pointer,
        upstream_content=upstream_content,
        lfs_tracked=True,
    )

    assert errors == [
        "candidate/UPSTREAM.yaml omits a modified upstream path: description/model.txt"
    ]


def test_dependency_policy_ci_fetches_and_verifies_lfs_objects() -> None:
    """Require provenance CI to materialize and verify every retained LFS object."""
    workflow = (validator.REPOSITORY_ROOT / ".github/workflows/ci.yaml").read_text(
        encoding="utf-8"
    )
    dependency_job = workflow.split("  validate-workspace-dependencies:", 1)[1].split(
        "\n  validate_objectives:", 1
    )[0]

    assert "lfs: true" in dependency_job
    assert "git lfs fsck --objects" in dependency_job


class FakeMetadataResponse:
    """Minimal context-managed response for metadata fetch tests."""

    def __init__(self, content: bytes) -> None:
        self.content = content

    def __enter__(self) -> "FakeMetadataResponse":
        return self

    def __exit__(self, *_: object) -> None:
        return None

    def read(self, size: int) -> bytes:
        return self.content[:size]


def test_fetch_upstream_tree_metadata_uses_bounded_authenticated_request(
    monkeypatch: MonkeyPatch,
) -> None:
    """Use the pinned GitHub tree endpoint with bounded authenticated reads."""
    captured_requests: list[tuple[Request, int]] = []
    payload = (
        b'{"truncated": false, "tree": '
        b'[{"path": "description/model", "type": "blob", "size": 3}]}'
    )

    def fake_urlopen(request: Request, timeout: int) -> FakeMetadataResponse:
        captured_requests.append((request, timeout))
        return FakeMetadataResponse(payload)

    monkeypatch.setenv("GITHUB_TOKEN", "test-token")
    monkeypatch.setattr(validator, "urlopen", fake_urlopen)
    entries, error = validator.fetch_upstream_tree_metadata(
        "https://github.com/example/repository.git",
        "0123456789abcdef0123456789abcdef01234567",
    )
    assert error is None
    assert entries == [{"path": "description/model", "type": "blob", "size": 3}]
    request, timeout = captured_requests[0]
    assert request.full_url == (
        "https://api.github.com/repos/example/repository/git/trees/"
        "0123456789abcdef0123456789abcdef01234567?recursive=1"
    )
    assert request.get_header("Authorization") == "Bearer test-token"
    assert timeout == validator.UPSTREAM_METADATA_TIMEOUT_SECONDS


def test_fetch_upstream_tree_metadata_rejects_oversized_response(
    monkeypatch: MonkeyPatch,
) -> None:
    """Reject metadata responses larger than the parser's explicit byte cap."""
    monkeypatch.setattr(validator, "UPSTREAM_MAX_METADATA_BYTES", 4)
    monkeypatch.setattr(
        validator, "urlopen", lambda *_args, **_kwargs: FakeMetadataResponse(b"12345")
    )
    assert validator.fetch_upstream_tree_metadata(
        "https://github.com/example/repository.git",
        "0123456789abcdef0123456789abcdef01234567",
    ) == ([], "pinned tree metadata exceeds the response limit")


def test_aggregate_metadata_budget_stops_later_network_requests(
    monkeypatch: MonkeyPatch,
) -> None:
    """Share metadata bytes across manifests and stop requests after exhaustion."""
    payload = b'{"truncated": false, "tree": []}'
    requests: list[Request] = []

    def fake_urlopen(request: Request, **_: object) -> FakeMetadataResponse:
        requests.append(request)
        return FakeMetadataResponse(payload)

    monkeypatch.setattr(validator, "urlopen", fake_urlopen)
    monkeypatch.setattr(
        validator, "UPSTREAM_MAX_TOTAL_METADATA_BYTES", len(payload) * 2 - 1
    )
    budget = validator.UpstreamValidationBudget()
    repository = "https://github.com/example/repository.git"
    commit = "0123456789abcdef0123456789abcdef01234567"

    assert validator.fetch_upstream_tree_metadata(repository, commit, budget) == (
        [],
        None,
    )
    assert validator.fetch_upstream_tree_metadata(repository, commit, budget) == (
        [],
        f"upstream verification exceeds {len(payload) * 2 - 1} metadata bytes",
    )
    assert validator.fetch_upstream_tree_metadata(repository, commit, budget) == (
        [],
        f"upstream verification exceeds {len(payload) * 2 - 1} metadata bytes",
    )
    assert len(requests) == 2


def test_expired_aggregate_deadline_stops_before_metadata_request(
    monkeypatch: MonkeyPatch,
) -> None:
    """Stop before network access when the shared verification deadline expires."""
    budget = validator.UpstreamValidationBudget()
    budget.deadline = 0.0

    def unexpected_urlopen(*_: object, **__: object) -> FakeMetadataResponse:
        raise AssertionError("metadata request must not run after deadline exhaustion")

    monkeypatch.setattr(validator, "urlopen", unexpected_urlopen)

    assert validator.fetch_upstream_tree_metadata(
        "https://github.com/example/repository.git",
        "0123456789abcdef0123456789abcdef01234567",
        budget,
    ) == (
        [],
        f"upstream verification exceeds {validator.UPSTREAM_MAX_TOTAL_SECONDS} seconds",
    )


@mark.parametrize(
    ("payload", "expected_error"),
    [
        (b"not-json", "pinned tree metadata is malformed"),
        (
            b'{"truncated": true, "tree": []}',
            "pinned tree metadata is truncated or malformed",
        ),
        (b'{"truncated": false, "tree": {}}', "pinned tree metadata is malformed"),
    ],
)
def test_fetch_upstream_tree_metadata_rejects_unusable_payload(
    monkeypatch: MonkeyPatch, payload: bytes, expected_error: str
) -> None:
    """Fail closed when GitHub tree metadata is incomplete or malformed."""
    monkeypatch.setattr(
        validator,
        "urlopen",
        lambda *_args, **_kwargs: FakeMetadataResponse(payload),
    )
    assert validator.fetch_upstream_tree_metadata(
        "https://github.com/example/repository.git",
        "0123456789abcdef0123456789abcdef01234567",
    ) == ([], expected_error)


@mark.parametrize(
    "metadata_error",
    [
        "could not retrieve pinned tree metadata",
        "pinned tree metadata is malformed",
        "pinned tree metadata is truncated or malformed",
        "pinned tree metadata exceeds the response limit",
    ],
)
def test_metadata_error_stops_before_any_git_command(
    tmp_path: Path, monkeypatch: MonkeyPatch, metadata_error: str
) -> None:
    """Return every metadata error before initializing or fetching with Git."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(
        validator,
        "fetch_upstream_tree_metadata",
        lambda *_: ([], metadata_error),
    )
    commands: list[list[str | Path]] = []

    def record_command(
        arguments: list[str | Path], **_: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", record_command)
    assert validator.fetch_and_validate_upstream(manifest_path) == [
        f"source/UPSTREAM.yaml {metadata_error}"
    ]
    assert commands == []


def test_oversized_tree_is_rejected_before_any_git_command(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject oversized pinned content before Git fetch or checkout materialization."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "UPSTREAM_MAX_SNAPSHOT_BYTES", 1)
    monkeypatch.setattr(
        validator,
        "fetch_upstream_tree_metadata",
        lambda *_: (
            [{"path": "description/model.bin", "type": "blob", "size": 2}],
            None,
        ),
    )
    commands: list[list[str | Path]] = []

    def record_command(
        arguments: list[str | Path], **_: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", record_command)
    assert validator.fetch_and_validate_upstream(manifest_path) == [
        "source/UPSTREAM.yaml upstream snapshot exceeds 1 bytes"
    ]
    assert commands == []


def test_excessive_tree_file_count_is_rejected_before_any_git_command(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject excessive retained file counts before starting a Git subprocess."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "UPSTREAM_MAX_SNAPSHOT_FILES", 1)
    monkeypatch.setattr(
        validator,
        "fetch_upstream_tree_metadata",
        lambda *_: (
            [
                {"path": "unrelated/large.bin", "type": "blob", "size": 10**12},
                {"path": "description/first", "type": "blob", "size": 1},
                {"path": "description/second", "type": "blob", "size": 1},
            ],
            None,
        ),
    )
    commands: list[list[str | Path]] = []

    def record_command(
        arguments: list[str | Path], **_: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", record_command)
    assert validator.fetch_and_validate_upstream(manifest_path) == [
        "source/UPSTREAM.yaml upstream snapshot exceeds 1 files"
    ]
    assert commands == []


@mark.parametrize(
    ("aggregate_name", "aggregate_limit", "first_size", "second_size", "unit"),
    [
        ("UPSTREAM_MAX_TOTAL_SNAPSHOT_FILES", 1, 0, 0, "retained files"),
        ("UPSTREAM_MAX_TOTAL_SNAPSHOT_BYTES", 3, 2, 2, "retained bytes"),
    ],
)
def test_tree_limits_share_aggregate_budget_across_manifests(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
    aggregate_name: str,
    aggregate_limit: int,
    first_size: int,
    second_size: int,
    unit: str,
) -> None:
    """Reject cumulative retained files and bytes across dependency manifests."""
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, aggregate_name, aggregate_limit)
    budget = validator.UpstreamValidationBudget()
    manifests: list[Path] = []
    for name in ("first", "second"):
        source_root = tmp_path / name
        (source_root / "description").mkdir(parents=True)
        manifest_path = source_root / "UPSTREAM.yaml"
        manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
        manifests.append(manifest_path)

    assert (
        validator.validate_upstream_tree_limits(
            manifests[0],
            [{"path": "description/model", "type": "blob", "size": first_size}],
            budget,
        )
        == []
    )
    assert validator.validate_upstream_tree_limits(
        manifests[1],
        [{"path": "description/model", "type": "blob", "size": second_size}],
        budget,
    ) == [
        f"second/UPSTREAM.yaml upstream verification exceeds {aggregate_limit} {unit}"
    ]


def test_upstream_fetch_timeout_fails(tmp_path: Path, monkeypatch: MonkeyPatch) -> None:
    """Fail closed when an upstream branch fetch does not complete."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)

    def timeout_on_fetch(
        arguments: list[str | Path], **kwargs: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        if "fetch" in arguments:
            assert kwargs["timeout"] == validator.UPSTREAM_FETCH_TIMEOUT_SECONDS
            raise subprocess.TimeoutExpired(
                arguments, validator.UPSTREAM_FETCH_TIMEOUT_SECONDS
            )
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", timeout_on_fetch)
    errors = validator.fetch_and_validate_upstream(manifest_path)
    assert errors == [
        "source/UPSTREAM.yaml could not fetch upstream branch main: unknown error"
    ]


def test_upstream_fetch_failure_stops_before_checkout(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Fail closed without inspecting a partial checkout after fetch failure."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)
    commands: list[list[str | Path]] = []
    comparisons: list[tuple[Path, Path]] = []

    def fail_fetch(
        arguments: list[str | Path], **kwargs: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        if "fetch" in arguments:
            assert kwargs["timeout"] == validator.UPSTREAM_FETCH_TIMEOUT_SECONDS
            raise subprocess.CalledProcessError(
                128, arguments, stderr="fatal: upstream unavailable"
            )
        return subprocess.CompletedProcess(arguments, 0)

    def record_comparison(manifest: Path, checkout: Path) -> list[str]:
        comparisons.append((manifest, checkout))
        return []

    monkeypatch.setattr(validator.subprocess, "run", fail_fetch)
    monkeypatch.setattr(validator, "validate_upstream_snapshot", record_comparison)
    errors = validator.fetch_and_validate_upstream(manifest_path)
    assert errors == [
        "source/UPSTREAM.yaml could not fetch upstream branch main: "
        "fatal: upstream unavailable"
    ]
    assert not any("checkout" in command for command in commands)
    assert comparisons == []


def test_commit_outside_branch_stops_before_checkout(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Fail closed before comparison when branch ancestry is not established."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)
    commands: list[list[str | Path]] = []
    comparisons: list[tuple[Path, Path]] = []

    def reject_ancestry(
        arguments: list[str | Path], **kwargs: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        if "fetch" in arguments:
            assert kwargs["timeout"] == validator.UPSTREAM_FETCH_TIMEOUT_SECONDS
        return subprocess.CompletedProcess(
            arguments, 1 if "merge-base" in arguments else 0
        )

    def record_comparison(manifest: Path, checkout: Path) -> list[str]:
        comparisons.append((manifest, checkout))
        return []

    monkeypatch.setattr(validator.subprocess, "run", reject_ancestry)
    monkeypatch.setattr(validator, "validate_upstream_snapshot", record_comparison)
    errors = validator.fetch_and_validate_upstream(manifest_path)
    assert errors == [
        "source/UPSTREAM.yaml pins commit "
        "0123456789abcdef0123456789abcdef01234567 outside upstream branch main"
    ]
    assert not any("checkout" in command for command in commands)
    assert comparisons == []


def test_upstream_ancestry_operational_failure_is_reported(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Distinguish a Git ancestry error from a valid not-ancestor result."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)

    def fail_ancestry(
        arguments: list[str | Path], **_: object
    ) -> subprocess.CompletedProcess:
        if "merge-base" in arguments:
            return subprocess.CompletedProcess(
                arguments, 128, stderr="fatal: corrupt repository\n"
            )
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", fail_ancestry)

    assert validator.fetch_and_validate_upstream(manifest_path) == [
        "source/UPSTREAM.yaml could not verify upstream branch ancestry: "
        "fatal: corrupt repository"
    ]


def test_external_git_phases_are_sparse_and_bounded(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Use blob filtering, sparse checkout, and timeouts for every external Git phase."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)
    commands: list[list[str | Path]] = []

    def record_command(
        arguments: list[str | Path], **kwargs: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        if "init" in arguments:
            assert kwargs["timeout"] == validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
        if any(
            phase in arguments
            for phase in ("fetch", "merge-base", "sparse-checkout", "checkout")
        ):
            expected_timeout = (
                validator.UPSTREAM_FETCH_TIMEOUT_SECONDS
                if "fetch" in arguments
                else validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
            )
            assert kwargs["timeout"] == expected_timeout
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", record_command)
    monkeypatch.setattr(validator, "validate_upstream_snapshot_limits", lambda *_: [])
    monkeypatch.setattr(validator, "validate_upstream_snapshot", lambda *_: [])

    assert validator.fetch_and_validate_upstream(manifest_path) == []
    fetch_command = next(command for command in commands if "fetch" in command)
    sparse_command = next(
        command for command in commands if "sparse-checkout" in command
    )
    assert "--filter=blob:none" in fetch_command
    assert "/description/" in sparse_command


def test_whole_root_sparse_checkout_materializes_root_and_nested_files(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Retain the complete upstream tree when a snapshot boundary is whole-root."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(
        VALID_MANIFEST.replace(
            "vendored_paths:\n", "snapshot_path: description\nvendored_paths:\n"
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    upstream = tmp_path / "upstream"
    checkout = tmp_path / "checkout"
    upstream.mkdir()
    subprocess.run(["git", "-C", upstream, "init", "--quiet"], check=True)
    subprocess.run(
        ["git", "-C", upstream, "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", upstream, "config", "user.name", "Test Fixture"],
        check=True,
    )
    (upstream / "root.txt").write_text("root\n", encoding="utf-8")
    (upstream / "nested").mkdir()
    (upstream / "nested" / "file.txt").write_text("nested\n", encoding="utf-8")
    subprocess.run(["git", "-C", upstream, "add", "."], check=True)
    subprocess.run(
        ["git", "-C", upstream, "commit", "--quiet", "-m", "fixture"],
        check=True,
    )
    subprocess.run(
        ["git", "clone", "--quiet", "--no-checkout", upstream, checkout],
        check=True,
    )
    subprocess.run(
        [
            "git",
            "-C",
            checkout,
            "sparse-checkout",
            "set",
            "--no-cone",
            "--",
            *validator.upstream_sparse_checkout_patterns(manifest_path),
        ],
        check=True,
    )
    subprocess.run(["git", "-C", checkout, "checkout", "--quiet"], check=True)

    assert (checkout / "root.txt").read_text(encoding="utf-8") == "root\n"
    assert (checkout / "nested" / "file.txt").read_text(encoding="utf-8") == (
        "nested\n"
    )


def test_upstream_ancestry_timeout_stops_before_checkout(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Fail closed when branch ancestry verification times out."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)
    commands: list[list[str | Path]] = []

    def timeout_on_ancestry(
        arguments: list[str | Path], **kwargs: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        commands.append(arguments)
        if "merge-base" in arguments:
            raise subprocess.TimeoutExpired(
                arguments, validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
            )
        return subprocess.CompletedProcess(arguments, 0)

    monkeypatch.setattr(validator.subprocess, "run", timeout_on_ancestry)
    errors = validator.fetch_and_validate_upstream(manifest_path)
    assert errors == [
        "source/UPSTREAM.yaml timed out while verifying upstream branch ancestry"
    ]
    assert not any("checkout" in command for command in commands)


def test_upstream_checkout_timeout_stops_before_comparison(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Fail closed when a sparse upstream checkout does not complete."""
    source_root = tmp_path / "source"
    (source_root / "description").mkdir(parents=True)
    manifest_path = source_root / "UPSTREAM.yaml"
    manifest_path.write_text(VALID_MANIFEST, encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    mock_valid_upstream_metadata(monkeypatch)
    comparisons: list[tuple[Path, Path]] = []

    def timeout_on_checkout(
        arguments: list[str | Path], **kwargs: object
    ) -> subprocess.CompletedProcess[list[str | Path]]:
        if "checkout" in arguments and "sparse-checkout" not in arguments:
            raise subprocess.TimeoutExpired(
                arguments, validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
            )
        return subprocess.CompletedProcess(arguments, 0)

    def record_comparison(manifest: Path, checkout: Path) -> list[str]:
        comparisons.append((manifest, checkout))
        return []

    monkeypatch.setattr(validator.subprocess, "run", timeout_on_checkout)
    monkeypatch.setattr(validator, "validate_upstream_snapshot", record_comparison)
    errors = validator.fetch_and_validate_upstream(manifest_path)
    assert errors == ["source/UPSTREAM.yaml could not complete upstream checkout"]
    assert comparisons == []


def test_upstream_snapshot_file_limit_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a retained upstream snapshot with too many files."""
    checkout = tmp_path / "checkout"
    checkout.mkdir()
    (checkout / "first").touch()
    (checkout / "second").touch()
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.touch()
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "UPSTREAM_MAX_SNAPSHOT_FILES", 1)
    assert validator.validate_upstream_snapshot_limits(manifest_path, checkout) == [
        "UPSTREAM.yaml upstream snapshot exceeds 1 files"
    ]


def test_upstream_snapshot_byte_limit_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a retained upstream snapshot whose files are too large."""
    checkout = tmp_path / "checkout"
    checkout.mkdir()
    (checkout / "large").write_bytes(b"oversized")
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.touch()
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "UPSTREAM_MAX_SNAPSHOT_BYTES", 1)
    assert validator.validate_upstream_snapshot_limits(manifest_path, checkout) == [
        "UPSTREAM.yaml upstream snapshot exceeds 1 bytes"
    ]


def test_upstream_snapshot_limit_does_not_follow_symlink(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Count a retained symlink itself rather than its external target."""
    checkout = tmp_path / "checkout"
    checkout.mkdir()
    external_target = tmp_path / "external"
    external_target.write_bytes(b"x" * 1024)
    symlink = checkout / "link"
    symlink.symlink_to(external_target)
    manifest_path = tmp_path / "UPSTREAM.yaml"
    manifest_path.touch()
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(
        validator, "UPSTREAM_MAX_SNAPSHOT_BYTES", symlink.lstat().st_size
    )
    assert validator.validate_upstream_snapshot_limits(manifest_path, checkout) == []


def test_tracked_submodules_returns_gitlinks(monkeypatch: MonkeyPatch) -> None:
    """Return only stage entries whose Git mode identifies a submodule."""
    result = subprocess.CompletedProcess(
        args=["git", "ls-files", "--stage"],
        returncode=0,
        stdout=(
            "160000 aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa 0\t"
            "src/moveit_pro_sam2\n"
            "100644 bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb 0\tREADME.md\n"
        ),
    )

    def bounded_ls_files(*_: object, **kwargs: object) -> subprocess.CompletedProcess:
        assert kwargs["timeout"] == validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
        return result

    monkeypatch.setattr(validator.subprocess, "run", bounded_ls_files)
    assert validator.tracked_submodules() == {"src/moveit_pro_sam2"}


def test_tracked_submodules_timeout_fails_closed(monkeypatch: MonkeyPatch) -> None:
    """Convert a local Git timeout into a deterministic validation failure."""

    def timeout_ls_files(
        arguments: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess:
        assert kwargs["timeout"] == validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
        raise subprocess.TimeoutExpired(
            arguments, validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
        )

    monkeypatch.setattr(validator.subprocess, "run", timeout_ls_files)
    with raises(RuntimeError, match="^could not inspect tracked submodules$"):
        validator.tracked_submodules()


def test_optional_model_dependency_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject an ordinary package that requires an optional model package."""
    package = tmp_path / "src" / "example"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package><exec_depend>moveit_pro_sam2</exec_depend></package>",
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    errors = validator.validate_optional_model_dependencies()
    assert any("makes optional model packages mandatory" in error for error in errors)


def test_unaccounted_vendored_root_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject an external dependency root without provenance metadata."""
    source_root = tmp_path / "src" / "external_dependencies" / "missing"
    source_root.mkdir(parents=True)
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    manifests, errors = validator.discover_vendoring_manifests()
    assert manifests == []
    assert errors == [
        "vendored source has no UPSTREAM.yaml: src/external_dependencies/missing"
    ]


def test_multiple_manifests_in_vendored_root_fail(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject ambiguous provenance within one external dependency root."""
    source_root = tmp_path / "src" / "external_dependencies" / "ambiguous"
    (source_root / "first").mkdir(parents=True)
    (source_root / "second").mkdir()
    (source_root / "first" / "UPSTREAM.yaml").touch()
    (source_root / "second" / "UPSTREAM.yaml").touch()
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    manifests, errors = validator.discover_vendoring_manifests()
    assert manifests == []
    assert errors == [
        "vendored source has multiple UPSTREAM.yaml files: "
        "src/external_dependencies/ambiguous"
    ]


def test_nested_manifest_does_not_leave_sibling_content_unaudited(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a manifest whose snapshot boundary excludes a root-level sibling."""
    source_root = tmp_path / "src" / "external_dependencies" / "nested"
    (source_root / "upstream_package").mkdir(parents=True)
    (source_root / "upstream_package" / "UPSTREAM.yaml").touch()
    (source_root / "undeclared_sibling.txt").write_text("not audited", encoding="utf-8")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)

    manifests, errors = validator.discover_vendoring_manifests()

    assert manifests == []
    assert errors == [
        "vendored source UPSTREAM.yaml must be at its root: "
        "src/external_dependencies/nested"
    ]


def test_excessive_manifest_count_stops_before_upstream_fetch(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject too many vendored sources before the first network operation."""
    manifests = [tmp_path / f"source-{index}" / "UPSTREAM.yaml" for index in range(2)]
    monkeypatch.setattr(validator, "UPSTREAM_MAX_MANIFESTS", 1, raising=False)
    monkeypatch.setattr(validator, "validate_vendor_manifest", lambda _: [])

    def unexpected_fetch(_: Path) -> list[str]:
        raise AssertionError("upstream fetch must not run above the manifest cap")

    monkeypatch.setattr(validator, "fetch_and_validate_upstream", unexpected_fetch)

    assert validator.validate_vendored_roots(manifests, verify_upstream=True) == [
        "vendored dependency inventory exceeds 1 manifests"
    ]


def test_retired_path_fails(tmp_path: Path, monkeypatch: MonkeyPatch) -> None:
    """Reject a retired dependency path that reappears in the workspace."""
    retired_path = "src/external/retired"
    (tmp_path / retired_path).mkdir(parents=True)
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "RETIRED_PATHS", {retired_path})
    assert validator.validate_retired_paths() == [
        f"retired dependency path exists: {retired_path}"
    ]


def test_clearpath_timeout_parameters_match_schema(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Accept supported timeout keys for every Clearpath controller instance."""
    schema = tmp_path / "schema.yaml"
    schema.write_text("controller:\n  command_timeout: {\n    type: double\n  }\n")
    config = tmp_path / "controllers.yaml"
    config.write_text(
        "controller_manager:\n  ros__parameters:\n    first:\n"
        "      type: clearpath_mecanum_drive_controller/MecanumDriveController\n"
        "    second:\n"
        "      type: clearpath_mecanum_drive_controller/MecanumDriveController\n"
        "first:\n  ros__parameters:\n    command_timeout: 1.0\n"
        "second:\n  ros__parameters:\n    command_timeout: 2.0\n"
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "CLEARPATH_PARAMETER_SCHEMA", Path("schema.yaml"))
    monkeypatch.setattr(
        validator, "CLEARPATH_CONTROLLER_CONFIGS", {Path("controllers.yaml")}
    )

    assert validator.validate_clearpath_timeout_parameters() == []


def test_unsupported_clearpath_timeout_parameter_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject the legacy timeout key that generated parameters ignore."""
    schema = tmp_path / "schema.yaml"
    schema.write_text("controller:\n  command_timeout: {\n    type: double\n  }\n")
    config = tmp_path / "controllers.yaml"
    config.write_text(
        "controller_manager:\n  ros__parameters:\n    controller:\n"
        "      type: clearpath_mecanum_drive_controller/MecanumDriveController\n"
        "controller:\n  ros__parameters:\n    cmd_vel_timeout: 1.0\n"
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "CLEARPATH_PARAMETER_SCHEMA", Path("schema.yaml"))
    monkeypatch.setattr(
        validator, "CLEARPATH_CONTROLLER_CONFIGS", {Path("controllers.yaml")}
    )

    errors = validator.validate_clearpath_timeout_parameters()
    assert any(
        "unsupported Clearpath parameter cmd_vel_timeout" in error for error in errors
    )
    assert any(
        "must configure exactly one command_timeout" in error for error in errors
    )


def test_missing_clearpath_controller_instance_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a config that no longer exposes a discoverable Clearpath controller."""
    schema = tmp_path / "schema.yaml"
    schema.write_text("controller:\n  command_timeout: {\n    type: double\n  }\n")
    config = tmp_path / "controllers.yaml"
    config.write_text("controller_manager:\n  ros__parameters: {}\n")
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "CLEARPATH_PARAMETER_SCHEMA", Path("schema.yaml"))
    monkeypatch.setattr(
        validator, "CLEARPATH_CONTROLLER_CONFIGS", {Path("controllers.yaml")}
    )

    assert validator.validate_clearpath_timeout_parameters() == [
        "controllers.yaml does not declare any Clearpath controller instances"
    ]


def test_missing_clearpath_timeout_schema_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a parameter schema that no longer declares the configured timeout."""
    schema = tmp_path / "schema.yaml"
    schema.write_text("controller:\n  publish_rate: {\n    type: double\n  }\n")
    config = tmp_path / "controllers.yaml"
    config.write_text(
        "controller_manager:\n  ros__parameters:\n    controller:\n"
        "      type: clearpath_mecanum_drive_controller/MecanumDriveController\n"
        "controller:\n  ros__parameters:\n    command_timeout: 1.0\n"
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "CLEARPATH_PARAMETER_SCHEMA", Path("schema.yaml"))
    monkeypatch.setattr(
        validator, "CLEARPATH_CONTROLLER_CONFIGS", {Path("controllers.yaml")}
    )

    assert "schema.yaml does not declare the command_timeout parameter" in (
        validator.validate_clearpath_timeout_parameters()
    )


def test_missing_clearpath_timeout_parameter_fails(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    """Reject a Clearpath controller instance with no command timeout."""
    schema = tmp_path / "schema.yaml"
    schema.write_text("controller:\n  command_timeout: {\n    type: double\n  }\n")
    config = tmp_path / "controllers.yaml"
    config.write_text(
        "controller_manager:\n  ros__parameters:\n    controller:\n"
        "      type: clearpath_mecanum_drive_controller/MecanumDriveController\n"
        "controller:\n  ros__parameters:\n    publish_rate: 50.0\n"
    )
    monkeypatch.setattr(validator, "REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(validator, "CLEARPATH_PARAMETER_SCHEMA", Path("schema.yaml"))
    monkeypatch.setattr(
        validator, "CLEARPATH_CONTROLLER_CONFIGS", {Path("controllers.yaml")}
    )

    assert validator.validate_clearpath_timeout_parameters() == [
        "controllers.yaml controller controller must configure exactly one "
        "command_timeout parameter, found 0"
    ]


def test_main_reports_tracked_submodule_inspection_failure(
    monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """Return a deterministic CLI failure when local Git exits unsuccessfully."""

    def fail_ls_files(
        arguments: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess:
        assert arguments == ["git", "ls-files", "--stage"]
        assert kwargs["timeout"] == validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
        raise subprocess.CalledProcessError(returncode=1, cmd=arguments)

    monkeypatch.setattr(validator.subprocess, "run", fail_ls_files)
    assert validator.main() == 1
    assert capsys.readouterr().err == "ERROR: could not inspect tracked submodules\n"


def test_main_succeeds_for_valid_workspace(
    monkeypatch: MonkeyPatch, capsys: CaptureFixture[str]
) -> None:
    """Report success when every dependency-policy check passes."""
    monkeypatch.setattr(
        validator, "tracked_submodules", lambda: validator.ALLOWED_SUBMODULES
    )
    monkeypatch.setattr(
        validator,
        "discover_vendoring_manifests",
        lambda: ([Path(f"vendor-{index}") for index in range(8)], []),
    )
    monkeypatch.setattr(validator, "validate_optional_model_dependencies", lambda: [])
    monkeypatch.setattr(
        validator, "validate_vendored_roots", lambda manifests, **kwargs: []
    )
    monkeypatch.setattr(validator, "validate_retired_paths", lambda: [])
    monkeypatch.setattr(validator, "validate_clearpath_timeout_parameters", lambda: [])
    assert validator.main() == 0
    assert (
        "Validated 8 vendored sources and 2 optional ML submodules."
        in capsys.readouterr().out
    )


def test_apache_material_detected_in_licenses_directory(tmp_path: Path) -> None:
    """Detect Apache material declared as LICENSES/Apache-2.0.txt.

    phoebe_ws states its Apache grant this way. Missing it turns off the
    classification check on the one tree that vendors Apache material, and
    reports success while doing so.
    """
    source_root = tmp_path / "source"
    (source_root / "LICENSES").mkdir(parents=True)
    (source_root / "LICENSES" / "Apache-2.0.txt").write_text(
        "Apache License\nVersion 2.0\n", encoding="utf-8"
    )
    retains_apache, errors = validator.inspect_license_inventory(
        source_root, Path("source/UPSTREAM.yaml")
    )
    assert errors == []
    assert retains_apache
