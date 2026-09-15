"""End-to-end refresh tests using real, disposable Git repositories."""

from pathlib import Path
import os
import shutil
import subprocess
import sys

import pytest


BIN = Path(__file__).resolve().parents[1]


def git(root, *args):
    return subprocess.run(
        ["git", "-C", str(root), *args], check=True, capture_output=True, text=True
    ).stdout.strip()


def commit(root):
    git(root, "add", ".")
    git(root, "commit", "-qm", "fixture")
    return git(root, "rev-parse", "HEAD")


@pytest.fixture
def fixture(tmp_path):
    upstream = tmp_path / "upstream"
    workspace = tmp_path / "workspace"
    for root in (upstream, workspace):
        root.mkdir()
        git(root, "init", "-q", "-b", "main")
        git(root, "config", "user.email", "fixture@example.org")
        git(root, "config", "user.name", "Fixture")
    (upstream / "pkg").mkdir()
    (upstream / "pkg/file.txt").write_text("one\ntwo\nthree\nfour\nfive\n")
    old = commit(upstream)
    git(upstream, "tag", "v1.0.0")
    dep = workspace / "src/external_dependencies/demo"
    dep.mkdir(parents=True)
    shutil.copytree(upstream / "pkg", dep / "pkg")
    (dep / "UPSTREAM.yaml").write_text(
        "# Keep this annotation\nupstream:\n  repository: https://github.com/fixture/upstream.git\n"
        f"  commit: {old}\n  branch: main\nvendored_paths:\n  - pkg\n"
        "pruning_notes: []\nmodified_paths: []\nnotes:\n  - fixture annotation\n"
    )
    (workspace / "bin").mkdir()
    for script in BIN.glob("*.py"):
        shutil.copy2(script, workspace / "bin" / script.name)
    commit(workspace)
    return upstream, workspace, dep


def refresh(workspace, *args):
    return subprocess.run(
        [
            sys.executable,
            str(workspace / "bin/validate_workspace_dependencies.py"),
            "--refresh-from-upstream",
            *args,
        ],
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "GIT_CONFIG_COUNT": "1",
            "GIT_CONFIG_KEY_0": f"url.{workspace.parent / 'upstream'}.insteadOf",
            "GIT_CONFIG_VALUE_0": "https://github.com/fixture/upstream.git",
        },
    )


def test_no_downgrade_or_loss_of_fork_commits(fixture):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("fork patch\n")
    fork = commit(upstream)
    (dep / "pkg/file.txt").write_text("fork patch\n")
    manifest = dep / "UPSTREAM.yaml"
    import re

    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {fork}", manifest.read_text())
    )
    commit(workspace)
    before = manifest.read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "no eligible stable tag" in result.stderr
    assert manifest.read_bytes() == before
    assert (dep / "pkg/file.txt").read_text() == "fork patch\n"


def test_preserves_local_edits_and_pruning_with_snapshot_mapping(fixture):
    upstream, workspace, dep = fixture
    # Move the upstream root under a local snapshot prefix.
    (dep / "nested").mkdir()
    shutil.move(dep / "pkg", dep / "nested/pkg")
    manifest = dep / "UPSTREAM.yaml"
    manifest.write_text(
        manifest.read_text()
        .replace("vendored_paths:", "snapshot_path: nested\nvendored_paths:")
        .replace("  - pkg", "  - nested/pkg")
        .replace("modified_paths: []", "modified_paths:\n  - nested/pkg/file.txt")
    )
    (dep / "nested/pkg/file.txt").write_text("one\ntwo\nthree\nfour\nlocal\n")
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("upstream\ntwo\nthree\nfour\nfive\n")
    new = commit(upstream)
    git(upstream, "tag", "1.2.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert (
        dep / "nested/pkg/file.txt"
    ).read_text() == "upstream\ntwo\nthree\nfour\nlocal\n"
    assert f"commit: {new}" in manifest.read_text()


@pytest.mark.parametrize("kind", ["unstaged", "staged", "untracked", "ignored"])
def test_dirty_target_is_untouched(fixture, kind):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("upstream\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    if kind == "ignored":
        (workspace / ".gitignore").write_text("scratch\n")
        commit(workspace)
    target = dep / ("pkg/file.txt" if kind in ("staged", "unstaged") else "scratch")
    target.write_text("unsaved work\n")
    if kind == "staged":
        git(workspace, "add", ".")
    before = (dep / "UPSTREAM.yaml").read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "dirty" in result.stderr
    assert target.read_text() == "unsaved work\n"
    assert (dep / "UPSTREAM.yaml").read_bytes() == before


@pytest.mark.parametrize("kind", ["candidate-link", "upstream-link", "traversal"])
def test_rejects_unsafe_paths_without_writes(fixture, kind):
    upstream, workspace, dep = fixture
    outside = workspace.parent / "outside"
    outside.write_text("do not touch\n")
    if kind == "candidate-link":
        (dep / "pkg/file.txt").unlink()
        (dep / "pkg/file.txt").symlink_to(outside)
    elif kind == "upstream-link":
        (upstream / "pkg/file.txt").unlink()
        (upstream / "pkg/file.txt").symlink_to(outside)
        commit(upstream)
        git(upstream, "tag", "v2.0.0")
    else:
        manifest = dep / "UPSTREAM.yaml"
        manifest.write_text(manifest.read_text().replace("  - pkg", "  - ../demo/pkg"))
    commit(workspace) if git(workspace, "status", "--porcelain") else None
    before = (dep / "UPSTREAM.yaml").read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "unsafe" in result.stderr or "non-normalized" in result.stderr
    assert outside.read_text() == "do not touch\n"
    assert (dep / "UPSTREAM.yaml").read_bytes() == before


def test_pruned_files_stay_absent_and_local_additions_survive(fixture):
    upstream, workspace, dep = fixture
    (upstream / "pkg/pruned.txt").write_text("prune me\n")
    old = commit(upstream)
    manifest = dep / "UPSTREAM.yaml"
    import re

    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text()).replace(
            "modified_paths: []", "modified_paths:\n  - pkg/local.txt"
        )
    )
    (dep / "pkg/local.txt").write_text("local addition\n")
    commit(workspace)
    (upstream / "pkg/pruned.txt").write_text("still pruned\n")
    (upstream / "pkg/file.txt").write_text("new\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert not (dep / "pkg/pruned.txt").exists()
    assert (dep / "pkg/local.txt").read_text() == "local addition\n"
    assert (dep / "pkg/file.txt").read_text() == "new\n"


@pytest.mark.parametrize("change", ["addition", "deletion", "mode"])
def test_ambiguous_inventory_changes_require_manual_review(fixture, change):
    upstream, workspace, dep = fixture
    if change == "addition":
        (upstream / "pkg/new.txt").write_text("should this be pruned?\n")
    elif change == "deletion":
        (upstream / "pkg/file.txt").unlink()
    else:
        (upstream / "pkg/file.txt").chmod(0o755)
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    before = (dep / "UPSTREAM.yaml").read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "inventory" in result.stderr and "manual" in result.stderr
    assert "v2.0.0" in result.stderr
    assert git(upstream, "rev-parse", "HEAD") in result.stderr
    assert ("pkg/new.txt" if change == "addition" else "pkg/file.txt") in result.stderr
    assert (dep / "UPSTREAM.yaml").read_bytes() == before


@pytest.mark.parametrize("changed", [False, True])
def test_upstream_lfs_pointer_never_replaces_local_asset_bytes(fixture, changed):
    import hashlib

    upstream, workspace, dep = fixture
    payload = b"real mesh bytes\x00\xff"

    def pointer(content):
        return f"version https://git-lfs.github.com/spec/v1\noid sha256:{hashlib.sha256(content).hexdigest()}\nsize {len(content)}\n"

    (upstream / "pkg/mesh.bin").write_text(pointer(payload))
    # Commit pointer literally; no filter is installed in this fixture upstream.
    old = commit(upstream)
    (dep / "pkg/mesh.bin").write_bytes(payload)
    manifest = dep / "UPSTREAM.yaml"
    import re

    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text())
    )
    commit(workspace)
    if changed:
        (upstream / "pkg/mesh.bin").write_text(pointer(b"new mesh"))
    (upstream / "pkg/file.txt").write_text("new\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    result = refresh(workspace, "demo")
    if changed:
        assert result.returncode == 1
        assert "LFS" in result.stderr and "manual" in result.stderr
    else:
        assert result.returncode == 0, result.stderr
    assert (dep / "pkg/mesh.bin").read_bytes() == payload


@pytest.mark.parametrize("absorbed", [False, True])
def test_invalid_modification_ledger_leaves_candidate_untouched(fixture, absorbed):
    upstream, workspace, dep = fixture
    (dep / "pkg/file.txt").write_text("one\ntwo\nthree\nfour\nlocal\n")
    manifest = dep / "UPSTREAM.yaml"
    if absorbed:
        manifest.write_text(
            manifest.read_text().replace(
                "modified_paths: []", "modified_paths:\n  - pkg/file.txt"
            )
        )
        (upstream / "pkg/file.txt").write_bytes((dep / "pkg/file.txt").read_bytes())
    else:
        (upstream / "pkg/file.txt").write_text("new\ntwo\nthree\nfour\nfive\n")
    commit(workspace)
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    before = (dep / "pkg/file.txt").read_bytes(), manifest.read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "modified_paths" in result.stderr
    assert before == ((dep / "pkg/file.txt").read_bytes(), manifest.read_bytes())


def test_preview_all_then_targeted_apply(fixture):
    upstream, workspace, dep = fixture
    other = dep.parent / "other"
    shutil.copytree(dep, other)
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("new\n")
    commit(upstream)
    git(upstream, "tag", "v1.10.0")
    before = (dep / "UPSTREAM.yaml").read_bytes()
    preview = refresh(workspace, "all", "--dry-run")
    assert preview.returncode == 0, preview.stderr
    assert "demo:" in preview.stdout and "other:" in preview.stdout
    assert "dry-run" in preview.stdout
    assert (dep / "UPSTREAM.yaml").read_bytes() == before
    assert (other / "UPSTREAM.yaml").read_bytes() == before
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert (dep / "pkg/file.txt").read_text() == "new\n"
    assert (other / "UPSTREAM.yaml").read_bytes() == before
    assert refresh(workspace, "missing").returncode == 1


def test_numeric_version_never_decreases_after_untagged_pin(fixture):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("pinned\n")
    old = commit(upstream)
    (dep / "pkg/file.txt").write_text("pinned\n")
    manifest = dep / "UPSTREAM.yaml"
    import re

    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text())
    )
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("lower version\n")
    commit(upstream)
    git(upstream, "tag", "v0.9.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "no eligible stable tag" in result.stderr
    assert (dep / "pkg/file.txt").read_text() == "pinned\n"


def test_conflict_is_actionable_and_entire_dependency_untouched(fixture):
    upstream, workspace, dep = fixture
    (dep / "pkg/file.txt").write_text("local\n")
    manifest = dep / "UPSTREAM.yaml"
    manifest.write_text(
        manifest.read_text().replace(
            "modified_paths: []", "modified_paths:\n  - pkg/file.txt"
        )
    )
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("upstream\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    before = git(workspace, "status", "--porcelain")
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "conflict" in result.stderr and "pkg/file.txt" in result.stderr
    assert "manual" in result.stderr
    assert git(workspace, "status", "--porcelain") == before


def test_highest_numeric_stable_branch_tag_not_prerelease_or_other_branch(fixture):
    upstream, workspace, dep = fixture
    for tag in ("v1.9.0", "v1.10.0", "v20.0.0-rc1", "release-30.0.0"):
        (upstream / "pkg/file.txt").write_text(tag + "\n")
        commit(upstream)
        git(upstream, "tag", "-a", tag, "-m", tag)
    git(upstream, "checkout", "-qb", "other")
    (upstream / "pkg/file.txt").write_text("other branch\n")
    commit(upstream)
    git(upstream, "tag", "v99.0.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert (dep / "pkg/file.txt").read_text() == "v1.10.0\n"


def test_no_tags_never_uses_head(fixture):
    upstream, workspace, dep = fixture
    git(upstream, "tag", "-d", "v1.0.0")
    result = refresh(workspace, "all")
    assert result.returncode == 1
    assert "no eligible stable tag" in result.stderr
    assert "branch main" in result.stderr
    assert git(upstream, "rev-parse", "HEAD") in result.stderr
    assert "https://github.com/fixture/upstream.git" in result.stderr
    assert not git(workspace, "status", "--porcelain")


def test_duplicate_version_tags_with_different_commits_are_ambiguous(fixture):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("first\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    (upstream / "pkg/file.txt").write_text("second\n")
    commit(upstream)
    git(upstream, "tag", "2.0.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "ambiguous" in result.stderr
    for tag in ("v2.0.0", "2.0.0"):
        assert f"{tag}={git(upstream, 'rev-parse', tag)}" in result.stderr
    assert not git(workspace, "status", "--porcelain")


@pytest.mark.parametrize("kind", ["outside", "apache"])
def test_snapshot_policy_errors_block_refresh(fixture, kind):
    upstream, workspace, dep = fixture
    if kind == "outside":
        (dep / "stray.txt").write_text("unaccounted\n")
    else:
        (dep / "pkg/LICENSE").write_text("Apache License\n")
        (upstream / "pkg/LICENSE").write_text("Apache License\n")
        old = commit(upstream)
        manifest = dep / "UPSTREAM.yaml"
        import re

        manifest.write_text(
            re.sub(
                r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text()
            ).replace(
                "modified_paths: []",
                "modified_paths:\n  - pkg/file.txt\napache_paths:\n  - pkg",
            )
        )
        (dep / "pkg/file.txt").write_text(
            "one\ntwo\nthree\nfour\nlocal without notice\n"
        )
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("updated\ntwo\nthree\nfour\nfive\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert (
        "outside vendored_paths" if kind == "outside" else "notice"
    ) in result.stderr
    assert not git(workspace, "status", "--porcelain")


def test_only_commit_field_changes_preserving_crlf_and_note_text(fixture):
    upstream, workspace, dep = fixture
    manifest = dep / "UPSTREAM.yaml"
    old = git(upstream, "rev-parse", "HEAD")
    content = (
        manifest.read_text().replace(f"  commit: {old}", f"  commit:   {old}  ")
        + f"  - Historical  commit: {old}\n"
    )
    original = content.replace("\n", "\r\n").encode()
    manifest.write_bytes(original)
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("new\n")
    new = commit(upstream)
    git(upstream, "tag", "v2.0.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert manifest.read_bytes() == original.replace(
        f"commit:   {old}".encode(), f"commit:   {new}".encode()
    )


def test_all_apply_and_verify_against_real_upstream(fixture):
    upstream, workspace, dep = fixture
    other = dep.parent / "other"
    shutil.copytree(dep, other)
    commit(workspace)
    (upstream / "pkg/file.txt").write_text("latest stable\n")
    new = commit(upstream)
    git(upstream, "tag", "v2.0.0")
    result = refresh(workspace, "all")
    assert result.returncode == 0, result.stderr
    for target in (dep, other):
        assert f"commit: {new}" in (target / "UPSTREAM.yaml").read_text()
        # Exercise the existing read-only snapshot verifier after actual apply.
        checked = subprocess.run(
            [
                sys.executable,
                "-B",
                "-c",
                "import sys; from pathlib import Path; "
                "sys.path.insert(0, 'bin'); import validate_workspace_dependencies as v; "
                f"errors = v.validate_upstream_snapshot(Path({str(target / 'UPSTREAM.yaml')!r}), Path({str(upstream)!r})); "
                "print(errors); sys.exit(bool(errors))",
            ],
            cwd=workspace,
            text=True,
            capture_output=True,
        )
        assert checked.returncode == 0, checked.stdout + checked.stderr
    # Up-to-date invocation after the human commits is a no-op.
    commit(workspace)
    assert refresh(workspace, "all").returncode == 0
    assert not git(workspace, "status", "--porcelain")


def test_edits_made_during_fetch_are_not_overwritten(fixture, monkeypatch):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("new\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    wrappers = workspace.parent / "wrappers"
    wrappers.mkdir()
    # Real Git still runs. Simulate a human edit while the fetch is in progress.
    wrapper = wrappers / "git"
    wrapper.write_text(
        f"#!{sys.executable}\nimport subprocess, sys\nfrom pathlib import Path\n"
        f"result = subprocess.run([{shutil.which('git')!r}, *sys.argv[1:]])\n"
        "if 'fetch' in sys.argv:\n"
        f"    Path({str(dep / 'pkg/file.txt')!r}).write_text('edited during fetch\\n')\n"
        "sys.exit(result.returncode)\n"
    )
    wrapper.chmod(0o755)
    monkeypatch.setenv("PATH", str(wrappers) + os.pathsep + os.environ["PATH"])
    before = (dep / "UPSTREAM.yaml").read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "dirty" in result.stderr
    assert (dep / "pkg/file.txt").read_text() == "edited during fetch\n"
    assert (dep / "UPSTREAM.yaml").read_bytes() == before


@pytest.mark.parametrize("change", ["addition", "deletion", "mode"])
def test_inventory_changes_in_absent_old_subtree_are_ignored(fixture, change):
    import re

    upstream, workspace, dep = fixture
    docs = upstream / "pkg/docs"
    docs.mkdir()
    (docs / "old.txt").write_text("omitted\n")
    old = commit(upstream)
    manifest = dep / "UPSTREAM.yaml"
    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text())
    )
    commit(workspace)
    if change == "addition":
        (docs / "new.txt").write_text("also omitted\n")
    elif change == "deletion":
        (docs / "old.txt").unlink()
    else:
        (docs / "old.txt").chmod(0o755)
    (upstream / "pkg/file.txt").write_text("updated\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert (dep / "pkg/file.txt").read_text() == "updated\n"
    assert not (dep / "pkg/docs").exists()


@pytest.mark.parametrize("dry_run", [False, True])
def test_incoming_license_requires_metadata_before_any_write(fixture, dry_run):
    import re

    upstream, workspace, dep = fixture
    (upstream / "pkg/LICENSE").write_text("BSD-3-Clause\n")
    old = commit(upstream)
    shutil.copy2(upstream / "pkg/LICENSE", dep / "pkg/LICENSE")
    manifest = dep / "UPSTREAM.yaml"
    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text())
    )
    commit(workspace)
    (upstream / "pkg/LICENSE").write_text("Apache License\n")
    (upstream / "pkg/file.txt").write_text("updated\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    before = {
        p: (p.read_bytes(), p.stat().st_mtime_ns) for p in dep.rglob("*") if p.is_file()
    }
    result = refresh(workspace, "demo", *(["--dry-run"] if dry_run else []))
    assert result.returncode == 1
    assert "apache_paths" in result.stderr and "manual" in result.stderr
    assert before == {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in before}


@pytest.mark.parametrize("dry_run", [False, True])
@pytest.mark.parametrize("failure", [None, "sha", "size", "missing", "local-patch"])
def test_real_lfs_store_refresh(fixture, failure, dry_run):
    import hashlib
    import re

    upstream, workspace, dep = fixture
    git(upstream, "lfs", "install", "--local")
    git(upstream, "lfs", "track", "*.bin")
    payload = b"old mesh\x00\xff"
    incoming = b"new mesh\x00\xfe"
    asset = upstream / "pkg/mesh.bin"
    asset.write_bytes(payload)
    old = commit(upstream)
    (dep / "pkg/mesh.bin").write_bytes(payload)
    manifest = dep / "UPSTREAM.yaml"
    manifest.write_text(
        re.sub(r"commit: [0-9a-f]+", f"commit: {old}", manifest.read_text())
    )
    if failure == "local-patch":
        (dep / "pkg/mesh.bin").write_bytes(b"local binary patch\x00")
        manifest.write_text(
            manifest.read_text().replace(
                "modified_paths: []", "modified_paths:\n  - pkg/mesh.bin"
            )
        )
    commit(workspace)
    asset.write_bytes(incoming)
    commit(upstream)
    oid = hashlib.sha256(incoming).hexdigest()
    obj = upstream / ".git/lfs/objects" / oid[:2] / oid[2:4] / oid
    assert obj.read_bytes() == incoming
    if failure == "sha":
        obj.write_bytes(b"x" * len(incoming))
    elif failure == "missing":
        obj.unlink()
    elif failure == "size":
        pointer = git(upstream, "show", "HEAD:pkg/mesh.bin") + "\n"
        # Commit a deliberately invalid size without the clean filter repairing it.
        (upstream / ".gitattributes").write_text("")
        asset.write_text(
            pointer.replace(f"size {len(incoming)}", f"size {len(incoming) + 1}")
        )
        commit(upstream)
    git(upstream, "tag", "v2.0.0")
    before = {p: p.read_bytes() for p in dep.rglob("*") if p.is_file()}
    result = refresh(workspace, "demo", *(["--dry-run"] if dry_run else []))
    if failure:
        assert result.returncode == 1
        assert "LFS" in result.stderr and "pkg/mesh.bin" in result.stderr
        assert before == {p: p.read_bytes() for p in before}
    else:
        assert result.returncode == 0, result.stderr
        if dry_run:
            assert before == {p: p.read_bytes() for p in before}
        else:
            assert (dep / "pkg/mesh.bin").read_bytes() == incoming


@pytest.mark.parametrize("advance", [False, True])
def test_unchanged_files_keep_mtimes(fixture, advance):
    upstream, workspace, dep = fixture
    if advance:
        (upstream / "outside.txt").write_text("not retained\n")
        commit(upstream)
        git(upstream, "tag", "v2.0.0")
    paths = [dep / "pkg/file.txt"] + ([] if advance else [dep / "UPSTREAM.yaml"])
    before = {p: p.stat().st_mtime_ns for p in paths}
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert before == {p: p.stat().st_mtime_ns for p in paths}
    if not advance:
        assert "already current" in result.stdout


def test_clean_crlf_checkout_has_honest_remedy(fixture):
    upstream, workspace, dep = fixture
    git(workspace, "config", "core.autocrlf", "true")
    target = dep / "pkg/file.txt"
    target.unlink()
    git(workspace, "checkout", "--", str(target))
    assert b"\r\n" in target.read_bytes()
    assert not git(workspace, "status", "--porcelain")
    before = target.read_bytes()
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "CRLF" in result.stderr and "pkg/file.txt" in result.stderr
    assert "modified_paths disagrees" not in result.stderr
    assert "LF checkout" in result.stderr
    assert target.read_bytes() == before


def test_inherited_git_locations_do_not_redirect_refresh(fixture, monkeypatch):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("updated\n")
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    config_before = (workspace / ".git/config").read_bytes()
    refs_before = git(workspace, "show-ref")
    for key, value in {
        "GIT_DIR": str(workspace / ".git"),
        "GIT_WORK_TREE": str(workspace),
        "GIT_INDEX_FILE": str(workspace / ".git/index"),
        "GIT_COMMON_DIR": str(workspace / ".git"),
        "GIT_OBJECT_DIRECTORY": str(workspace / ".git/objects"),
        "GIT_PREFIX": "src/",
    }.items():
        monkeypatch.setenv(key, value)
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert (dep / "pkg/file.txt").read_text() == "updated\n"
    assert git(workspace, "show-ref") == refs_before
    assert (workspace / ".git/config").read_bytes() == config_before


def test_missing_branch_error_is_contextual(fixture):
    upstream, workspace, dep = fixture
    manifest = dep / "UPSTREAM.yaml"
    manifest.write_text(manifest.read_text().replace("branch: main", "branch: absent"))
    commit(workspace)
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "missing remote branch" in result.stderr
    assert "branch absent" in result.stderr
    assert git(upstream, "rev-parse", "HEAD") in result.stderr
    assert not git(workspace, "status", "--porcelain")


def test_git_error_does_not_echo_credentials(fixture, monkeypatch):
    upstream, workspace, dep = fixture
    wrappers = workspace.parent / "wrappers"
    wrappers.mkdir()
    wrapper = wrappers / "git"
    wrapper.write_text(
        f"#!{sys.executable}\nimport subprocess, sys\n"
        "if 'fetch' in sys.argv:\n"
        "    print('fatal: Authentication failed for https://user:secret-token@github.com/fixture/upstream.git Authorization: Bearer secret-token', file=sys.stderr)\n"
        "    sys.exit(128)\n"
        f"sys.exit(subprocess.run([{shutil.which('git')!r}, *sys.argv[1:]]).returncode)\n"
    )
    wrapper.chmod(0o755)
    monkeypatch.setenv("PATH", str(wrappers) + os.pathsep + os.environ["PATH"])
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "authentication/access failure" in result.stderr
    assert "secret-token" not in result.stderr
    assert "branch main" in result.stderr


def test_malformed_incoming_lfs_pointer_is_not_installed(fixture):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text(
        "version https://git-lfs.github.com/spec/v1\noid sha256:invalid\nsize 2\n"
    )
    commit(upstream)
    git(upstream, "tag", "v2.0.0")
    before = {p: p.read_bytes() for p in dep.rglob("*") if p.is_file()}
    result = refresh(workspace, "demo")
    assert result.returncode == 1
    assert "invalid LFS pointer" in result.stderr
    assert "pkg/file.txt" in result.stderr
    assert before == {p: p.read_bytes() for p in before}


def test_refresh_applies_tag_and_preserves_manifest_annotations(fixture):
    upstream, workspace, dep = fixture
    (upstream / "pkg/file.txt").write_text("updated\ntwo\nthree\nfour\nfive\n")
    new = commit(upstream)
    git(upstream, "tag", "v1.1.0")
    result = refresh(workspace, "demo")
    assert result.returncode == 0, result.stderr
    assert (dep / "pkg/file.txt").read_bytes() == (
        upstream / "pkg/file.txt"
    ).read_bytes()
    assert f"commit: {new}" in (dep / "UPSTREAM.yaml").read_text()
    assert "# Keep this annotation" in (dep / "UPSTREAM.yaml").read_text()
    assert "  - fixture annotation" in (dep / "UPSTREAM.yaml").read_text()
