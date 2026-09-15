"""Explicit, local-only refresh of vendored snapshots from stable upstream tags."""

from pathlib import Path
import hashlib
import os
import re
import subprocess
import sys
import tempfile

import validate_workspace_dependencies as validator

STABLE_TAG = re.compile(r"v?(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)")


def git(root: Path, *args: str) -> bytes:
    environment = os.environ.copy()
    # Keep authentication and URL rewrites, not the invoking alias's repository.
    for key in (
        "GIT_DIR",
        "GIT_WORK_TREE",
        "GIT_COMMON_DIR",
        "GIT_INDEX_FILE",
        "GIT_OBJECT_DIRECTORY",
        "GIT_ALTERNATE_OBJECT_DIRECTORIES",
        "GIT_PREFIX",
        "GIT_CEILING_DIRECTORIES",
        "GIT_DISCOVERY_ACROSS_FILESYSTEM",
    ):
        environment.pop(key, None)
    return subprocess.run(
        ["git", "-C", str(root), *args],
        check=True,
        capture_output=True,
        timeout=300,
        env=environment,
    ).stdout


def content_matches(local: bytes, upstream: bytes) -> bool:
    """Compare actual bytes with upstream content or its LFS object identity."""
    if pointer := validator.LFS_POINTER_OID.fullmatch(upstream):
        return (hashlib.sha256(local).hexdigest().encode(), len(local)) == (
            pointer[1],
            int(pointer[2]),
        )
    return local == upstream


def check_ledger(path: Path, local: bytes, upstream: bytes, manifest: dict) -> None:
    if (not content_matches(local, upstream)) != (
        path.as_posix() in manifest.get("modified_paths", [])
    ):
        raise ValueError(
            f"modified_paths disagrees at {path}; reconcile the ledger manually (including absorbed patches)"
        )
    if (
        not content_matches(local, upstream)
        and validator.path_is_declared(
            path, {Path(p) for p in manifest.get("apache_paths", [])}
        )
        and not validator.path_is_declared(
            path, {Path(p) for p in manifest.get("apache_excluded_paths", [])}
        )
        and validator.PICKNIK_MODIFICATION_NOTICE not in local
    ):
        raise ValueError(
            f"missing PickNik modification notice at {path}; review the Apache declaration manually"
        )


def require_clean(manifest_path: Path) -> None:
    """Do not fold uncommitted work into a refresh."""
    if git(
        manifest_path.parent,
        "status",
        "--porcelain",
        "--untracked-files=all",
        "--ignored",
        "--",
        ".",
    ):
        raise ValueError(
            "dirty dependency; commit or stash its changes (including ignored files) before refreshing"
        )


def refresh_one(manifest_path: Path, *, dry_run: bool = False) -> None:
    require_clean(manifest_path)
    # Refresh never follows symlinks, even ones the read-only validator accepts.
    for path in [
        manifest_path,
        *manifest_path.parents,
        *manifest_path.parent.rglob("*"),
    ]:
        if path.is_symlink():
            raise ValueError(
                f"unsafe symlink for automatic refresh: {path}; refresh manually"
            )
    if errors := validator.validate_vendor_manifest(manifest_path):
        raise ValueError("; ".join(errors))
    manifest = validator.parse_vendor_manifest(manifest_path)
    retained = {Path(value) for value in manifest["vendored_paths"]}
    for path in manifest_path.parent.rglob("*"):
        if (
            path.is_file()
            and path != manifest_path
            and not validator.path_is_declared(
                path.relative_to(manifest_path.parent), retained
            )
        ):
            raise ValueError(
                f"{path} is outside vendored_paths; reconcile the manifest manually"
            )
    upstream = manifest["upstream"]
    old = upstream["commit"]
    with tempfile.TemporaryDirectory(prefix="workspace-refresh-") as temporary:
        repository = Path(temporary)
        git(repository, "init", "--quiet")
        git(
            repository,
            "fetch",
            "--quiet",
            "--tags",
            upstream["repository"],
            f"+refs/heads/{upstream['branch']}:refs/remotes/upstream/branch",
        )
        git(
            repository,
            "merge-base",
            "--is-ancestor",
            old,
            "refs/remotes/upstream/branch",
        )
        tags = (
            git(
                repository,
                "tag",
                "--merged",
                "refs/remotes/upstream/branch",
                "--contains",
                old,
            )
            .decode()
            .splitlines()
        )
        prior_tags = git(repository, "tag", "--merged", old).decode().splitlines()
        floor = max(
            (
                tuple(map(int, match.groups()))
                for name in prior_tags
                if (match := STABLE_TAG.fullmatch(name))
            ),
            default=(0, 0, 0),
        )
        versions = [
            (tuple(map(int, match.groups())), tag)
            for tag in tags
            if (match := STABLE_TAG.fullmatch(tag))
            and tuple(map(int, match.groups())) >= floor
        ]
        if not versions:
            raise ValueError(
                f"no eligible stable tag in {upstream['repository']} on branch {upstream['branch']} "
                f"containing pin {old}; review the configured branch and refresh manually"
            )
        version, tag = max(versions)
        tag_commits = {
            name: git(repository, "rev-parse", f"refs/tags/{name}^{{commit}}")
            .decode()
            .strip()
            for candidate_version, name in versions
            if candidate_version == version
        }
        commits = set(tag_commits.values())
        if len(commits) != 1:
            choices = ", ".join(
                f"{name}={sha}" for name, sha in sorted(tag_commits.items())
            )
            raise ValueError(
                f"ambiguous stable version tags: {choices}; select and refresh manually"
            )
        new = commits.pop()
        require_clean(manifest_path)
        snapshot = Path(manifest.get("snapshot_path", "."))
        boundaries = {
            Path(value).relative_to(snapshot) for value in manifest["vendored_paths"]
        }
        inventories = []
        for revision in (old, new):
            inventory = {}
            for entry in git(repository, "ls-tree", "-rz", revision).split(b"\0"):
                if not entry:
                    continue
                metadata, name = entry.split(b"\t", 1)
                relative = Path(name.decode())
                if validator.path_is_declared(relative, boundaries):
                    inventory[relative] = metadata.split()[0]
            inventories.append(inventory)
        # Only directories present upstream at the old pin and entirely absent
        # locally prove prior subtree pruning. New siblings remain ambiguous.
        pruned = {
            parent
            for path in inventories[0]
            for parent in path.parents
            if validator.path_is_declared(parent, boundaries)
            and not (manifest_path.parent / snapshot / parent).exists()
        }
        changed = sorted(
            path
            for path in inventories[0].keys() | inventories[1].keys()
            if inventories[0].get(path) != inventories[1].get(path)
            and not validator.path_is_declared(path, pruned)
        )
        if changed:
            raise ValueError(
                f"unsafe inventory change at {tag} ({new}): "
                + ", ".join(str(path) for path in changed)
                + "; pruning_notes cannot decide retention; refresh manually and review vendored_paths"
            )
        proposed = {}
        for boundary in manifest["vendored_paths"]:
            target = manifest_path.parent / boundary
            paths = target.rglob("*") if target.is_dir() else [target]
            for path in paths:
                if not path.is_file():
                    continue
                relative = path.relative_to(manifest_path.parent).relative_to(snapshot)
                modes = [inventory.get(relative, b"") for inventory in inventories]
                if modes == [b"", b""] and path.relative_to(
                    manifest_path.parent
                ).as_posix() in manifest.get("modified_paths", []):
                    continue
                for mode in modes:
                    if mode not in (b"100644", b"100755"):
                        raise ValueError(
                            f"unsafe or missing upstream file: {relative}; refresh manually"
                        )
                base = git(repository, "show", f"{old}:{relative.as_posix()}")
                incoming = git(repository, "show", f"{new}:{relative.as_posix()}")
                local = path.read_bytes()
                for content in (base, incoming, local):
                    if content.startswith(
                        b"version https://git-lfs.github.com/spec/"
                    ) and not validator.LFS_POINTER_OID.fullmatch(content):
                        raise ValueError(
                            f"invalid LFS pointer at {relative}; refresh manually"
                        )
                if validator.LFS_POINTER_OID.fullmatch(local):
                    raise ValueError(
                        f"LFS pointer in local snapshot {relative}; run git lfs pull before refreshing"
                    )
                local_relative = path.relative_to(manifest_path.parent)
                eol = git(
                    manifest_path.parent,
                    "ls-files",
                    "--eol",
                    "--",
                    local_relative.as_posix(),
                )
                if b"i/lf " in eol and b"w/crlf " in eol:
                    raise ValueError(
                        f"unsupported clean CRLF checkout conversion at {local_relative}; "
                        "use an LF checkout (review core.autocrlf and text/eol attributes, "
                        "then re-check out the clean files), or refresh manually; "
                        "do not add checkout conversion to modified_paths"
                    )
                check_ledger(local_relative, local, base, manifest)
                if any(
                    validator.LFS_POINTER_OID.fullmatch(content)
                    for content in (base, incoming)
                ):
                    if base != incoming:
                        pointer = validator.LFS_POINTER_OID.fullmatch(incoming)
                        if not pointer or not content_matches(local, base):
                            raise ValueError(
                                f"divergent or unsupported LFS change at {relative}; refresh manually"
                            )
                        storage = repository / "lfs-store"
                        try:
                            git(
                                repository,
                                "config",
                                "remote.origin.url",
                                upstream["repository"],
                            )
                            git(repository, "config", "lfs.storage", str(storage))
                            git(
                                repository,
                                "lfs",
                                "fetch",
                                f"--include={relative.as_posix()}",
                                "--exclude=",
                                "origin",
                                new,
                            )
                            oid = pointer[1].decode()
                            content = (
                                storage / "objects" / oid[:2] / oid[2:4] / oid
                            ).read_bytes()
                        except (OSError, subprocess.SubprocessError) as error:
                            raise ValueError(
                                f"could not retrieve LFS bytes at {relative} from {tag} ({new}); check Git LFS access and refresh manually"
                            ) from error
                        if not content_matches(
                            content, incoming
                        ) or validator.LFS_POINTER_OID.fullmatch(content):
                            raise ValueError(
                                f"invalid LFS SHA-256 or size at {relative} from {tag} ({new}); refresh manually"
                            )
                        proposed[path] = content
                        check_ledger(local_relative, content, incoming, manifest)
                    continue
                if local == base:
                    proposed[path] = incoming
                elif incoming != base and local != incoming:
                    for name, content in (
                        ("local", local),
                        ("base", base),
                        ("incoming", incoming),
                    ):
                        (repository / name).write_bytes(content)
                    try:
                        proposed[path] = git(
                            repository, "merge-file", "-p", "local", "base", "incoming"
                        )
                    except subprocess.CalledProcessError as error:
                        raise ValueError(
                            f"merge conflict at {local_relative}; rebase the local patch manually; candidate untouched"
                        ) from error
                check_ledger(
                    local_relative, proposed.get(path, local), incoming, manifest
                )
        updated_manifest, replacements = re.subn(
            rb"(?m)^(  commit:[ \t]*)" + old.encode() + rb"([ \t]*\r?)$",
            lambda match: match[1] + new.encode() + match[2],
            manifest_path.read_bytes(),
        )
        if replacements != 1:
            raise ValueError(
                "cannot safely replace commit field; normalize the manifest manually"
            )
        staged = repository / "candidate"
        # Validate the complete proposal, including unchanged files and local
        # additions, before touching any original bytes.
        import shutil

        shutil.copytree(manifest_path.parent, staged)
        for path, content in proposed.items():
            (staged / path.relative_to(manifest_path.parent)).write_bytes(content)
        staged_manifest = staged / manifest_path.name
        staged_manifest.write_bytes(updated_manifest)
        if errors := validator.validate_vendor_manifest(staged_manifest):
            raise ValueError(
                f"invalid proposed snapshot at {tag} ({new}); review metadata manually: "
                + "; ".join(errors)
            )
        require_clean(manifest_path)
        if not dry_run:
            for path, content in {**proposed, manifest_path: updated_manifest}.items():
                if path.read_bytes() != content:
                    path.write_bytes(content)
        print(
            f"{manifest_path.parent.name}: {tag} {old} -> {new}"
            + (" (already current)" if old == new else "")
            + (" (dry-run)" if dry_run else "")
        )


def main(root: Path, selection: str, *, dry_run: bool = False) -> int:
    manifests = sorted(
        (root / validator.EXTERNAL_DEPENDENCIES_ROOT).glob("*/UPSTREAM.yaml")
    )
    selected = [
        path
        for path in manifests
        if selection == "all" or path.parent.name == selection
    ]
    if not selected:
        print(
            f"ERROR: unknown dependency {selection}; use a manifest directory name",
            file=sys.stderr,
        )
        return 1
    failed = False
    for path in selected:
        try:
            refresh_one(path, dry_run=dry_run)
        except subprocess.SubprocessError as error:
            # Classify stderr, never echo it: Git/credential helpers may include
            # rewritten URLs, headers, or arbitrary secrets in diagnostics.
            detail = (getattr(error, "stderr", None) or b"").lower()
            reason = "Git operation failed; check Git access and the configured pin"
            if b"couldn't find remote ref" in detail:
                reason = "missing remote branch"
            elif any(
                word in detail
                for word in (
                    b"authentication",
                    b"permission denied",
                    b"could not read username",
                    b"403",
                    b"401",
                )
            ):
                reason = "authentication/access failure; check Git credentials"
            elif isinstance(error, subprocess.TimeoutExpired):
                reason = "Git operation timed out; check network access"
            elif "merge-base" in error.cmd:
                reason = (
                    "pin missing or outside configured branch ancestry; review manually"
                )
            context = ""
            manifest = validator.parse_vendor_manifest(path)
            upstream = manifest.get("upstream", {})
            if isinstance(upstream, dict) and not validator.validate_upstream(
                path, upstream
            ):
                context = f" in {upstream['repository']} on branch {upstream['branch']} at pin {upstream['commit']}"
            print(f"ERROR: {path.parent.name}: {reason}{context}", file=sys.stderr)
            failed = True
        except (ValueError, OSError) as error:
            print(f"ERROR: {path.parent.name}: {error}", file=sys.stderr)
            failed = True
    return int(failed)
