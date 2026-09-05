#!/usr/bin/env python3
"""Validate the workspace's source-dependency policy."""

from pathlib import Path, PureWindowsPath
import argparse
import fnmatch
import functools
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ALLOWED_SUBMODULES = {
    "src/moveit_pro_sam2",
    "src/moveit_pro_sam3",
}
OPTIONAL_MODEL_PACKAGES = {Path(path).name for path in ALLOWED_SUBMODULES}
EXTERNAL_DEPENDENCIES_ROOT = Path("src/external_dependencies")
RETIRED_PATHS = {
    "src/external_dependencies/ros2_kortex_vision",
    "src/external_dependencies/serial",
    "src/external_dependencies/ros2_robotiq_gripper/robotiq_driver",
    "src/external_dependencies/ros2_robotiq_gripper/robotiq_hardware_tests",
    "src/external_dependencies/ridgeback/ridgeback_control",
    "src/external_dependencies/ridgeback/ridgeback_msgs",
    "src/external_dependencies/ridgeback/ridgeback_navigation",
    "src/moveit_pro_clipseg",
    "src/moveit_pro_kinova_configs/kinova_gen3_site_config",
    "src/moveit_pro_ur_configs/picknik_ur_site_config",
}
FULL_GIT_SHA = re.compile(r"[0-9a-f]{40}")
GITHUB_REPOSITORY_PATH = re.compile(r"/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+\.git")
GIT_BRANCH_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._/-]*")
LFS_POINTER_OID = re.compile(
    rb"\Aversion https://git-lfs.github.com/spec/v1\n"
    rb"oid sha256:([0-9a-f]{64})\nsize ([0-9]+)\n?\Z"
)
PICKNIK_MODIFICATION_NOTICE = b"Modified by PickNik Inc."
# franka_description carries the full license title. phoebe_ws writes only the
# SPDX identifier, in a LICENSES/ directory. Match both.
APACHE_LICENSE_MARKERS = (b"Apache License", b"Apache-2.0")
ALLOWED_MANIFEST_KEYS = {
    "upstream",
    "snapshot_path",
    "vendored_paths",
    # Free-text rationale for what was dropped, not machine-checked paths.
    # Nothing reconciles deletions against upstream, so entries here are
    # documentation only.
    "pruning_notes",
    "modified_paths",
    "apache_paths",
    "apache_excluded_paths",
    "notes",
}
ALLOWED_UPSTREAM_KEYS = {"repository", "commit", "branch"}
VENDORED_PATHS_KEY = "vendored_paths"
DEPENDENCY_TAG = re.compile(
    r"<[a-z_]*depend(?:\s[^>]*)?>\s*([^<]+?)\s*</[a-z_]*depend>"
)
CLEARPATH_PARAMETER_SCHEMA = Path(
    "src/external_dependencies/clearpath_mecanum_drive_controller/"
    "clearpath_mecanum_drive_controller/src/clearpath_mecanum_drive_controller.yaml"
)
CLEARPATH_CONTROLLER_CONFIGS = {
    Path("src/hangar_sim/config/control/picknik_ur.ros2_control.yaml"),
    Path(
        "src/external_dependencies/phoebe_ws/src/phoebe_sim/config/control/"
        "dual_arm.ros2_control.yaml"
    ),
}
UPSTREAM_FETCH_TIMEOUT_SECONDS = 300
UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS = 300
UPSTREAM_MAX_MANIFESTS = 16
UPSTREAM_MAX_SNAPSHOT_FILES = 10_000
UPSTREAM_MAX_SNAPSHOT_BYTES = 1024 * 1024 * 1024
UPSTREAM_METADATA_TIMEOUT_SECONDS = 60
UPSTREAM_MAX_METADATA_BYTES = 16 * 1024 * 1024
UPSTREAM_MAX_TOTAL_METADATA_BYTES = 64 * 1024 * 1024
UPSTREAM_MAX_TOTAL_SNAPSHOT_FILES = 50_000
UPSTREAM_MAX_TOTAL_SNAPSHOT_BYTES = 4 * 1024 * 1024 * 1024
UPSTREAM_MAX_TOTAL_SECONDS = 900


class UpstreamValidationBudget:
    """Track aggregate resources consumed by one upstream-verification run."""

    def __init__(self) -> None:
        self.metadata_bytes = 0
        self.snapshot_files = 0
        self.snapshot_bytes = 0
        self.deadline = time.monotonic() + UPSTREAM_MAX_TOTAL_SECONDS
        self.exhaustion_error: str | None = None

    def deadline_error(self) -> str | None:
        """Return an error after the aggregate wall-clock deadline expires."""
        if self.exhaustion_error is not None:
            return self.exhaustion_error
        if time.monotonic() > self.deadline:
            self.exhaustion_error = (
                f"upstream verification exceeds {UPSTREAM_MAX_TOTAL_SECONDS} seconds"
            )
            return self.exhaustion_error
        return None

    def operation_timeout(self, maximum_seconds: int) -> float:
        """Cap one blocking operation by the remaining aggregate deadline."""
        remaining_seconds = self.deadline - time.monotonic()
        return max(0.001, min(float(maximum_seconds), remaining_seconds))

    def consume_metadata(self, byte_count: int) -> str | None:
        """Consume metadata response bytes and return an error above the run cap."""
        self.metadata_bytes += byte_count
        if self.metadata_bytes > UPSTREAM_MAX_TOTAL_METADATA_BYTES:
            self.exhaustion_error = (
                "upstream verification exceeds "
                f"{UPSTREAM_MAX_TOTAL_METADATA_BYTES} metadata bytes"
            )
        return self.exhaustion_error

    def consume_snapshot(self, file_count: int, byte_count: int) -> str | None:
        """Consume retained-tree resources and return an error above either cap."""
        self.snapshot_files += file_count
        self.snapshot_bytes += byte_count
        if self.snapshot_files > UPSTREAM_MAX_TOTAL_SNAPSHOT_FILES:
            self.exhaustion_error = (
                "upstream verification exceeds "
                f"{UPSTREAM_MAX_TOTAL_SNAPSHOT_FILES} retained files"
            )
        elif self.snapshot_bytes > UPSTREAM_MAX_TOTAL_SNAPSHOT_BYTES:
            self.exhaustion_error = (
                "upstream verification exceeds "
                f"{UPSTREAM_MAX_TOTAL_SNAPSHOT_BYTES} retained bytes"
            )
        return self.exhaustion_error


def tracked_submodules() -> set[str]:
    try:
        output = subprocess.run(
            ["git", "ls-files", "--stage"],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS,
        ).stdout
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        raise RuntimeError("could not inspect tracked submodules") from error
    return {
        line.split("\t", 1)[1]
        for line in output.splitlines()
        if line.startswith("160000 ")
    }


def parse_indented_manifest_line(
    manifest: dict[str, object], active_key: str | None, raw_line: str, line_number: int
) -> None:
    """Parse one list item or upstream field."""
    if raw_line.startswith("  - "):
        active_value = manifest.get(active_key or "")
        if not isinstance(active_value, list):
            raise ValueError(f"line {line_number}: list item has no list key")
        item = raw_line[4:].strip()
        if not item:
            raise ValueError(f"line {line_number}: list item is empty")
        active_value.append(item)
        return

    upstream = manifest.get("upstream")
    if active_key != "upstream" or not isinstance(upstream, dict):
        raise ValueError(f"line {line_number}: unexpected indentation")
    key, separator, value = raw_line.strip().partition(":")
    if not separator or not key or not value.strip():
        raise ValueError(f"line {line_number}: invalid upstream entry")
    if key not in ALLOWED_UPSTREAM_KEYS:
        raise ValueError(f"line {line_number}: unknown upstream key: {key}")
    if key in upstream:
        raise ValueError(f"line {line_number}: duplicate upstream key: {key}")
    upstream[key] = value.strip()


def parse_top_level_manifest_line(
    manifest: dict[str, object], raw_line: str, line_number: int
) -> str:
    """Parse one top-level manifest field and return its key."""
    key, separator, value = raw_line.partition(":")
    if not separator or not key:
        raise ValueError(f"line {line_number}: invalid top-level entry")
    if key not in ALLOWED_MANIFEST_KEYS:
        raise ValueError(f"line {line_number}: unknown top-level key: {key}")
    if key in manifest:
        raise ValueError(f"line {line_number}: duplicate top-level key: {key}")
    if key == "upstream" and not value.strip():
        manifest[key] = {}
    elif key == "snapshot_path" and value.strip():
        manifest[key] = value.strip()
    elif not value.strip() or value.strip() == "[]":
        manifest[key] = []
    else:
        raise ValueError(f"line {line_number}: unsupported scalar value")
    return key


def parse_vendor_manifest(path: Path) -> dict[str, object]:
    """Parse the deliberately small YAML subset used by UPSTREAM.yaml."""
    manifest: dict[str, object] = {}
    active_key: str | None = None

    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        if (
            raw_line.startswith("   ")
            or raw_line != raw_line.lstrip()
            and not raw_line.startswith("  ")
        ):
            raise ValueError(f"line {line_number}: unsupported indentation")
        if raw_line.startswith("  "):
            parse_indented_manifest_line(manifest, active_key, raw_line, line_number)
        else:
            active_key = parse_top_level_manifest_line(manifest, raw_line, line_number)

    return manifest


def is_normalized_manifest_path(value: str, *, allow_root: bool = False) -> bool:
    """Return whether a manifest path is portable, relative, and traversal-free."""
    if (
        Path(value).is_absolute()
        or PureWindowsPath(value).drive
        or "\\" in value
        or any(character in value for character in "*?[]")
    ):
        return False
    parts = value.split("/")
    if any(part in ("", "..") for part in parts):
        return False
    return "." not in parts or (allow_root and parts == ["."])


def parse_modified_entries(
    values: object, relative_path: Path
) -> tuple[list[str], list[str]]:
    """Read modified_paths, rejecting a path declared more than once."""
    paths: list[str] = []
    errors: list[str] = []
    if values is not None and not isinstance(values, list):
        errors.append(f"{relative_path} must contain a modified_paths list")
        return paths, errors
    if not isinstance(values, list):
        return paths, errors
    for value in values:
        if not isinstance(value, str):
            errors.append(f"{relative_path} modified_paths entries must be strings")
            continue
        entry = value.strip()
        if entry in paths:
            errors.append(f"{relative_path} declares {entry} in modified_paths twice")
            continue
        paths.append(entry)
    return paths, errors


def validate_manifest_path_list(
    path: Path,
    relative_path: Path,
    values: object,
    *,
    key: str,
    require_nonempty: bool = False,
) -> list[str]:
    """Validate one manifest path list for shape, confinement, and existence."""
    if not isinstance(values, list):
        return [f"{relative_path} {key} must be a list"]
    if require_nonempty and not values:
        return [f"{relative_path} must retain at least one vendored path"]

    errors: list[str] = []
    manifest_root = path.parent.resolve()
    singular_key = "vendored path" if key == VENDORED_PATHS_KEY else "modified path"
    for value in values:
        if not isinstance(value, str) or not value:
            errors.append(f"{relative_path} contains an invalid {singular_key}")
            continue
        if key == VENDORED_PATHS_KEY and value == ".":
            errors.append(
                f"{relative_path} must identify concrete vendored paths, not ."
            )
            continue
        if not is_normalized_manifest_path(
            value, allow_root=key in ("apache_paths", "apache_excluded_paths")
        ):
            errors.append(
                f"{relative_path} {key} contains a non-normalized path: {value}"
            )
            continue
        try:
            resolved_path = (path.parent / value).resolve()
        except (OSError, RuntimeError):
            errors.append(
                f"{relative_path} {singular_key} could not be resolved: {value}"
            )
            continue
        if not resolved_path.is_relative_to(manifest_root):
            errors.append(
                f"{relative_path} {singular_key} escapes its source root: {value}"
            )
            continue
        if not resolved_path.exists():
            errors.append(
                f"{relative_path} references a missing {singular_key}: {value}"
            )
    return errors


def validate_upstream(relative_path: Path, upstream: dict[str, object]) -> list[str]:
    """Validate immutable upstream repository metadata."""
    errors: list[str] = []
    repository = upstream.get("repository")
    parsed_repository = urlparse(repository) if isinstance(repository, str) else None
    try:
        repository_port = (
            parsed_repository.port if parsed_repository is not None else None
        )
    except ValueError:
        repository_port = -1
    if (
        parsed_repository is None
        or parsed_repository.scheme != "https"
        or parsed_repository.hostname != "github.com"
        or parsed_repository.username is not None
        or parsed_repository.password is not None
        or repository_port not in (None, 443)
        or parsed_repository.query
        or parsed_repository.fragment
        or GITHUB_REPOSITORY_PATH.fullmatch(parsed_repository.path) is None
    ):
        errors.append(f"{relative_path} has an invalid upstream repository URL")

    commit = upstream.get("commit")
    if not isinstance(commit, str) or FULL_GIT_SHA.fullmatch(commit) is None:
        errors.append(f"{relative_path} must pin a full lowercase Git commit SHA")
    branch = upstream.get("branch")
    if (
        not isinstance(branch, str)
        or GIT_BRANCH_NAME.fullmatch(branch) is None
        or ".." in branch
        or "@{" in branch
        or "//" in branch
        or any(
            part.startswith(".") or part.endswith(".lock") for part in branch.split("/")
        )
        or branch.endswith(("/", "."))
    ):
        errors.append(f"{relative_path} must identify the upstream branch")
    return errors


def validate_retained_symlinks(
    root: Path, relative_manifest: Path, retained_paths: set[Path]
) -> list[str]:
    """Reject retained symlinks that are unsafe or do not resolve to confined files."""
    errors: list[str] = []
    resolved_root = root.resolve()
    for path in root.rglob("*"):
        if not path.is_symlink():
            continue
        relative_path = path.relative_to(root)
        if relative_path.is_relative_to(Path(".git")) or not path_is_declared(
            relative_path, retained_paths
        ):
            continue
        if path.name.startswith("LICENSE") or path.name == "COPYING":
            continue
        try:
            target = os.readlink(path)
        except OSError as error:
            errors.append(
                f"{relative_manifest} could not inspect retained symlink "
                f"{relative_path.as_posix()}: {error}"
            )
            continue
        target_path = Path(target)
        unsafe_target = (
            target_path.is_absolute()
            or PureWindowsPath(target).drive != ""
            or "\\" in target
            or any(part in ("", ".", "..") for part in target.split("/"))
        )
        try:
            resolved_target = (path.parent / target_path).resolve(strict=False)
        except (OSError, RuntimeError):
            errors.append(
                f"{relative_manifest} retained symlink target could not be resolved: "
                f"{relative_path.as_posix()} -> {target}"
            )
            continue
        if unsafe_target or not resolved_target.is_relative_to(resolved_root):
            errors.append(
                f"{relative_manifest} retained symlink has an unsafe target: "
                f"{relative_path.as_posix()} -> {target}"
            )
        elif not resolved_target.exists():
            errors.append(
                f"{relative_manifest} retained symlink target is missing: "
                f"{relative_path.as_posix()} -> {target}"
            )
        elif not resolved_target.is_file():
            errors.append(
                f"{relative_manifest} retained symlink target is not a file: "
                f"{relative_path.as_posix()} -> {target}"
            )
    return errors


def inspect_license_inventory(
    root: Path, relative_manifest: Path
) -> tuple[bool, list[str]]:
    """Inspect every license entry without dereferencing retained symlinks."""
    retains_apache_material = False
    errors: list[str] = []
    for license_path in root.rglob("*"):
        if not (
            license_path.name.startswith("LICENSE")
            or license_path.name == "COPYING"
            or license_path.parent.name == "LICENSES"
        ):
            continue
        relative_license = license_path.relative_to(root).as_posix()
        if license_path.is_symlink():
            errors.append(
                f"{relative_manifest} retained license file must not be a symlink: "
                f"{relative_license}"
            )
            continue
        if not license_path.is_file():
            continue
        try:
            with license_path.open("rb") as license_file:
                license_head = license_file.read(64 * 1024)
                if any(marker in license_head for marker in APACHE_LICENSE_MARKERS):
                    retains_apache_material = True
        except OSError as error:
            errors.append(
                f"{relative_manifest} could not inspect retained license file "
                f"{relative_license}: {error}"
            )
    return retains_apache_material, errors


def unclassified_apache_modifications(
    modified_paths: set[Path],
    apache_paths: set[Path],
    apache_excluded_paths: set[Path],
) -> set[Path]:
    """Return modifications without an explicit Apache license classification."""
    return {
        path
        for path in modified_paths
        if not path_is_declared(path, apache_paths)
        and not path_is_declared(path, apache_excluded_paths)
    }


def validate_vendor_manifest(path: Path) -> list[str]:
    relative_path = (
        path.relative_to(REPOSITORY_ROOT)
        if path.is_relative_to(REPOSITORY_ROOT)
        else path
    )
    try:
        manifest = parse_vendor_manifest(path)
    except ValueError as error:
        return [f"{relative_path} is invalid: {error}"]

    upstream = manifest.get("upstream")
    if not isinstance(upstream, dict):
        return [f"{relative_path} must contain an upstream mapping"]

    errors = validate_upstream(relative_path, upstream)

    snapshot_value = manifest.get("snapshot_path", ".")
    snapshot_root: Path | None = None
    if not isinstance(snapshot_value, str) or not snapshot_value:
        errors.append(f"{relative_path} snapshot_path must be a nonempty string")
    elif not is_normalized_manifest_path(snapshot_value, allow_root=True):
        errors.append(
            f"{relative_path} snapshot_path must be a normalized relative path"
        )
    else:
        snapshot_root = (path.parent / snapshot_value).resolve()
        manifest_root = path.parent.resolve()
        if not snapshot_root.is_relative_to(manifest_root):
            errors.append(f"{relative_path} snapshot_path escapes its source root")
        elif not snapshot_root.is_dir():
            errors.append(f"{relative_path} snapshot_path is not a directory")

    for key in ("vendored_paths", "pruning_notes", "notes"):
        value = manifest.get(key)
        if not isinstance(value, list):
            errors.append(f"{relative_path} must contain a {key} list")

    notes = manifest.get("notes")
    if isinstance(notes, list) and not notes:
        errors.append(f"{relative_path} must contain at least one provenance note")

    vendored_values = manifest.get(VENDORED_PATHS_KEY)
    vendored_path_errors = validate_manifest_path_list(
        path,
        relative_path,
        vendored_values,
        key=VENDORED_PATHS_KEY,
        require_nonempty=True,
    )
    errors.extend(vendored_path_errors)
    if isinstance(vendored_values, list) and not vendored_path_errors:
        retained_paths = {
            Path(value) for value in vendored_values if isinstance(value, str)
        }
        errors.extend(
            validate_retained_symlinks(path.parent, relative_path, retained_paths)
        )
    if (
        snapshot_root is not None
        and isinstance(vendored_values, list)
        and not vendored_path_errors
    ):
        for value in vendored_values:
            if (
                isinstance(value, str)
                and is_normalized_manifest_path(value)
                and not (path.parent / value).resolve().is_relative_to(snapshot_root)
            ):
                errors.append(
                    f"{relative_path} vendored path is outside snapshot_path: {value}"
                )
    modified_paths = manifest.get("modified_paths")
    if modified_paths is not None:
        modified_path_values, entry_errors = parse_modified_entries(
            modified_paths, relative_path
        )
        errors.extend(entry_errors)
        if not entry_errors:
            errors.extend(
                validate_manifest_path_list(
                    path, relative_path, modified_path_values, key="modified_paths"
                )
            )
    apache_paths = manifest.get("apache_paths")
    if apache_paths is not None:
        errors.extend(
            validate_manifest_path_list(
                path,
                relative_path,
                apache_paths,
                key="apache_paths",
                require_nonempty=True,
            )
        )
    apache_excluded_paths = manifest.get("apache_excluded_paths")
    if apache_excluded_paths is not None:
        errors.extend(
            validate_manifest_path_list(
                path,
                relative_path,
                apache_excluded_paths,
                key="apache_excluded_paths",
                require_nonempty=True,
            )
        )
        if apache_paths is None:
            errors.append(
                f"{relative_path} apache_excluded_paths requires apache_paths"
            )

    if isinstance(apache_paths, list) and isinstance(vendored_values, list):
        declared_vendored_paths = {
            Path(item) for item in vendored_values if isinstance(item, str)
        }
        for value in apache_paths:
            apache_path = Path(value) if isinstance(value, str) else None
            if apache_path is not None and not any(
                apache_path == vendored_path
                or apache_path.is_relative_to(vendored_path)
                or vendored_path.is_relative_to(apache_path)
                for vendored_path in declared_vendored_paths
            ):
                errors.append(
                    f"{relative_path} Apache path is outside vendored_paths: {value}"
                )
    if isinstance(apache_excluded_paths, list) and isinstance(vendored_values, list):
        declared_vendored_paths = {
            Path(item) for item in vendored_values if isinstance(item, str)
        }
        for value in apache_excluded_paths:
            if isinstance(value, str) and not path_is_declared(
                Path(value), declared_vendored_paths
            ):
                errors.append(
                    f"{relative_path} Apache exclusion is outside vendored_paths: {value}"
                )

    retains_apache_material, license_errors = inspect_license_inventory(
        path.parent, relative_path
    )
    errors.extend(license_errors)
    if retains_apache_material and apache_paths is None:
        errors.append(
            f"{relative_path} retains Apache-licensed material but does not declare apache_paths"
        )
    if (
        retains_apache_material
        and isinstance(modified_paths, list)
        and isinstance(apache_paths, list)
    ):
        declared_modified_paths = {
            Path(entry_path) for entry_path in modified_path_values
        }
        declared_apache_paths = {
            Path(item) for item in apache_paths if isinstance(item, str)
        }
        declared_apache_exclusions = {
            Path(item)
            for item in (
                apache_excluded_paths if isinstance(apache_excluded_paths, list) else []
            )
            if isinstance(item, str)
        }
        for unclassified_path in sorted(
            unclassified_apache_modifications(
                declared_modified_paths,
                declared_apache_paths,
                declared_apache_exclusions,
            )
        ):
            errors.append(
                f"{relative_path} modified path is not classified by apache_paths "
                f"or apache_excluded_paths: {unclassified_path}"
            )
    return errors


@functools.cache
def lfs_tracked_patterns(repository_root: Path) -> tuple[str, ...]:
    """Return the .gitattributes patterns that route files through Git LFS."""
    attributes = repository_root / ".gitattributes"
    if not attributes.is_file():
        return ()
    try:
        contents = attributes.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return ()
    patterns = []
    for line in contents.splitlines():
        entry = line.strip()
        if not entry or entry.startswith("#") or "filter=lfs" not in entry:
            continue
        patterns.append(entry.split()[0])
    return tuple(patterns)


def path_is_lfs_tracked(repository_relative_path: Path) -> bool:
    """Return whether .gitattributes routes this path through Git LFS."""
    posix_path = repository_relative_path.as_posix()
    return any(
        fnmatch.fnmatch(posix_path, pattern)
        or fnmatch.fnmatch(repository_relative_path.name, pattern)
        for pattern in lfs_tracked_patterns(REPOSITORY_ROOT)
    )


def file_is_lfs_tracked(path: Path) -> bool:
    """Return whether .gitattributes routes this file through Git LFS.

    Paths outside the repository have no .gitattributes entry, so this returns
    False and their pointer-shaped bytes stay untrusted.
    """
    try:
        repository_relative_path = path.relative_to(REPOSITORY_ROOT)
    except ValueError:
        return False
    return path_is_lfs_tracked(repository_relative_path)


def effective_file_digest(
    path: Path, *, trust_lfs_pointer: bool = True
) -> tuple[str, int]:
    """Return content digest and size, resolving Git LFS pointer metadata.

    A Git LFS pointer records the sha256 of the smudged content, so a pointer and
    the content it stands for hash identically. That is what lets this compare a
    skip-smudge upstream fetch against a smudged local checkout.

    It also means this treats pointer-shaped text as proof of content equality.
    Pass trust_lfs_pointer=False wherever the bytes are not guaranteed to be a
    real pointer, which is the candidate tree unless .gitattributes routes that
    path through LFS.
    """
    if path.is_symlink():
        link_content = f"symlink:{os.readlink(path)}".encode()
        return hashlib.sha256(link_content).hexdigest(), len(link_content)
    content = path.read_bytes()
    if trust_lfs_pointer and (match := LFS_POINTER_OID.fullmatch(content)):
        return match.group(1).decode(), int(match.group(2))
    return hashlib.sha256(content).hexdigest(), len(content)


def snapshot_files(
    root: Path, *, verify_lfs_tracking: bool = False
) -> dict[Path, tuple[str, int]]:
    """Return effective content identities for a source snapshot.

    Set verify_lfs_tracking for a tree inside this repository, where
    .gitattributes decides which files git smudges from a pointer. The upstream
    fetch sets GIT_LFS_SKIP_SMUDGE=1, so every pointer there is genuine.
    """
    snapshot: dict[Path, tuple[str, int]] = {}
    for path in root.rglob("*"):
        relative_path = path.relative_to(root)
        if not (path.is_file() or path.is_symlink()):
            continue
        if relative_path.is_relative_to(Path(".git")) or path.name == "UPSTREAM.yaml":
            continue
        trust_lfs_pointer = True
        if verify_lfs_tracking:
            trust_lfs_pointer = file_is_lfs_tracked(path)
        snapshot[relative_path] = effective_file_digest(
            path, trust_lfs_pointer=trust_lfs_pointer
        )
    return snapshot


def path_is_declared(path: Path, declared_paths: set[Path]) -> bool:
    """Return whether path is equal to or below a declared retained boundary."""
    return any(
        path == declared or path.is_relative_to(declared) for declared in declared_paths
    )


def validate_upstream_snapshot(manifest_path: Path, upstream_root: Path) -> list[str]:
    """Compare retained files with an independently fetched upstream snapshot."""
    relative_manifest = manifest_path.relative_to(REPOSITORY_ROOT)
    manifest = parse_vendor_manifest(manifest_path)
    vendored_values = manifest[VENDORED_PATHS_KEY]
    modified_values, modified_entry_errors = parse_modified_entries(
        manifest.get("modified_paths", []), relative_manifest
    )
    if modified_entry_errors:
        return modified_entry_errors
    apache_values = manifest.get("apache_paths", [])
    apache_excluded_values = manifest.get("apache_excluded_paths", [])
    if (
        not isinstance(vendored_values, list)
        or not isinstance(modified_values, list)
        or not isinstance(apache_values, list)
        or not isinstance(apache_excluded_values, list)
    ):
        return [f"{relative_manifest} has invalid retained-path metadata"]
    vendored_paths = {Path(str(path)) for path in vendored_values}
    modified_paths = {Path(str(path)) for path in modified_values}
    apache_paths = {Path(str(path)) for path in apache_values}
    apache_excluded_paths = {Path(str(path)) for path in apache_excluded_values}
    snapshot_path = Path(str(manifest.get("snapshot_path", ".")))
    retains_apache_material, candidate_license_errors = inspect_license_inventory(
        manifest_path.parent, relative_manifest
    )
    if candidate_license_errors:
        return candidate_license_errors
    _, upstream_license_errors = inspect_license_inventory(
        upstream_root, relative_manifest
    )
    if upstream_license_errors:
        return upstream_license_errors
    upstream_retained_paths = {
        path.relative_to(snapshot_path) for path in vendored_paths
    }
    if symlink_errors := validate_retained_symlinks(
        upstream_root, relative_manifest, upstream_retained_paths
    ):
        return symlink_errors
    candidate_files = snapshot_files(manifest_path.parent, verify_lfs_tracking=True)
    upstream_files = snapshot_files(upstream_root)

    errors = [
        f"{relative_manifest} retains a file outside vendored_paths: {path}"
        for path in sorted(candidate_files)
        if not path_is_declared(path, vendored_paths)
    ]
    for path in sorted(vendored_paths):
        if not path.is_relative_to(snapshot_path):
            errors.append(
                f"{relative_manifest} vendored path is outside snapshot_path: {path}"
            )
    effective_modifications = {
        path
        for path, digest in candidate_files.items()
        if path_is_declared(path, vendored_paths)
        and path.is_relative_to(snapshot_path)
        and upstream_files.get(path.relative_to(snapshot_path)) != digest
    }
    if retains_apache_material:
        for unclassified_path in sorted(
            unclassified_apache_modifications(
                effective_modifications, apache_paths, apache_excluded_paths
            )
        ):
            errors.append(
                f"{relative_manifest} modified path is not classified by "
                f"apache_paths or apache_excluded_paths: {unclassified_path}"
            )
    for path in sorted(effective_modifications - modified_paths):
        errors.append(f"{relative_manifest} omits a modified upstream path: {path}")
    for path in sorted(modified_paths - effective_modifications):
        errors.append(
            f"{relative_manifest} lists an unchanged upstream path as modified: {path}"
        )
    for path in sorted(effective_modifications & modified_paths):
        upstream_path = path.relative_to(snapshot_path)
        candidate_path = manifest_path.parent / path
        if (
            path_is_declared(path, apache_paths)
            and not path_is_declared(path, apache_excluded_paths)
            and upstream_path in upstream_files
            and candidate_path.is_file()
            and not candidate_path.is_symlink()
            and PICKNIK_MODIFICATION_NOTICE not in candidate_path.read_bytes()
        ):
            errors.append(
                f"{relative_manifest} modified Apache-licensed path lacks a "
                f"PickNik change notice: {path}"
            )
    return errors


def upstream_sparse_checkout_patterns(manifest_path: Path) -> list[str]:
    """Return anchored sparse-checkout patterns for one retained snapshot."""
    manifest = parse_vendor_manifest(manifest_path)
    vendored_values = manifest[VENDORED_PATHS_KEY]
    snapshot_path = Path(str(manifest.get("snapshot_path", ".")))
    if not isinstance(vendored_values, list):
        return []

    patterns: list[str] = []
    for value in vendored_values:
        candidate_path = Path(str(value))
        upstream_path = candidate_path.relative_to(snapshot_path)
        if upstream_path == Path("."):
            patterns.append("/*")
            continue
        pattern = f"/{upstream_path.as_posix()}"
        if (manifest_path.parent / candidate_path).is_dir():
            pattern += "/"
        patterns.append(pattern)
    return patterns


def fetch_upstream_tree_metadata(
    repository: str,
    commit: str,
    budget: UpstreamValidationBudget | None = None,
) -> tuple[list[dict], str | None]:
    """Fetch bounded GitHub tree metadata without downloading repository blobs."""
    if budget is not None and (deadline_error := budget.deadline_error()):
        return [], deadline_error
    repository_path = urlparse(repository).path.removeprefix("/").removesuffix(".git")
    url = (
        f"https://api.github.com/repos/{repository_path}/git/trees/{commit}?recursive=1"
    )
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "moveit-pro-workspace-dependency-validator",
        "X-GitHub-Api-Version": "2022-11-28",
    }
    if token := os.environ.get("GITHUB_TOKEN"):
        headers["Authorization"] = f"Bearer {token}"
    request = Request(url, headers=headers)
    try:
        timeout = (
            budget.operation_timeout(UPSTREAM_METADATA_TIMEOUT_SECONDS)
            if budget is not None
            else UPSTREAM_METADATA_TIMEOUT_SECONDS
        )
        with urlopen(request, timeout=timeout) as response:
            encoded_payload = response.read(UPSTREAM_MAX_METADATA_BYTES + 1)
    except (HTTPError, URLError, OSError, TimeoutError, ValueError):
        return [], "could not retrieve pinned tree metadata"
    if len(encoded_payload) > UPSTREAM_MAX_METADATA_BYTES:
        return [], "pinned tree metadata exceeds the response limit"
    if budget is not None and (
        budget_error := budget.consume_metadata(len(encoded_payload))
    ):
        return [], budget_error
    try:
        payload = json.loads(encoded_payload)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return [], "pinned tree metadata is malformed"
    if not isinstance(payload, dict) or payload.get("truncated") is not False:
        return [], "pinned tree metadata is truncated or malformed"
    tree = payload.get("tree")
    if not isinstance(tree, list) or not all(isinstance(entry, dict) for entry in tree):
        return [], "pinned tree metadata is malformed"
    return tree, None


def validate_upstream_tree_limits(
    manifest_path: Path,
    tree_entries: list[dict],
    budget: UpstreamValidationBudget | None = None,
) -> list[str]:
    """Reject oversized retained trees before Git downloads or materializes blobs."""
    relative_manifest = manifest_path.relative_to(REPOSITORY_ROOT)
    manifest = parse_vendor_manifest(manifest_path)
    snapshot_path = Path(str(manifest.get("snapshot_path", ".")))
    vendored_values = manifest[VENDORED_PATHS_KEY]
    if not isinstance(vendored_values, list):
        return [f"{relative_manifest} has invalid retained-path metadata"]
    retained_paths = {
        Path(str(path)).relative_to(snapshot_path) for path in vendored_values
    }
    file_count = 0
    total_bytes = 0
    for entry in tree_entries:
        path_value = entry.get("path")
        entry_type = entry.get("type")
        path = Path(path_value) if isinstance(path_value, str) else None
        if (
            path is None
            or not path_value
            or path.is_absolute()
            or any(part in ("", ".", "..") for part in path.parts)
        ):
            return [f"{relative_manifest} pinned tree metadata contains an unsafe path"]
        if not path_is_declared(path, retained_paths):
            continue
        if entry_type != "blob":
            if entry_type == "tree":
                continue
            return [
                f"{relative_manifest} retained upstream path has unsupported tree type: {path}"
            ]
        size = entry.get("size")
        if not isinstance(size, int) or isinstance(size, bool) or size < 0:
            return [
                f"{relative_manifest} retained upstream blob has invalid size metadata: {path}"
            ]
        file_count += 1
        total_bytes += size
        if file_count > UPSTREAM_MAX_SNAPSHOT_FILES:
            return [
                f"{relative_manifest} upstream snapshot exceeds "
                f"{UPSTREAM_MAX_SNAPSHOT_FILES} files"
            ]
        if total_bytes > UPSTREAM_MAX_SNAPSHOT_BYTES:
            return [
                f"{relative_manifest} upstream snapshot exceeds "
                f"{UPSTREAM_MAX_SNAPSHOT_BYTES} bytes"
            ]
    if budget is not None and (
        budget_error := budget.consume_snapshot(file_count, total_bytes)
    ):
        return [f"{relative_manifest} {budget_error}"]
    return []


def validate_upstream_snapshot_limits(
    manifest_path: Path, upstream_root: Path
) -> list[str]:
    """Reject retained snapshots that exceed bounded hashing resources."""
    relative_manifest = manifest_path.relative_to(REPOSITORY_ROOT)
    file_count = 0
    total_bytes = 0
    for path in upstream_root.rglob("*"):
        relative_path = path.relative_to(upstream_root)
        if relative_path.is_relative_to(Path(".git")) or not (
            path.is_file() or path.is_symlink()
        ):
            continue
        file_count += 1
        total_bytes += path.lstat().st_size
        if file_count > UPSTREAM_MAX_SNAPSHOT_FILES:
            return [
                f"{relative_manifest} upstream snapshot exceeds "
                f"{UPSTREAM_MAX_SNAPSHOT_FILES} files"
            ]
        if total_bytes > UPSTREAM_MAX_SNAPSHOT_BYTES:
            return [
                f"{relative_manifest} upstream snapshot exceeds "
                f"{UPSTREAM_MAX_SNAPSHOT_BYTES} bytes"
            ]
    return []


def fetch_and_validate_upstream(
    manifest_path: Path, budget: UpstreamValidationBudget | None = None
) -> list[str]:
    """Fetch one pinned upstream branch and validate its retained snapshot."""
    relative_manifest = manifest_path.relative_to(REPOSITORY_ROOT)
    manifest = parse_vendor_manifest(manifest_path)
    upstream = manifest["upstream"]
    if not isinstance(upstream, dict):
        return [f"{relative_manifest} has invalid upstream metadata"]
    repository = str(upstream["repository"])
    commit = str(upstream["commit"])
    branch = str(upstream["branch"])

    if budget is not None and (deadline_error := budget.deadline_error()):
        return [f"{relative_manifest} {deadline_error}"]
    tree_entries, metadata_error = fetch_upstream_tree_metadata(
        repository, commit, budget
    )
    if metadata_error is not None:
        return [f"{relative_manifest} {metadata_error}"]
    if limit_errors := validate_upstream_tree_limits(
        manifest_path, tree_entries, budget
    ):
        return limit_errors
    if budget is not None and (deadline_error := budget.deadline_error()):
        return [f"{relative_manifest} {deadline_error}"]

    def operation_timeout(maximum_seconds: int) -> float:
        return (
            budget.operation_timeout(maximum_seconds)
            if budget is not None
            else float(maximum_seconds)
        )

    with tempfile.TemporaryDirectory(prefix="workspace-upstream-") as temporary_dir:
        checkout = Path(temporary_dir) / "checkout"
        try:
            subprocess.run(
                ["git", "init", "--quiet", checkout],
                check=True,
                capture_output=True,
                text=True,
                timeout=operation_timeout(UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS),
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return [f"{relative_manifest} could not initialize upstream checkout"]
        if budget is not None and (deadline_error := budget.deadline_error()):
            return [f"{relative_manifest} {deadline_error}"]
        try:
            subprocess.run(
                [
                    "git",
                    "-C",
                    checkout,
                    "fetch",
                    "--quiet",
                    "--no-tags",
                    "--filter=blob:none",
                    repository,
                    f"+refs/heads/{branch}:refs/remotes/upstream/{branch}",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=operation_timeout(UPSTREAM_FETCH_TIMEOUT_SECONDS),
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            diagnostic = (
                error.stderr.strip().splitlines()[-1]
                if isinstance(error.stderr, str) and error.stderr
                else "unknown error"
            )
            return [
                f"{relative_manifest} could not fetch upstream branch {branch}: "
                f"{diagnostic}"
            ]

        upstream_ref = f"refs/remotes/upstream/{branch}"
        try:
            ancestry_result = subprocess.run(
                [
                    "git",
                    "-C",
                    checkout,
                    "merge-base",
                    "--is-ancestor",
                    commit,
                    upstream_ref,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=operation_timeout(UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS),
            )
        except subprocess.TimeoutExpired:
            return [
                f"{relative_manifest} timed out while verifying upstream branch ancestry"
            ]
        if ancestry_result.returncode == 1:
            return [
                f"{relative_manifest} pins commit {commit} outside upstream branch "
                f"{branch}"
            ]
        if ancestry_result.returncode != 0:
            diagnostic = (
                ancestry_result.stderr.strip().splitlines()[-1]
                if ancestry_result.stderr
                else "unknown error"
            )
            return [
                f"{relative_manifest} could not verify upstream branch ancestry: "
                f"{diagnostic}"
            ]

        environment = os.environ.copy()
        environment["GIT_LFS_SKIP_SMUDGE"] = "1"
        try:
            subprocess.run(
                [
                    "git",
                    "-C",
                    checkout,
                    "sparse-checkout",
                    "set",
                    "--no-cone",
                    "--",
                    *upstream_sparse_checkout_patterns(manifest_path),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=operation_timeout(UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS),
            )
            subprocess.run(
                ["git", "-C", checkout, "checkout", "--quiet", commit],
                check=True,
                capture_output=True,
                text=True,
                timeout=operation_timeout(UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS),
                env=environment,
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            phase = "sparse checkout" if "sparse-checkout" in error.cmd else "checkout"
            return [f"{relative_manifest} could not complete upstream {phase}"]
        if budget is not None and (deadline_error := budget.deadline_error()):
            return [f"{relative_manifest} {deadline_error}"]
        if limit_errors := validate_upstream_snapshot_limits(manifest_path, checkout):
            return limit_errors
        return validate_upstream_snapshot(manifest_path, checkout)


def validate_optional_model_dependencies() -> list[str]:
    errors: list[str] = []
    for package_manifest in (REPOSITORY_ROOT / "src").glob("**/package.xml"):
        relative_manifest = package_manifest.relative_to(REPOSITORY_ROOT)
        if any(
            relative_manifest.is_relative_to(Path(submodule))
            for submodule in ALLOWED_SUBMODULES
        ):
            continue
        dependencies = set(
            DEPENDENCY_TAG.findall(package_manifest.read_text(encoding="utf-8"))
        )
        mandatory_models = dependencies & OPTIONAL_MODEL_PACKAGES
        if mandatory_models:
            errors.append(
                f"{relative_manifest} makes optional model packages mandatory: "
                f"{sorted(mandatory_models)}"
            )
    return errors


def discover_vendoring_manifests() -> tuple[list[Path], list[str]]:
    """Find exactly one provenance manifest for each external dependency root."""
    external_dependencies = REPOSITORY_ROOT / EXTERNAL_DEPENDENCIES_ROOT
    manifests: list[Path] = []
    errors: list[str] = []
    for source_root in sorted(
        path for path in external_dependencies.iterdir() if path.is_dir()
    ):
        source_manifests = sorted(source_root.rglob("UPSTREAM.yaml"))
        relative_root = source_root.relative_to(REPOSITORY_ROOT)
        if not source_manifests:
            errors.append(f"vendored source has no UPSTREAM.yaml: {relative_root}")
            continue
        if len(source_manifests) > 1:
            errors.append(
                f"vendored source has multiple UPSTREAM.yaml files: {relative_root}"
            )
            continue
        if source_manifests[0].parent != source_root:
            errors.append(
                f"vendored source UPSTREAM.yaml must be at its root: {relative_root}"
            )
            continue
        manifests.append(source_manifests[0])
    return manifests, errors


def validate_vendored_roots(
    manifests: list[Path], *, verify_upstream: bool = False
) -> list[str]:
    if len(manifests) > UPSTREAM_MAX_MANIFESTS:
        return [
            f"vendored dependency inventory exceeds {UPSTREAM_MAX_MANIFESTS} manifests"
        ]
    errors: list[str] = []
    budget = UpstreamValidationBudget() if verify_upstream else None
    budget_exhausted = False
    for manifest in manifests:
        # Structural validation needs no network, so it runs for every manifest
        # even after the upstream budget is spent. Stopping at the exhausted
        # manifest would report one failure per CI run.
        manifest_errors = validate_vendor_manifest(manifest)
        errors.extend(manifest_errors)
        if verify_upstream and not manifest_errors and not budget_exhausted:
            errors.extend(fetch_and_validate_upstream(manifest, budget))
            budget_exhausted = (
                budget is not None and budget.exhaustion_error is not None
            )
    return errors


def validate_retired_paths() -> list[str]:
    return [
        f"retired dependency path exists: {relative_path}"
        for relative_path in sorted(RETIRED_PATHS)
        if (REPOSITORY_ROOT / relative_path).exists()
    ]


def validate_clearpath_timeout_parameters() -> list[str]:
    """Validate Clearpath controller timeout keys against its generated schema."""
    schema = (REPOSITORY_ROOT / CLEARPATH_PARAMETER_SCHEMA).read_text(encoding="utf-8")
    if re.search(r"^  command_timeout:\s*{", schema, re.MULTILINE) is None:
        return [
            f"{CLEARPATH_PARAMETER_SCHEMA} does not declare the command_timeout parameter"
        ]

    errors: list[str] = []
    for relative_path in CLEARPATH_CONTROLLER_CONFIGS:
        content = (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")
        section_matches = list(
            re.finditer(r"^([A-Za-z0-9_]+):\s*$", content, re.MULTILINE)
        )
        sections = {
            match.group(1): content[
                match.start() : (
                    section_matches[index + 1].start()
                    if index + 1 < len(section_matches)
                    else len(content)
                )
            ]
            for index, match in enumerate(section_matches)
        }
        controller_manager = sections.get("controller_manager", "")
        controller_names = re.findall(
            r"^    ([A-Za-z0-9_]+):\s*\n"
            r"      type: clearpath_mecanum_drive_controller/MecanumDriveController\s*$",
            controller_manager,
            re.MULTILINE,
        )
        if not controller_names:
            errors.append(
                f"{relative_path} does not declare any Clearpath controller instances"
            )
        for controller_name in controller_names:
            controller_config = sections.get(controller_name, "")
            if re.search(r"^\s+cmd_vel_timeout:\s*", controller_config, re.MULTILINE):
                errors.append(
                    f"{relative_path} controller {controller_name} uses unsupported "
                    "Clearpath parameter cmd_vel_timeout"
                )
            command_timeout_count = len(
                re.findall(r"^\s+command_timeout:\s*", controller_config, re.MULTILINE)
            )
            if command_timeout_count != 1:
                errors.append(
                    f"{relative_path} controller {controller_name} must configure exactly "
                    f"one command_timeout parameter, found {command_timeout_count}"
                )
    return errors


def main(*, verify_upstream: bool = False) -> int:
    try:
        actual_submodules = tracked_submodules()
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    vendoring_manifests, vendoring_errors = discover_vendoring_manifests()
    errors = validate_optional_model_dependencies()
    errors.extend(vendoring_errors)
    errors.extend(
        validate_vendored_roots(vendoring_manifests, verify_upstream=verify_upstream)
    )
    errors.extend(validate_retired_paths())
    errors.extend(validate_clearpath_timeout_parameters())
    if actual_submodules != ALLOWED_SUBMODULES:
        errors.append(
            "tracked submodules differ from the optional ML allowlist: "
            f"expected {sorted(ALLOWED_SUBMODULES)}, got {sorted(actual_submodules)}"
        )

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print(
        f"Validated {len(vendoring_manifests)} vendored sources and "
        f"{len(ALLOWED_SUBMODULES)} optional ML submodules."
    )
    return 0


if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument(
        "--verify-upstream",
        action="store_true",
        help="fetch pinned upstream branches and verify the modification ledgers",
    )
    arguments = argument_parser.parse_args()
    sys.exit(main(verify_upstream=arguments.verify_upstream))
