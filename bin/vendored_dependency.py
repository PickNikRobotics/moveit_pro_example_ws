#!/usr/bin/env python3
"""Report drift of vendored dependencies from upstream, and refresh one to a newer commit."""

from dataclasses import dataclass
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlparse
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_workspace_dependencies as validator  # noqa: E402

COMPARE_TIMEOUT_SECONDS = 30
COMPARE_MAX_RESPONSE_BYTES = 16 * 1024 * 1024
REFRESH_GIT_IDENTITY = {
    "GIT_AUTHOR_NAME": "vendored_dependency.py",
    "GIT_AUTHOR_EMAIL": "noreply@picknik.ai",
    "GIT_COMMITTER_NAME": "vendored_dependency.py",
    "GIT_COMMITTER_EMAIL": "noreply@picknik.ai",
}
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


class RefreshError(Exception):
    """A refresh step failed in a way the user must act on."""


def git(
    checkout: Path, *arguments: str, check: bool = True
) -> subprocess.CompletedProcess[str]:
    """Run one Git command inside the temporary upstream checkout."""
    timeout = validator.UPSTREAM_GIT_OPERATION_TIMEOUT_SECONDS
    if arguments and arguments[0] == "fetch":
        timeout = validator.UPSTREAM_FETCH_TIMEOUT_SECONDS
    return subprocess.run(
        ["git", "-c", "commit.gpgsign=false", "-C", str(checkout), *arguments],
        check=check,
        capture_output=True,
        text=True,
        timeout=timeout,
        env={
            **os.environ,
            **REFRESH_GIT_IDENTITY,
            "GIT_TERMINAL_PROMPT": "0",
            "GIT_ASKPASS": "echo",
        },
    )


def discard_command(relative_dir: Path) -> str:
    """Return the git command that throws away a refresh of one vendored source."""
    return (
        f"git restore --source=HEAD --staged --worktree -- {relative_dir} "
        f"&& git clean -fd -- {relative_dir}"
    )


def git_subcommand(command: list[str | Path]) -> str:
    """Name the Git subcommand in a command `git()` built, for diagnostics."""
    parts = [str(part) for part in command]
    start = parts.index("-C") + 2 if "-C" in parts else 1
    return " ".join(parts[start : start + 2])


def nul_separated(output: str) -> list[str]:
    """Split `git ... -z` output into paths, which may contain whitespace."""
    return [path for path in output.split("\0") if path]


def last_line(output: str | None) -> str:
    """Return the last line of a Git stderr stream, for diagnostics."""
    lines = output.strip().splitlines() if output else []
    return lines[-1] if lines else "no output"


def resolve_manifest(source: str) -> Path:
    """Find the manifest for a vendored source given its name or path."""
    manifests, errors = validator.discover_vendoring_manifests()
    if errors:
        raise RefreshError("\n".join(errors))
    candidate = Path(source)
    for manifest_path in manifests:
        if manifest_path.parent.name == source:
            return manifest_path
        if candidate.is_absolute() and candidate.resolve() in (
            manifest_path.parent,
            manifest_path,
        ):
            return manifest_path
        if (validator.REPOSITORY_ROOT / candidate).resolve() in (
            manifest_path.parent,
            manifest_path,
        ):
            return manifest_path
    names = ", ".join(path.parent.name for path in manifests)
    raise RefreshError(f"no vendored source named {source}; known sources: {names}")


def lfs_pointers(root: Path) -> list[Path]:
    """Return vendored files that are unsmudged Git LFS pointers."""
    pointers = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and not path.is_symlink() and path.stat().st_size < 512:
            if validator.LFS_POINTER_OID.match(path.read_bytes()):
                pointers.append(path.relative_to(root))
    return pointers


def replace_tree(source: Path, destination: Path) -> None:
    """Replace destination with a copy of source, which may be a file or directory."""
    if destination.is_symlink() or destination.is_file():
        destination.unlink()
    elif destination.is_dir():
        shutil.rmtree(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.is_dir() and not source.is_symlink():
        shutil.copytree(source, destination, symlinks=True)
    else:
        shutil.copy2(source, destination, follow_symlinks=False)


@dataclass(frozen=True)
class Source:
    """One vendored source as its UPSTREAM.yaml describes it."""

    manifest_path: Path
    relative_dir: Path
    repository: str
    old_commit: str
    branch: str
    snapshot_path: Path
    retained: list[tuple[Path, Path]]
    manifest_text: str

    @property
    def pinned_line(self) -> str:
        return f"  commit: {self.old_commit}\n"

    @property
    def discard(self) -> str:
        return discard_command(self.relative_dir)


def load_source(manifest_path: Path) -> Source:
    """Validate a manifest and gather what the refresh needs from it."""
    relative_manifest = manifest_path.relative_to(validator.REPOSITORY_ROOT)
    if errors := validator.validate_vendor_manifest(manifest_path):
        raise RefreshError("\n".join(errors))
    manifest = validator.parse_vendor_manifest(manifest_path)
    upstream = manifest["upstream"]
    if not isinstance(upstream, dict):
        raise RefreshError(f"{relative_manifest} has invalid upstream metadata")
    old_commit = str(upstream["commit"])
    pinned_line = f"  commit: {old_commit}\n"
    manifest_text = manifest_path.read_text(encoding="utf-8")
    if manifest_text.count(pinned_line) != 1:
        raise RefreshError(
            f"{relative_manifest} must contain exactly one `{pinned_line.strip()}` line"
        )
    snapshot_path = Path(str(manifest.get("snapshot_path", ".")))
    vendored_values = manifest[validator.VENDORED_PATHS_KEY]
    assert isinstance(vendored_values, list)
    retained = [
        (manifest_path.parent / str(value), Path(str(value)).relative_to(snapshot_path))
        for value in vendored_values
    ]
    if any(upstream_path == Path(".") for _, upstream_path in retained):
        raise RefreshError(
            f"{relative_manifest} vendors snapshot_path itself, which refresh does not support"
        )
    if pointers := lfs_pointers(manifest_path.parent):
        listed = ", ".join(str(path) for path in pointers[:5])
        raise RefreshError(
            f"{relative_manifest.parent} holds unsmudged Git LFS pointers ({listed}). "
            "Run `git lfs pull` first so the refresh sees real file contents."
        )
    return Source(
        manifest_path=manifest_path,
        relative_dir=relative_manifest.parent,
        repository=str(upstream["repository"]),
        old_commit=old_commit,
        branch=str(upstream["branch"]),
        snapshot_path=snapshot_path,
        retained=retained,
        manifest_text=manifest_text,
    )


def fetch_upstream(checkout: Path, source: Source) -> str:
    """Fetch the pinned branch into a promisor remote; return its local ref."""
    upstream_ref = f"refs/remotes/upstream/{source.branch}"
    git(checkout, "remote", "add", "upstream", source.repository)
    git(checkout, "config", "remote.upstream.promisor", "true")
    git(checkout, "config", "remote.upstream.partialclonefilter", "blob:none")
    try:
        git(
            checkout,
            "fetch",
            "--quiet",
            "--no-tags",
            "--filter=blob:none",
            "upstream",
            f"+refs/heads/{source.branch}:{upstream_ref}",
        )
    except subprocess.CalledProcessError as error:
        raise RefreshError(
            f"could not fetch upstream branch {source.branch} from "
            f"{source.repository}: {last_line(error.stderr)}"
        ) from error
    return upstream_ref


def resolve_target(
    checkout: Path, source: Source, upstream_ref: str, target: str | None
) -> str:
    """Resolve the requested commit and confirm it sits on the pinned branch."""
    resolved = git(
        checkout,
        "rev-parse",
        "--verify",
        "--quiet",
        "--end-of-options",
        f"{target or upstream_ref}^{{commit}}",
        check=False,
    )
    if resolved.returncode != 0:
        wanted = target if target is not None else f"the head of {source.branch}"
        raise RefreshError(
            f"could not resolve {wanted} on upstream branch {source.branch}"
        )
    new_commit = resolved.stdout.strip()
    ancestry = git(
        checkout, "merge-base", "--is-ancestor", new_commit, upstream_ref, check=False
    )
    if ancestry.returncode == 1:
        raise RefreshError(f"{new_commit} is not on upstream branch {source.branch}")
    if ancestry.returncode != 0:
        raise RefreshError(
            f"could not verify that {new_commit} is on upstream branch "
            f"{source.branch}: {last_line(ancestry.stderr)}"
        )
    return new_commit


def capture_local_changes(
    checkout: Path, source: Source
) -> tuple[str, list[str]] | None:
    """Commit the vendored tree on top of the old pin; return (commit, files) or None."""
    git(
        checkout,
        "sparse-checkout",
        "set",
        "--no-cone",
        "--",
        *validator.upstream_sparse_checkout_patterns(source.manifest_path),
    )
    git(checkout, "checkout", "--quiet", source.old_commit)
    for vendored_path, upstream_path in source.retained:
        if vendored_path.exists() or vendored_path.is_symlink():
            replace_tree(vendored_path, checkout / upstream_path)
        elif (checkout / upstream_path).is_dir():
            shutil.rmtree(checkout / upstream_path)
        elif (checkout / upstream_path).exists():
            (checkout / upstream_path).unlink()
    git(
        checkout,
        "add",
        "--all",
        "--force",
        "--",
        *(str(path) for _, path in source.retained),
    )
    staged = git(checkout, "diff", "--cached", "--quiet", check=False)
    if staged.returncode not in (0, 1):
        raise RefreshError(
            f"could not diff local modifications: {last_line(staged.stderr)}"
        )
    if staged.returncode == 0:
        return None
    git(
        checkout,
        "commit",
        "--quiet",
        "--no-verify",
        "--message",
        f"Local modifications from {source.relative_dir}",
    )
    local_commit = git(checkout, "rev-parse", "HEAD").stdout.strip()
    files = nul_separated(
        git(
            checkout,
            "diff-tree",
            "--no-commit-id",
            "--name-only",
            "-r",
            "-z",
            local_commit,
        ).stdout
    )
    return local_commit, files


def reapply_local_changes(
    checkout: Path, source: Source, local_commit: str
) -> list[str]:
    """Cherry-pick the local changes onto the checked-out commit; return conflicts."""
    picked = git(checkout, "cherry-pick", "--no-commit", local_commit, check=False)
    if picked.returncode == 0:
        return []
    conflicts = nul_separated(
        git(checkout, "diff", "--name-only", "--diff-filter=U", "-z").stdout
    )
    if not conflicts:
        raise RefreshError(
            f"could not re-apply local modifications: {last_line(picked.stderr)}"
        )
    # A file pruned locally that upstream went on to modify is a modify/delete
    # conflict; the pruning was deliberate, so keep it deleted.
    pruned = set(
        nul_separated(
            git(
                checkout,
                "diff-tree",
                "--no-commit-id",
                "--name-only",
                "-r",
                "-z",
                "--diff-filter=D",
                source.old_commit,
                local_commit,
            ).stdout
        )
    )
    for path in [path for path in conflicts if path in pruned]:
        git(checkout, "rm", "--quiet", "--force", "--", path)
        conflicts.remove(path)
    return conflicts


def copy_back(checkout: Path, source: Source, new_commit: str) -> list[Path]:
    """Replace the vendored tree and pin; return retained paths upstream no longer has."""
    missing_upstream: list[Path] = []
    for vendored_path, upstream_path in source.retained:
        if (checkout / upstream_path).exists() or (
            checkout / upstream_path
        ).is_symlink():
            replace_tree(checkout / upstream_path, vendored_path)
        else:
            missing_upstream.append(
                vendored_path.relative_to(source.manifest_path.parent)
            )
    source.manifest_path.write_text(
        source.manifest_text.replace(
            source.pinned_line, f"  commit: {new_commit}\n", 1
        ),
        encoding="utf-8",
    )
    if pointers := lfs_pointers(source.manifest_path.parent):
        listed = ", ".join(str(path) for path in pointers[:5])
        raise RefreshError(
            f"refresh copied unsmudged Git LFS pointers ({listed}); install git-lfs "
            f"and unset GIT_LFS_SKIP_SMUDGE, then discard with: {source.discard}"
        )
    return missing_upstream


def check_ledger(checkout: Path, source: Source, new_commit: str) -> int:
    """Run the validator's ledger comparison against the pristine new tree."""
    git(checkout, "reset", "--quiet", "--hard", new_commit)
    ledger_errors = validator.validate_upstream_snapshot(source.manifest_path, checkout)
    if ledger_errors:
        print("The vendored tree and pin are rewritten, but the ledger needs updating:")
        for error in ledger_errors:
            print(f"  {error}")
        print(f"Fix modified_paths, or discard with: {source.discard}")
        return 1
    print(
        "UPSTREAM.yaml ledger matches. Review `git status`, build the configs that use it, then commit."
    )
    return 0


def refresh(manifest_path: Path, target: str | None) -> int:
    """Re-vendor one source at a newer upstream commit, re-applying local edits."""
    source = load_source(manifest_path)
    with tempfile.TemporaryDirectory(prefix="workspace-refresh-") as temporary_dir:
        checkout = Path(temporary_dir) / "checkout"
        git(checkout.parent, "init", "--quiet", "--template=", str(checkout))
        upstream_ref = fetch_upstream(checkout, source)
        new_commit = resolve_target(checkout, source, upstream_ref, target)
        if new_commit == source.old_commit:
            print(
                f"{source.relative_dir} already pins {new_commit} on {source.branch}."
            )
            return 0
        local = capture_local_changes(checkout, source)
        git(checkout, "checkout", "--quiet", "--force", new_commit)
        conflicts = reapply_local_changes(checkout, source, local[0]) if local else []
        missing_upstream = copy_back(checkout, source, new_commit)

        print(
            f"Refreshed {source.relative_dir} from {source.old_commit[:9]} "
            f"to {new_commit[:9]} on {source.branch}."
        )
        if local:
            print(f"Re-applied local modifications to {len(local[1])} files.")
        for path in missing_upstream:
            print(
                f"WARNING: upstream {new_commit[:9]} no longer has {path}; "
                "remove it from vendored_paths or restore it."
            )
        if conflicts:
            print("Conflicts while re-applying local modifications:")
            for path in conflicts:
                print(f"  {source.relative_dir / source.snapshot_path / path}")
            print(
                "Resolve the markers, then run bin/validate_workspace_dependencies.py "
                f"--verify-upstream. The pin is already bumped; to discard: {source.discard}"
            )
            return 1
        return check_ledger(checkout, source, new_commit)


def update(source: str, target: str | None) -> int:
    try:
        manifest_path = resolve_manifest(source)
    except RefreshError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    relative_dir = manifest_path.parent.relative_to(validator.REPOSITORY_ROOT)
    try:
        return refresh(manifest_path, target)
    except RefreshError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    except OSError as error:
        print(
            f"ERROR: {error}. {relative_dir} may be partially rewritten; "
            f"to discard: {discard_command(relative_dir)}",
            file=sys.stderr,
        )
        return 1
    except subprocess.CalledProcessError as error:
        print(
            f"ERROR: git {git_subcommand(error.cmd)} failed: {last_line(error.stderr)}",
            file=sys.stderr,
        )
        return 1
    except subprocess.TimeoutExpired as error:
        print(
            f"ERROR: git {git_subcommand(error.cmd)} timed out after "
            f"{error.timeout:.0f} seconds",
            file=sys.stderr,
        )
        return 1


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
    update_parser = subparsers.add_parser(
        "update",
        help="re-vendor one source at a newer upstream commit, re-applying local edits",
    )
    update_parser.add_argument(
        "source", help="name of a directory under src/external_dependencies"
    )
    update_parser.add_argument(
        "--to",
        metavar="COMMIT",
        help="upstream commit to pin; defaults to the head of the pinned upstream branch",
    )
    arguments = argument_parser.parse_args(argv)
    if arguments.command == "update":
        return update(arguments.source, arguments.to)
    return status(markdown=arguments.markdown)


if __name__ == "__main__":
    sys.exit(main())
