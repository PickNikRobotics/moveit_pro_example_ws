# MoveIt Pro Example Workspace

This workspace contains reference materials for using MoveIt Pro, including example robot configurations, simulated environments, and reusable behaviors. Most robot configurations are simulation-only, and hardware-only dependencies are intentionally excluded to reduce build time and avoid maintaining complex dependencies that the simulation examples do not use. This workspace now also includes the hardware-capable `so101_base_config`, whose driver dependency is vendored under `src/external_dependencies`.

## Cloning

Install [Git LFS](https://git-lfs.com/) before cloning so robot meshes and scene assets are checked out correctly:

```bash
sudo apt install git-lfs
git lfs install
git clone <repo-url>
```

Robot descriptions and simulation assets are vendored under `src/external_dependencies`, along with `feetech_ros2_driver`, the hardware driver `so101_base_config` uses to command real SO-101 hardware. Each vendored source has an `UPSTREAM.yaml` recording the upstream repository, the exact commit the files came from, which paths were retained, and which of them PickNik modified. No source submodules are required for simulation.

The `moveit_pro_sam2` and `moveit_pro_sam3` submodules contain optional perception models used by ML demonstration Objectives. Initialize them only when those Objectives are needed:

```bash
git submodule update --init src/moveit_pro_sam2 src/moveit_pro_sam3
```

## Robot Configs

- `april_tag_sim`
- `dual_arm_sim`
- `factory_sim`
- `grinding_sim`
- `hangar_sim`
- `kitchen_sim`
- `lab_sim`
- `lunar_sim`
- `so101_base_config`
- `so101_sim`
- `vla_sim`
- `moveit_pro_franka_configs/franka_base_config`
- `moveit_pro_kinova_configs/kinova_gen3_base_config`
- `moveit_pro_kinova_configs/kinova_sim`
- `moveit_pro_kinova_configs/space_satellite_sim`
- `moveit_pro_kinova_configs/space_satellite_sim_camera_cal`
- `moveit_pro_ur_configs/mock_sim`
- `moveit_pro_ur_configs/multi_arm_sim`
- `moveit_pro_ur_configs/picknik_ur_base_config`

The hardware-only `kinova_gen3_site_config` and `picknik_ur_site_config` configurations are not included. They bring up physical Kinova and Universal Robots hardware, respectively, while the retained base and simulation configurations provide the descriptions and interfaces needed by this workspace.

## Updating vendored dependencies

Each `UPSTREAM.yaml` under `src/external_dependencies` records the upstream repository, branch, exact commit, retained paths, and local modifications. Refresh these snapshots explicitly from the repository root. You need Python 3.10+, Git, Git LFS, network access to the configured repositories, and temporary disk space for upstream Git history, fetched LFS assets, and a staged dependency copy. Commit or stash changes in the dependencies you select first, including untracked and ignored files. Run `git lfs pull` if your checkout still contains LFS pointers instead of asset bytes.

Preview all dependencies, then apply only the one you want:

```bash
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream all --dry-run
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream feetech_ros2_driver --dry-run
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream feetech_ros2_driver
```

`feetech_ros2_driver` is the **dependency directory name** under `src/external_dependencies`, not a robot config or its dependency graph. The example may report `already current`; it is not a promise that a newer eligible tag exists. To apply every eligible refresh instead:

```bash
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream all
```

These commands fetch upstream data and change only local vendored files and the manifest's `commit` field. They never commit, push, create a PR, or update the optional ML submodules. `--dry-run` performs the same selection and merge checks without writing the snapshot. A later apply fetches tags again, so review the reported tag and commit.

### Which tag is selected?

The refresher selects the highest numeric `MAJOR.MINOR.PATCH` tag, optionally prefixed with `v`, whose commit is both reachable from the configured branch and contains the current pinned commit in its ancestry. Annotated and lightweight tags work. Prereleases, build suffixes, and other naming conventions are excluded. The version cannot be lower than the highest stable tag reachable from the current pin. Equal versions with different eligible commits are ambiguous and require manual selection.

This is deliberately stricter than choosing the newest tag by date or following branch HEAD. In particular, a tag predating PickNik fork commits cannot replace those patches. A pin already at the selected tag is unchanged. If there is no eligible tag, the dependency is left untouched with an error explaining that it needs manual review. There is **no branch-HEAD fallback**. Forks such as `main-picknik` or `ros2-fix-deps` may have no qualifying release.

### Local patches, pruning, and manual cases

For an unchanged retained-file inventory (excluding demonstrably already-pruned subtrees), the refresher uses Git's three-way text merge with the old upstream pin as the base. It preserves committed local additions and existing pruned files, honors `snapshot_path`, and keeps manifest comments and annotations byte-for-byte except for the commit field. It checks the modification ledger before and after merging, and validates the complete proposed snapshot and manifest in temporary staging before writing. Incoming license changes that need new metadata stop the refresh for manual review.

The manifest's `pruning_notes` are prose, not executable selection rules. Changes wholly inside an old upstream directory that is entirely absent locally stay pruned. Other added, deleted, renamed, or mode/type-changed upstream paths inside a retained boundary require a **manual refresh**, rather than guessing which new files to include or silently dropping files. Other manual cases include merge conflicts, symlinks, divergent local binary patches, unsupported LFS-to-ordinary-file transitions, and patches absorbed upstream that need a `modified_paths` correction. Changed upstream LFS assets can update unmodified local assets: Git LFS fetches actual bytes into temporary storage, and the refresher verifies their SHA-256 and size against the selected pointer. Missing or invalid objects stop the dependency; pointers are never installed. Unchanged bytes are not rewritten.

Clean CRLF checkout conversion is not normalized automatically. Use an LF checkout after reviewing `core.autocrlf` and text/EOL attributes, or refresh manually; do not record checkout conversion as a local patch.

A selection, merge, LFS retrieval, or proposed-validation error leaves that dependency untouched. Writes are not crash-atomic: an interruption or write-time disk failure can leave a partial update. In `all` mode, independent successful dependencies can still be refreshed; the command exits nonzero if any dependency fails. Inspect each reported result. For a manual refresh, check out the chosen upstream commit separately, preserve licenses and notices, reapply local patches and pruning, and update the manifest's commit and path declarations. Do not change branches merely to bypass a missing tag or discard fork patches.

### Review and publish the local changes

```bash
git diff --stat
git diff -- src/external_dependencies
python3 bin/validate_workspace_dependencies.py                    # offline structure check
python3 bin/validate_workspace_dependencies.py --verify-upstream  # read-only comparison with each pinned upstream commit
git diff --check
```

Review release notes, license changes, and any provenance notes that mention an older release, then build and test every robot config that consumes the changed packages. The refresher does not prove runtime compatibility. After review and validation, commit and push through your normal PR workflow, for example for a single dependency:

```bash
git add src/external_dependencies/feetech_ros2_driver
git commit -m "Refresh vendored Feetech driver dependency"
git push
```

`--verify-upstream` needs network access but does not refresh files. CI checks structure only; it does not compare vendored contents, schedule refreshes, or create upstream-verification issues. The optional ML model submodules can be advanced independently when their demonstration Objectives need a newer model package.
