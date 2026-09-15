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

Each `UPSTREAM.yaml` under `src/external_dependencies` records the exact upstream commit and retained paths. To refresh one: check the tree out at the new commit, preserve its license files, reapply the pruning described in `pruning_notes`, and validate every config that consumes the package.

Then update `commit` and the retained-path lists, and check the result:

```bash
python3 bin/validate_workspace_dependencies.py                    # structure, runs on every PR
python3 bin/validate_workspace_dependencies.py --verify-upstream  # fetches the pinned commit and compares files
```

The second command needs network access. Run it manually after re-vendoring. CI checks structure only; it does not compare vendored contents or create upstream-verification issues.

The optional ML model submodules can be advanced independently when their demonstration Objectives need a newer model package.

### Optional quick refresh

For eligible upstream releases, the command below is a quicker alternative to the manual workflow above.

Run from the repository root with Python 3.10+, Git, Git LFS, network access, and space for temporary upstream copies. Selected dependencies must have no uncommitted, untracked, or ignored files. Run `git lfs pull` first if assets are still LFS pointers.

Preview all dependencies, then refresh one:

```bash
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream all --dry-run
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream feetech_ros2_driver --dry-run
python3 bin/validate_workspace_dependencies.py --refresh-from-upstream feetech_ros2_driver
```

Use a directory name under `src/external_dependencies`, not a robot config. Replace it with `all` to refresh every eligible dependency. `--dry-run` checks without writing; apply fetches tags again, so check the reported tag and commit. These commands never commit, push, create a PR, or update optional ML submodules.

The selected release is the highest stable `MAJOR.MINOR.PATCH` tag (optional `v` prefix) reachable from the branch in `UPSTREAM.yaml` and containing the current pin. Prereleases, build suffixes, downgrades, and ambiguous versions are rejected. There is no branch-HEAD fallback: forks without an eligible tag need manual review.

Local patches and pruning are preserved where they can be merged safely. Conflicts, uncertain file selection, unsupported binary changes, and required license or manifest corrections stop that dependency before writing. Follow the reported diagnostic rather than bypassing it. In `all` mode, other dependencies can still succeed; any failure returns a nonzero exit code. An interruption during writing can leave a partial update.

Review and validate the changes:

```bash
git diff -- src/external_dependencies
python3 bin/validate_workspace_dependencies.py                    # offline structure check
python3 bin/validate_workspace_dependencies.py --verify-upstream  # read-only network comparison
git diff --check
```

Review release notes and licenses, update stale provenance notes, and test every robot config that uses the changed packages. Then commit and push, for example:

```bash
git add src/external_dependencies/feetech_ros2_driver
git commit -m "Refresh vendored Feetech driver dependency"
git push
```

CI checks structure only. Upstream comparison and refresh are user-triggered.
