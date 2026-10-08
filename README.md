# MoveIt Pro Example Workspace

This workspace contains reference materials for using MoveIt Pro, including example robot configurations, simulated environments, and reusable behaviors. Most robot configurations are simulation-only, and hardware-only dependencies are intentionally excluded to reduce build time and avoid maintaining complex dependencies that the simulation examples do not use. This workspace now also includes the hardware-capable `so101_base_config`, whose driver dependency is vendored under `src/external_dependencies`.

## Cloning

Install [Git LFS](https://git-lfs.com/) before cloning so robot meshes and scene assets are checked out correctly:

```bash
sudo apt install git-lfs
git lfs install
git clone <repo-url>
```

Robot descriptions and simulation assets are vendored under `src/external_dependencies`, along with `feetech_ros2_driver`, the hardware driver `so101_base_config` uses to command real SO-101 hardware; each vendored source has an `UPSTREAM.yaml` file recording its repository, commit, and pruned paths. No source submodules are required for simulation.

The `moveit_pro_sam2` submodule contains an optional perception model used by ML demonstration Objectives. Initialize it only when those Objectives are needed:

```bash
git submodule update --init src/moveit_pro_sam2
```

The `moveit_pro_sam3` package is part of this repository but contains no model files. Its build downloads the SAM3 ONNX files; see `src/moveit_pro_sam3/README.md`.

## Robot Configs

- `april_tag_sim`
- `dual_arm_sim`
- `factory_sim`
- `grinding_sim`
- `hand_eye_calibration_sim`
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
- `moveit_pro_ur_configs/mock_sim`
- `moveit_pro_ur_configs/multi_arm_sim`
- `moveit_pro_ur_configs/picknik_ur_base_config`

The hardware-only `kinova_gen3_site_config` and `picknik_ur_site_config` configurations are not included. They bring up physical Kinova and Universal Robots hardware, respectively, while the retained base and simulation configurations provide the descriptions and interfaces needed by this workspace.

## Updating vendored dependencies

Each `UPSTREAM.yaml` file under `src/external_dependencies` records the exact upstream commit and retained paths. Run `bin/vendored_dependency.py status` to see how many commits each pinned upstream branch has moved past its recorded commit; CI publishes the same table in the job summary of the `Validate workspace dependencies` job. To refresh a dependency by hand, fetch upstream at the new commit, copy the retained paths in, preserve its license files, reapply the documented pruning and local edits, update `commit:` in `UPSTREAM.yaml`, and validate every config that consumes the package. `bin/vendored_dependency.py update <source>` (optionally `--to <commit>`) does the same steps for one source: it re-vendors the retained paths at the new commit, carries over every local difference from the pinned commit (edits and pruned files alike) as a three-way merge, updates `commit:`, and then runs the manifest checks (`modified_paths` ledger and license policy). It refuses to start while the source directory has uncommitted, untracked, or ignored files, so the result can always be discarded with `git restore` and `git clean`. Conflicts are left as ordinary conflict markers to resolve by hand. A run that stops early has already rewritten the source and its pin; the message names the discard command. Refresh one source per PR, and build and run the configs that consume it before committing. Every Sunday CI runs `update` for each drifted source and opens or updates a draft PR per source on the `vendored-refresh/<source>` branch; an update that stops on conflicts or a needed `UPSTREAM.yaml` edit still opens its PR, with the conflict markers and the update output, for a person to finish.

The `moveit_pro_sam2` submodule can be advanced independently when its demonstration Objectives need a newer model package.
