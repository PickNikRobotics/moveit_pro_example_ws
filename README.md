# MoveIt Pro Example Workspace

This simulation-only workspace contains reference materials for using MoveIt Pro, including example robot configurations, simulated environments, and reusable behaviors. Hardware robot configurations, drivers, and hardware-only dependencies are intentionally excluded to reduce build time and avoid maintaining complex dependencies that the simulation examples do not use.

## Cloning

Install [Git LFS](https://git-lfs.com/) before cloning so robot meshes and scene assets are checked out correctly:

```bash
sudo apt install git-lfs
git lfs install
git clone <repo-url>
```

Robot descriptions and simulation assets are vendored under `src/external_dependencies`; each vendored source has an `UPSTREAM.yaml` file recording its repository, commit, and pruned paths. No source submodules are required for simulation.

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
- `phoebe_sim`
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

Each `UPSTREAM.yaml` file under `src/external_dependencies` records the exact upstream commit and retained paths. Run `bin/vendored_dependency.py status` to see how many commits each pinned upstream branch has moved past its recorded commit; CI publishes the same table in the job summary of the `Validate workspace dependencies` job. Run `bin/vendored_dependency.py update <source>` (optionally `--to <commit>`) to refresh one source: it re-vendors the retained paths at the new commit, carries over every local difference from the pinned commit (edits and pruned files alike) as a three-way merge, updates `commit:` in `UPSTREAM.yaml`, and then checks that `modified_paths` still matches. Conflicts are left as ordinary conflict markers to resolve by hand. A run that stops early has already rewritten the source and its pin; the message names the `git restore` command that discards it. Refresh one source per PR, and build and run the configs that consume it before committing.

The optional ML model submodules can be advanced independently when their demonstration Objectives need a newer model package.
