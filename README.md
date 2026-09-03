# MoveIt Pro Example Workspace

This simulation-only workspace contains reference materials for using MoveIt Pro, including example robot configurations, simulated environments, and reusable behaviors. Hardware robot configurations, drivers, and hardware-only dependencies are intentionally excluded to reduce build time and avoid maintaining complex dependencies that the simulation examples do not use.

## Cloning

Install [Git LFS](https://git-lfs.com/) before cloning so robot meshes and scene assets are checked out correctly:

```bash
sudo apt install git-lfs
git lfs install
git clone <repo-url>
```

Robot descriptions and simulation assets are vendored under `src/external_dependencies`. Each vendored source has an `UPSTREAM.yaml` recording the upstream repository, the exact commit the files came from, which paths were retained, and which of them PickNik modified. No source submodules are required for simulation.

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

Each `UPSTREAM.yaml` under `src/external_dependencies` records the exact upstream commit and retained paths. To refresh one: check the tree out at the new commit, preserve its license files, reapply the pruning described in `pruning_notes`, and validate every config that consumes the package.

Then update `commit` and the retained-path lists, and check the result:

```bash
python3 bin/validate_workspace_dependencies.py                    # structure, runs on every PR
python3 bin/validate_workspace_dependencies.py --verify-upstream  # fetches the pinned commit and compares files
```

The second one needs network access. CI runs it weekly rather than per PR, so run it yourself after a re-vendor.

The optional ML model submodules can be advanced independently when their demonstration Objectives need a newer model package.
