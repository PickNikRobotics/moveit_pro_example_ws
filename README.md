# MoveIt Pro Example Workspace

This workspace contains reference materials for using MoveIt Pro, including example robot configurations, simulated environments, and reusable behaviors.

## Cloning

This repository uses git submodules. Clone with:
```bash
git clone --recurse-submodules <repo-url>
```

If you already cloned without submodules, initialize them with:
```bash
git submodule update --recursive --init
```

Several submodules (notably `picknik_accessories`) use git LFS. Install [git-lfs](https://git-lfs.com/) first (e.g., `sudo apt install git-lfs && git lfs install`); without it the commands below fail with `git: 'lfs' is not a git command`. After updating submodules, pull LFS objects:
```bash
git submodule foreach --recursive git lfs pull
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
- `moveit_pro_kinova_configs/kinova_gen3_site_config`
- `moveit_pro_kinova_configs/kinova_sim`
- `moveit_pro_kinova_configs/space_satellite_sim`
- `moveit_pro_kinova_configs/space_satellite_sim_camera_cal`
- `moveit_pro_ur_configs/mock_sim`
- `moveit_pro_ur_configs/multi_arm_sim`
- `moveit_pro_ur_configs/picknik_ur_base_config`
- `moveit_pro_ur_configs/picknik_ur_site_config`
- `so101_moveit_config`
- `so101_moveit_hardware_config`

## SO-101 VLA workspace

This fork includes a single-follower SO-101 configuration for MuJoCo
simulation and a physical arm whose Feetech hardware driver, controller
manager, and standard ros2_control controllers run on its Raspberry Pi.
MoveIt Pro connects through a directionally filtered ROS2DDS bridge; the same
stable camera topics and VLA Objectives are available in simulation and
hardware. See
[the SO-101 workspace guide](docs/so101/README.md).

## Updating Submodules

To pull the latest commits for all submodules:
```bash
git submodule update --remote --recursive
git submodule foreach --recursive git lfs pull
```
