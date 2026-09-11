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

## CLIPSeg models are no longer bundled

Starting with the 9.4 patch release that includes this change, this workspace
no longer vendors the `moveit_pro_clipseg` submodule.
The `GetMasks2DFromTextQuery` Behavior still exists in MoveIt Pro 9.4, but the
example configs no longer ship weights for it, so the following Objectives fail
at run time with

    The ONNX model path could not be resolved: Package 'moveit_pro_clipseg' was not found

- kitchen_sim: Segment Image from Prompt
- lab_sim: ML Segment Image, ML Segment Image Loop, ML Segment Point Cloud,
  AddBottlesToPlanningScene
- hangar_sim: Segment Image from Text Prompt, ML Move Boxes to Loading Zone,
  Move Boxes Looping

To restore them, build a ROS package that installs CLIP and CLIPSeg ONNX models
to `share/<your_package>/models/` and set `model_package` (and the
`clip_model_path` / `clipseg_model_path` ports) on those Objectives to point at
it. To move off CLIPSeg entirely, see the SAM3 equivalents in `lab_sim`
(`ML Find Objects on Table`, `ML Segment Bottles from File`), `dual_arm_sim`
(`Find Red Block`, `Find Green Block`, `Sort Blocks`), and MoveIt Pro 10.0,
where these Objectives were migrated to `GetMasks2DFromExemplar`.

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

## Updating Submodules

To pull the latest commits for all submodules:
```bash
git submodule update --remote --recursive
git submodule foreach --recursive git lfs pull
```
