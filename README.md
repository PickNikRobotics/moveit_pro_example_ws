# MoveIt Pro Example Workspace

This is fork of the [MoveIt Pro Empty Workspace](https://github.com/PickNikRobotics/moveit_pro_empty_ws).
This workspace contains reference materials for using MoveIt Pro, including:
- [Example base UR5e configuration](src/moveit_pro_ur_configs/picknik_ur_base_config)
- [A physics based simulation environment with a robot on a linear rail](src/lab_sim)
- [Mobile manipulation configuration](src/hangar_sim)
- [Example behaviors](src/example_behaviors)

Since the [picknik_accessories](https://github.com/PickNikRobotics/picknik_accessories) package uses git LFS, [it cannot be added as a subtree](https://github.com/git-lfs/git-lfs/issues/854).
Please ensure you have the submodule up to date using:
```bash
git submodule update --recursive --init
```

## Working with Git Subtrees

This repository was created through the combination of multiple repositories using git subtree.
If you have no interest in manually pulling or pushing upstream changes, you can ignore the following section and treat this repository as a single repository.

### Repository Structure


The structure of this repository is as follows:

<pre>
.
├── <a href="README.md">README.md</a>
└── src
    ├── <a href="https://github.com/PickNikRobotics/example_behaviors">example_behaviors</a>
    ├── <a href="https://github.com/PickNikRobotics/lab_sim">lab_sim</a>
    ├── <a href="https://github.com/PickNikRobotics/moveit_pro_ur_configs">moveit_pro_ur_configs</a>
    │   ├── picknik_ur_base_config
    │   ├── mock_sim
    │   ├── multi_arm_sim
    │   ├── picknik_ur_sim_config
    │   └── picknik_ur_site_config
    ├── <a href="https://github.com/PickNikRobotics/moveit_pro_kinova_configs">moveit_pro_kinova_configs</a>
    │   ├── kinova_gen3_base_config
    │   ├── kinova_sim
    │   └── moveit_studio_kinova_pstop_manager
    ├── <a href="https://github.com/PickNikRobotics/moveit_pro_mobile_manipulation">moveit_pro_mobile_manipulation</a>
    │   ├── mobile_manipulation_config
    │   └── picknik_ur_mobile_config
    ├── <a href="https://github.com/PickNikRobotics/fanuc_sim">fanuc_sim</a>
    ├── <a href="https://github.com/PickNikRobotics/picknik_accessories">picknik_accessories</a> (submodule)
    └── external_dependencies
        ├── <a href="https://github.com/sjahr/ridgeback/tree/ros2">ridgeback</a>
        ├── <a href="https://github.com/PickNikRobotics/ros2_robotiq_gripper">ros2_robotiq_gripper</a>
        └── <a href="https://github.com/tylerjw/serial/tree/ros2">serial</a>
</pre>

This repository contains a **copy** of the git repositories that were added as subtrees.
File changes and commits are treated as if they happen only in this repository.
If you update the contents of a subtree, you can merge the latest `main` branch of [lab_sim](https://github.com/PickNikRobotics/lab_sim) using the following command:
```bash
git subtree pull --prefix src/lab_sim https://github.com/PickNikRobotics/lab_sim main --squash
```

To pull the upstream changes to all subtrees and submodules, a convenience script is provided.
From the top level, you can execute:
```bash
./sync_subtrees.sh
```

## AprilTag Detection Analysis

### Data collection steps:

1. To run the objectives for this detection analysis, a behavior update to the SaveToYaml Behavior is required that will
   be released in MoveIt Pro version 8.4. If you are on a version older than 8.4.0 (which is unreleased as of this
   writing), then you will need to manually pull a more recent version with the changes. To do this, once you have the
   latest released version of MoveIt Pro [installed](https://docs.picknik.ai/software_installation/), edit the
   `/opt/moveit_pro/.version` file on your host machine and replace the version name with
   `save-to-yaml-dont-overwrite-if-namespaced`.
1. Make sure the `apriltag-detection-testing` branch is checked out in your `moveit_pro_exmple_ws` workspace.
1. Rebuild Pro with `moveit_pro build`.
1. Launch moveit_pro using lab_sim with `moveit_pro run -c lab_sim`.
1. Run the `AprilTag Detection Testing 0` objective which collects 10 samples of each of the 6 AprilTags with the camera
   parallel to the tags.
1. Once complete, move the YAML files with the results to a new directory in the objectives folder called
   `0degree_runs`: `mv tag*.yaml 0degree_runs/.`.
1. Run the `AprilTag Detection Testing 45` objective which collects 10 samples of each of the 6 AprilTags with the camera
   at a 45 degree angle to the tags.
1. Once complete, move the YAML files with the results to a new directory in the objectives folder called
   `45degree_runs`: `mv tag*.yaml 45degree_runs/.`.

### Data analysis steps:

Simply run the `analyze_apriltag_detection.py` script.
