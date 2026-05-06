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

Several submodules (notably `picknik_accessories`) use git LFS. After updating submodules, pull LFS objects:
```bash
git submodule foreach --recursive git lfs pull
```

## Repository Structure

<pre>
.
├── README.md
├── Dockerfile
├── docker-compose.yaml
├── colcon-defaults.yaml
├── scripts/                          # Behaviors Hub maintenance scripts
└── src
    ├── example_behaviors             # Reusable example C++ behaviors
    ├── lab_sim_behaviors             # Behaviors specific to lab_sim
    │
    ├── april_tag_sim                 # AprilTag detection sim
    ├── dual_arm_sim                  # Dual-arm manipulation sim
    ├── factory_sim                   # Factory environment sim
    ├── grinding_sim                  # Grinding/finishing sim
    ├── hangar_sim                    # Mobile manipulation in a hangar
    ├── kitchen_sim                   # Kitchen environment sim
    ├── lab_sim                       # UR5e on linear rail in a lab
    ├── lunar_sim                     # Lunar surface sim
    │
    ├── moveit_pro_ur_configs         # Universal Robots configurations
    │   ├── picknik_ur_base_config
    │   ├── picknik_ur_site_config
    │   ├── mock_sim
    │   └── multi_arm_sim
    ├── moveit_pro_kinova_configs     # Kinova Gen3 configurations
    │   ├── kinova_gen3_base_config
    │   ├── kinova_gen3_site_config
    │   ├── kinova_sim
    │   ├── moveit_studio_kinova_pstop_manager
    │   ├── space_satellite_sim
    │   └── space_satellite_sim_camera_cal
    ├── moveit_pro_franka_configs     # Franka configurations
    │   └── franka_base_config
    │
    ├── moveit_pro_clipseg            # CLIPSeg perception (submodule)
    ├── moveit_pro_sam2               # SAM2 perception (submodule)
    ├── moveit_pro_sam3               # SAM3 perception (submodule)
    ├── picknik_accessories           # Shared meshes, URDFs, MuJoCo assets (submodule, LFS)
    │
    └── external_dependencies
        ├── clearpath_mecanum_drive_controller   (submodule)
        ├── fanuc                                (submodule)
        ├── franka_config/franka_description     (submodule)
        ├── phoebe_ws                            (submodule)
        ├── ros2_kortex                          (submodule)
        ├── ros2_kortex_vision                   (submodule)
        ├── ur_description                       (submodule)
        ├── ridgeback
        ├── ros2_robotiq_gripper
        └── serial
</pre>

## Updating Submodules

To pull the latest commits for all submodules:
```bash
git submodule update --remote --recursive
git submodule foreach --recursive git lfs pull
```
