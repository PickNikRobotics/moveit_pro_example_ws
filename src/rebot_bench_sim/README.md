# reBot bench simulation

A thin overlay of [`rebot_bench_base_config`](../rebot_bench_base_config/README.md),
following `so101_sim` / `so101_base_config`. It inherits the description,
controllers, Objectives and waypoints, forces `hardware_interface: mock`, and
replaces the base camera driver launch with MoveIt Pro's empty launch file.
No camera driver or simulated camera feed starts, even if the base camera YAML
is enabled. Camera attachment frames and the visual wrist bracket remain in
the shared model.

```bash
moveit_pro run --config-package rebot_bench_sim
```

Run **Move reBot to Waypoint** with **Raised** and **Zeroed Rest**, or the
**Open Gripper** / **Close Gripper** Objectives. **Teleoperate** supports arm
Joint/Pose jogging and proportional J7 Joint jogging. See the base README for
limits, controller switching, visual provenance and verification commands.
The only arm plugin is `mock_components/GenericSystem`; no real motor backend
is implemented. This is a kinematic mock, not a physics or hardware validation.

`test/test_config_inheritance.py` exercises MoveIt Pro's real configuration
loader to verify base inheritance, the mock selector and the empty camera launch.
