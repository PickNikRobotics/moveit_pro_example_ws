# kinova_vla_test_sim

A MoveIt Pro configuration to simulate a Kinova Gen3 (7-DoF) + Robotiq 2F-85 stacking colored cubes in MuJoCo, for testing VLA policy execution.

- `train/`: how the pi0.5 checkpoint is trained, as a config file plus a pinned environment
- `docker/`: how that checkpoint is served to the `Execute Color-Stack Policy` objective

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
