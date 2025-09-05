# FANUC CRX config

A MoveIt Pro configuration for a FANUC CRX robot.

To change the model of robot, simply change the `robot_model` urdf parameter  in the `config.yaml` file to a model described in `fanuc_crx_description`.

This config is for running Moveit Pro with hardware, however it can also be run with mock hardware by setting the `use_mock` urdf parameter to `"true"` in the `config.yaml`.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
