# hangar_sim

A MoveIt Pro MuJoCo simulation for PickNik's Universal Robots (UR) arms.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

On Jazzy, navigation uses nav2's MPPI controller with `open_loop: true` (see `params/nav2_params_jazzy.yaml`), which needs `nav2_mppi_controller` 1.3.13 or newer; the workspace `Dockerfile` installs that build. Older builds ignore the parameter without error and drive the base well below `vx_max`. Humble runs `params/nav2_params.yaml` unchanged: its nav2 1.1.20 predates the parameter, so there is nothing to set and the `Dockerfile` skips the install there. The lag was only observed on Jazzy.
