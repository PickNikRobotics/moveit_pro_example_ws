# Factory sim config

A MoveIt Pro configuration for a Fanuc LR Mate 200iD.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## SAM2 model bundle

The auto-mask Objectives use the optional `moveit_pro_sam2` package and its shared `models/sam2_bundle.yaml` manifest. Initialize the workspace's `src/moveit_pro_sam2` submodule and build that package before running these Objectives. The runtime must support the `moveit_pro.sam2.v1` bundle ABI.

Model provenance, graph hashes, and redistribution notices are maintained in `src/moveit_pro_sam2/README.md`. This package is governed by its [BSD 3-Clause License](LICENSE).

## Bin poses

[`config/bin_poses.yaml`](config/bin_poses.yaml) holds exactly two YAML documents: the pick bin pose first, the place bin pose second. Positions are meters in `world`; orientations are unit quaternions in `x, y, z, w` order. Both frames must be exactly `world`, every component must be finite, and the orientation is meant to be a yaw about world z only: the drop tilt is defined relative to the bin, so a rolled or pitched bin tilts the drop pose with it. The bin origin preserves the previous `bin_tops.urdf` planning datum, 0.025 m above the MuJoCo bin body origin in `description/scene.xml`, with the rim center 0.17 m above it.

The `Add Bins to Planning Scene` Objective reads both poses and derives everything that depends on them: four rim collision boxes per bin (wall centers 0.5846 m apart along the bin x axis and 0.356 m apart along y, each wall 0.025 m thick and 0.05 m tall, so the outer footprint is 0.6096 m by 0.381 m), the perception guess and crop pose above the pick bin (crop box 0.55 m along the bin's x axis, 0.32 m along y, 0.28 m along z), and the drop pose above the place bin. Bin walls and bottom are not modeled because resting parts contact them. Bin dimensions are fixed; the configuration changes position and orientation only.

Rim IDs are `pick_bin/x_positive`, `pick_bin/x_negative`, `pick_bin/y_positive`, `pick_bin/y_negative` and the matching four `place_bin` IDs. Repeating setup overwrites those IDs and leaves other collision objects and attached objects in place. The eight scene updates are applied one at a time, so a scene service failure can leave the geometry partially updated. Reset the planning scene before adding bins when upgrading from an earlier `factory_sim` that loaded the rims from a URDF; both bracket picking Objectives already do this.

The MuJoCo bins in [`description/scene.xml`](description/scene.xml) are positioned separately. Moving the planning poses does not move the simulated bins, so update both before validating in simulation, keeping the 0.025 m z offset between the two origins. After editing the YAML, rebuild `factory_sim` so the installed copy changes, then run `Add Bins to Planning Scene` and confirm rim placement, the perception crop, and the drop target before running a pick cycle. A different package-relative YAML path can be supplied through the `configuration_file` port.
