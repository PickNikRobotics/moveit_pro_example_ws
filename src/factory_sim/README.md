# Factory sim config

A MoveIt Pro configuration for a Fanuc LR Mate 200iD.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## Model licensing and provenance

The ONNX model files in [`models/`](models/) are exports of Meta's SAM 2.1 model and are distributed under the [Apache License 2.0](models/LICENSE), with upstream attribution in [`models/NOTICE`](models/NOTICE). If you redistribute the models or derivative works, include the license text and the NOTICE file with them. All other files in this package are governed by the package's [BSD 3-Clause License](LICENSE).

The models were exported by PickNik in December 2025 from Meta's SAM 2.1 `sam2.1_hiera_large` checkpoint ([facebookresearch/sam2](https://github.com/facebookresearch/sam2)) using `torch.onnx.export` (PyTorch 2.6.0, per the ONNX producer metadata), split into three graphs (image encoder, prompt encoder, decoder) so image embeddings can be cached across prompts. No training or fine-tuning was performed. The export scripts and the exact checkpoint revision were not preserved.

| File | SHA-256 |
| --- | --- |
| `sam2.1_hiera_l_image_encoder.onnx` | `cbdbe1e1ad3b616985d0f9c071a5948f8f8342d8e55e98857159405678b70cec` |
| `sam2.1_prompt_encoder.onnx` | `eb2130a99f74bb2f6a430c0f1fe6af291b45ead4bf0beee715330c3def901206` |
| `sam2.1_decoder.onnx` | `e11609a8d694f2186f5974cf0d86e1588dbbb126088c4a7849adbc546b284e3a` |

## Bin poses

[`config/bin_poses.yaml`](config/bin_poses.yaml) holds exactly two YAML documents: the pick bin pose first, the place bin pose second. Positions are meters in `world`; orientations are unit quaternions in `x, y, z, w` order. Both frames must be exactly `world`, every component must be finite, and the orientation is meant to be a yaw about world z only: the drop tilt is defined relative to the bin, so a rolled or pitched bin tilts the drop pose with it. The bin origin preserves the previous `bin_tops.urdf` planning datum, 0.025 m above the MuJoCo bin body origin in `description/scene.xml`, with the rim center 0.17 m above it.

The `Add Bins to Planning Scene` Objective reads both poses and derives everything that depends on them: four rim collision boxes per bin (wall centers 0.5846 m apart along the bin x axis and 0.356 m apart along y, each wall 0.025 m thick and 0.05 m tall, so the outer footprint is 0.6096 m by 0.381 m), the perception guess and crop pose above the pick bin (crop box 0.55 m along the bin's x axis, 0.32 m along y, 0.28 m along z), and the drop pose above the place bin. Bin walls and bottom are not modeled because resting parts contact them. Bin dimensions are fixed; the configuration changes position and orientation only.

Rim IDs are `pick_bin/x_positive`, `pick_bin/x_negative`, `pick_bin/y_positive`, `pick_bin/y_negative` and the matching four `place_bin` IDs. Repeating setup overwrites those IDs and leaves other collision objects and attached objects in place. The eight scene updates are applied one at a time, so a scene service failure can leave the geometry partially updated. Reset the planning scene before adding bins when upgrading from an earlier `factory_sim` that loaded the rims from a URDF; both bracket picking Objectives already do this.

The MuJoCo bins in [`description/scene.xml`](description/scene.xml) are positioned separately. Moving the planning poses does not move the simulated bins, so update both before validating in simulation, keeping the 0.025 m z offset between the two origins. After editing the YAML, rebuild `factory_sim` so the installed copy changes, then run `Add Bins to Planning Scene` and confirm rim placement, the perception crop, and the drop target before running a pick cycle. A different package-relative YAML path can be supplied through the `configuration_file` port.
