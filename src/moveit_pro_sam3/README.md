# moveit_pro_sam3

ROS package that provides SAM 3 ONNX models for image segmentation to MoveIt Pro Behaviors.

## Usage

This repository does not contain the model files. Building the package downloads them from the upstream release listed under [Provenance](#provenance), checks each file against its SHA-256, and installs them to `share/moveit_pro_sam3/models`. A file already in the build directory with the expected hash is not downloaded again.

If you decline Meta's SAM License when `moveit_pro` asks, it writes a `COLCON_IGNORE` into this package, so the build skips it and downloads nothing.

To build without network access, download the four release assets into one directory, then pass that directory to the build:

```bash
colcon build --packages-select moveit_pro_sam3 --cmake-args -DSAM3_MODEL_URL=file:///path/to/directory
```

## Model license and use restrictions

The ONNX model files this package downloads are exports of Meta's SAM 3 model and are distributed under the [SAM License](models/LICENSE). By using or redistributing these models, you agree to that license.

The SAM License includes restrictions that downstream users must follow. In particular:

- Do not use the SAM Materials for military or warfare purposes, activities subject to the International Traffic in Arms Regulations (ITAR), nuclear industries or applications, espionage, or the development or use of guns or illegal weapons.
- Comply with applicable trade controls, privacy laws, and data-protection laws.
- Redistribute the models and derivative works only under the SAM License and include a copy of the license.
- Acknowledge the use of the SAM Materials when publishing research results produced with them.

This summary is provided for convenience and does not replace the full [SAM License](models/LICENSE).

## Provenance

The downloaded files are the assets of [mamoll/sam3-onnx, release `v1.0.0`](https://github.com/mamoll/sam3-onnx/releases/tag/v1.0.0), installed under their release names. They are ONNX exports of Meta's SAM 3 model, and the release repository includes the SAM License. PickNik performed no training or fine-tuning.

| File | SHA-256 |
| --- | --- |
| `sam3_decoder.onnx` | `4ae7cca96889f1c72063cdb5cb9115d240ce1c8e5d2d4dbd389bc73c7d3f68b9` |
| `sam3_geometry_encoder.onnx` | `fde841d45a0bf890d402236632e9dcaa6e5fd19e3c40fc48fe8cd1c6911f34eb` |
| `sam3_text_encoder.onnx` | `a6cef9ee22aa9d165666b2753cc59a201fa90b6a399e1cd3f3e0a1e8ec38ac12` |
| `sam3_vision_encoder.onnx` | `4e4de058c8566469b1aa39f81b67dcba8c9f4161cbd60b84b3d5655f452ffb2e` |

## Package licensing

The downloaded ONNX model files, and derivative works of those models, are governed by the [SAM License](models/LICENSE). All other files in this package are governed by the [BSD 3-Clause License](LICENSE), unless a file states otherwise.
