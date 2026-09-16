# moveit_pro_sam3

ROS package that provides SAM 3 ONNX models for image segmentation to MoveIt Pro Behaviors.

## Usage

This repository does not contain the model files. Building the package downloads them from the upstream release listed under [Provenance](#provenance), checks each file against its SHA-256, and installs them to `share/moveit_pro_sam3/models`. A file already in the build directory with the expected hash is not downloaded again.

If you decline Meta's SAM License when `moveit_pro` asks, it writes a `COLCON_IGNORE` into this package, so the build skips it and downloads nothing.

To build without network access, download the four upstream assets into one directory under their upstream names, then pass that directory to the build:

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

The downloaded files are the `*-q4f16.onnx` assets from [jamjamjon/assets, release `sam3`](https://github.com/jamjamjon/assets/releases/tag/sam3), renamed as listed below. They are third-party ONNX exports of Meta's SAM 3 model with q4f16 weight quantization (4-bit weights, float16 activations), produced for the [usls](https://github.com/jamjamjon/usls) project. PickNik performed no training, fine-tuning, or other modification.

| File | Upstream asset | SHA-256 |
| --- | --- | --- |
| `sam3_decoder.onnx` | `decoder-q4f16.onnx` | `8496d685a950604626c3e6b972c21f5b4b876b15ce98b9b899875f392947b83c` |
| `sam3_geometry_encoder.onnx` | `geometry-encoder-q4f16.onnx` | `6717167a4454e063ab71895b5c03d067ce7386437fb489d1484feb2ca3abf741` |
| `sam3_text_encoder.onnx` | `text-encoder-q4f16.onnx` | `639ba5a9991b012d3290fbba27eb8dbcebdc86ed9f747572d7bea1678317d8d1` |
| `sam3_vision_encoder.onnx` | `vision-encoder-q4f16.onnx` | `9c00c5db8739f4c0cd0158a916f0e9755e0240bb35abca2934ab308e72cb2d6e` |

## Package licensing

The downloaded ONNX model files, and derivative works of those models, are governed by the [SAM License](models/LICENSE). All other files in this package are governed by the [BSD 3-Clause License](LICENSE), unless a file states otherwise.
