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
