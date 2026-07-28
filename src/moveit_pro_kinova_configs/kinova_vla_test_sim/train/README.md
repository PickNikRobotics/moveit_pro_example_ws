# Train

Recipe for the checkpoint served by `docker/serve_policy.py`.

## LeRobot Train

```bash
uv sync
uv run lerobot-train \
  --config_path=pi05_colorstack_lora_backbone.yaml \
  --output_dir=/path/to/checkpoints/my_run \
  --job_name=my_run
```

Needs `HF_TOKEN` in the environment and a GPU with 40+ GB. On smaller GPUs, add
`--policy.path=lerobot/pi05_base --policy.gradient_checkpointing=true`.

## Merge, then serve

`lerobot-train` saves a LoRA adapter; `serve_policy.py` needs a dense checkpoint:

```bash
uv run python merge_lora_checkpoint.py <run>/checkpoints/<step>/pretrained_model <out>_merged
docker compose run --rm --service-ports serve_policy --checkpoint <out>_merged --policy-class pi05
```

## Gotcha: `--policy.*` overrides need `--policy.path`

Otherwise the run fails with `DecodingError: policy: Expected a dict with a 'type' key`.
