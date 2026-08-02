# Train

Recipe for the checkpoint served by `docker/serve_policy.py`.

## Combine the recordings

Each prompt is recorded as its own Pro dataset. Merge them into the one dataset
training reads, under `~/.local/share/moveit_pro/trainer/recordings`:

```bash
uv run python combine_datasets.py <out> <dataset>-lerobot <dataset>-lerobot ...
```

This also backfills the quantile statistics the Trainer's converter leaves out,
which pi0.5 needs to normalize state and action. Training a single unmerged
dataset needs that backfill on its own:

```bash
uv run python backfill_stats.py <dataset>-lerobot
```

## LeRobot Train

```bash
uv sync
uv run lerobot-train \
  --config_path=pi05_colorstack_lora_backbone.yaml \
  --output_dir=/path/to/checkpoints/my_run \
  --job_name=my_run
```

Needs `HF_TOKEN` in the environment and a GPU with 40+ GB. On smaller GPUs, add
`--policy.gradient_checkpointing=true`.

## Merge, then serve

`lerobot-train` saves a LoRA adapter; `serve_policy.py` needs a dense checkpoint:

```bash
uv run python merge_lora_checkpoint.py <run>/checkpoints/<step>/pretrained_model <out>_merged
docker compose run --rm --service-ports serve_policy --checkpoint <out>_merged --policy-class pi05
```

## Gotcha: `--policy.*` overrides need `--policy.path`

Otherwise the run fails with `DecodingError: policy: Expected a dict with a 'type' key`.
