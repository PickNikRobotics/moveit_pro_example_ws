# Train

Recipe for the checkpoint the [inference server](../docker/README.md) serves,
starting from datasets recorded by the `Record Cube-Stack ...` objectives.

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
  --config_path=pi05_cubestack_lora_backbone.yaml \
  --output_dir=/path/to/checkpoints/my_run \
  --job_name=my_run
```

Needs `HF_TOKEN` in the environment and a GPU with 40+ GB. A 24 GB card fits with
gradient checkpointing, at 14.7 GB and ~1.1 step/s:

```bash
  --policy.path=lerobot/pi05_base --policy.gradient_checkpointing=true
```

Any `--policy.*` override has to be accompanied by `--policy.path`, as above.

## Merge, then serve

`lerobot-train` saves a LoRA adapter; the inference server needs a dense
checkpoint:

```bash
uv run python merge_lora_checkpoint.py <run>/checkpoints/<step>/pretrained_model <out>_merged
```

Drop `<out>_merged` into [`../models/`](../models/) and point
[`../config/vla_serving.yaml`](../config/vla_serving.yaml) at
`/models/<out>_merged`, then restart the `inference_server` container.
