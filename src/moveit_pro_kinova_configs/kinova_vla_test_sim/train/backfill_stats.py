"""Add the quantile statistics a Pro-recorded dataset is missing.

The Trainer's converter records only min/max/mean/std/count for state and
action. Policies that normalize by quantile -- pi0.5 maps both that way --
read q01/q99 and cannot train without them.

LeRobot recomputes these from the written parquet in well under a second, so
run this once per converted dataset, before training or before merging:

    uv run python backfill_stats.py <dataset-dir> [<dataset-dir> ...]

Image statistics stay absent. LeRobot has no recompute path for them, and
pi0.5 normalizes images with IDENTITY, so nothing reads them -- but the
training config must then set `use_imagenet_stats: false`, or LeRobot fails
writing ImageNet constants into a camera entry that does not exist.
"""

import sys
from pathlib import Path

from lerobot.datasets.dataset_tools import recompute_stats
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def backfill(root: Path) -> None:
    """Recompute `root`'s statistics in place."""
    dataset = LeRobotDataset(repo_id=f"local/{root.name}", root=root)
    before = sorted(dataset.meta.stats.get("action", {}))
    dataset = recompute_stats(dataset)
    after = sorted(dataset.meta.stats.get("action", {}))
    print(f"{root.name}: action stats {before} -> {after}")


def main() -> int:
    roots = [Path(arg).expanduser() for arg in sys.argv[1:]]
    if not roots:
        print(__doc__, file=sys.stderr)
        return 2
    for root in roots:
        if not (root / "meta" / "info.json").is_file():
            print(f"not a LeRobot dataset: {root}", file=sys.stderr)
            return 1
        backfill(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
