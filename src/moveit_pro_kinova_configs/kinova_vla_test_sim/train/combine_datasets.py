"""Combine the per-prompt recordings into one dataset to train on.

Each prompt is recorded as its own Pro dataset. Training wants them as one,
with the prompts unified into a single task table:

    uv run python combine_datasets.py <out-dir> <dataset-dir> <dataset-dir> ...

Statistics are backfilled into each source first. Aggregation pools the
per-dataset statistics rather than recomputing them, so a source still missing
its quantiles would carry that gap into the merged dataset.
"""

import sys
from pathlib import Path

from lerobot.datasets.aggregate import aggregate_datasets

from backfill_stats import backfill


def combine(output: Path, sources: list[Path]) -> None:
    """Backfill each source's statistics, then merge them into `output`."""
    for source in sources:
        backfill(source)
    aggregate_datasets(
        repo_ids=[f"local/{source.name}" for source in sources],
        aggr_repo_id=f"local/{output.name}",
        roots=sources,
        aggr_root=output,
    )
    print(f"combined {len(sources)} dataset(s) into {output}")


def main() -> int:
    args = [Path(arg).expanduser() for arg in sys.argv[1:]]
    if len(args) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    output, sources = args[0], args[1:]
    for source in sources:
        if not (source / "meta" / "info.json").is_file():
            print(f"not a LeRobot dataset: {source}", file=sys.stderr)
            return 1
    if output.exists():
        print(f"output already exists: {output}", file=sys.stderr)
        return 1
    combine(output, sources)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
