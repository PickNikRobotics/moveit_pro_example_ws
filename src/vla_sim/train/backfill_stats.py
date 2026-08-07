# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Add the quantile statistics a Pro-recorded dataset is missing.

The Trainer's converter records only min/max/mean/std/count for state and
action. Policies that normalize by quantile -- pi0.5 maps both that way --
read q01/q99 and cannot train without them.

LeRobot recomputes these from the written parquet in well under a second, so
run this once per converted dataset, before training or before merging:

    uv run python backfill_stats.py <dataset-dir> [<dataset-dir> ...]

This writes state and action statistics. pi0.5 normalizes images with IDENTITY,
so set `use_imagenet_stats: false` in the training config to match.
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
