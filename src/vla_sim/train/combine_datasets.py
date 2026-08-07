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

"""Combine the per-prompt recordings into one dataset to train on.

Each prompt is recorded as its own Pro dataset. Training wants them as one,
with the prompts unified into a single task table:

    uv run python combine_datasets.py <out-dir> <dataset-dir> <dataset-dir> ...

Aggregation pools the per-dataset statistics, so each source is backfilled
first and the merged dataset inherits a complete set.
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
