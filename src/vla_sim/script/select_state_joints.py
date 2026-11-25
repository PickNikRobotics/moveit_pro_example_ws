#!/usr/bin/env python3

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

"""Copy a LeRobot v3 dataset, keeping only the named joints of `observation.state`.

Trainer records every joint in `/joint_states`, so a `vla_sim` dataset's state
also carries the Robotiq linkage joints. This writes a new dataset whose state
holds only the selected joints (by default the `action` joints, in `action`
order), so its state feature matches the published cube-stacking dataset's. The
state column, `meta/info.json`, `meta/stats.json` and the per-episode statistics
in `meta/episodes` are narrowed to the kept joints, using the existing per-joint
statistics. Everything else is copied unchanged; videos are hard-linked when
possible.

With `--like <reference>`, the copy is also made mergeable with the reference
dataset by LeRobot's `aggregate_datasets`, which requires identical feature
metadata. After checking that both datasets have the same fps, robot type,
feature keys, and per-feature dtype, shape and names, it takes each non-video
feature's metadata from the reference (they may differ only in a per-feature
`fps` equal to the dataset fps), and rebuilds each video feature's `info` block
with the reference's keys: stream properties are checked against the probed
mp4s, and encoder settings, which a file does not record, are written as null.
Quantile statistics the reference has and the source lacks are computed from
the data.
"""

import argparse
import copy
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
from typing import Callable, Sequence

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

STATE_KEY = "observation.state"
EPISODE_STATS_PREFIX = f"stats/{STATE_KEY}/"
# Statistics that hold one value for the whole feature rather than one per joint.
SCALAR_STATS = {"count"}
# LeRobot names a quantile statistic `q<percent>`, for example `q01` for the 1st percentile.
QUANTILE_STAT = re.compile(r"q(\d{2})")
# `info` keys describing how a video was encoded (LeRobot's `VIDEO_ENCODER_INFO_KEYS`). An
# mp4 does not record them, and LeRobot's merge ignores them.
VIDEO_ENCODER_INFO_KEYS = {
    f"video.{name}"
    for name in ("g", "crf", "preset", "fast_decode", "extra_options", "video_backend")
}
# `info` keys `probe_video` reads from an mp4.
PROBED_KEYS = {
    "video.codec",
    "video.pix_fmt",
    "video.fps",
    "video.height",
    "video.width",
    "video.channels",
    "has_audio",
}
# Dataset-level `info.json` keys LeRobot requires to match before merging or loading.
MERGE_INFO_KEYS = ("codebase_version", "fps", "robot_type")


class DatasetError(ValueError):
    """The source dataset cannot be narrowed as requested."""


def _read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _write_json(path: Path, content: dict) -> None:
    with path.open("w", encoding="utf-8") as stream:
        json.dump(content, stream, indent=4, ensure_ascii=False)
        stream.write("\n")


def _feature_names(info: dict, key: str) -> list[str]:
    feature = info.get("features", {}).get(key)
    if feature is None:
        raise DatasetError(f"meta/info.json has no `{key}` feature")
    names = feature.get("names")
    if not isinstance(names, list) or not all(isinstance(n, str) for n in names):
        raise DatasetError(
            f"`{key}` names must be a flat list of joint names, got {names!r}"
        )
    return names


def resolve_indices(names: Sequence[str], joints: Sequence[str]) -> list[int]:
    """Return the position of each of `joints` in `names`, in `joints` order."""
    duplicates = sorted({joint for joint in joints if list(joints).count(joint) > 1})
    if duplicates:
        raise DatasetError(f"joints listed more than once: {', '.join(duplicates)}")
    missing = [joint for joint in joints if joint not in names]
    if missing:
        raise DatasetError(
            f"`{STATE_KEY}` has no joint named {', '.join(missing)}; it has {', '.join(names)}"
        )
    return [list(names).index(joint) for joint in joints]


def _select_stat(
    stat: str, value: list, indices: list[int], width: int, where: str
) -> list:
    if stat in SCALAR_STATS:
        return value
    if len(value) != width:
        raise DatasetError(
            f"{where} `{stat}` has {len(value)} entries, expected {width}"
        )
    return [value[i] for i in indices]


def _select_list_column(
    column: pa.ChunkedArray, indices: list[int], width: int, where: str
) -> pa.Array:
    """Keep `indices` of every fixed-width list in `column`, preserving its Arrow type."""
    array = column.combine_chunks()
    if (
        array.null_count
        or pc.any(pc.not_equal(pc.list_value_length(array), width)).as_py()
    ):
        raise DatasetError(f"{where} must hold a {width}-entry list in every row")
    values = array.flatten().to_numpy(zero_copy_only=False).reshape(-1, width)
    selected = pa.array(values[:, indices].ravel(), type=array.type.value_type)
    if pa.types.is_fixed_size_list(array.type):
        return pa.FixedSizeListArray.from_arrays(selected, len(indices))
    list_class = (
        pa.LargeListArray if pa.types.is_large_list(array.type) else pa.ListArray
    )
    offsets = pa.array(
        np.arange(len(array) + 1) * len(indices), type=array.offsets.type
    )
    return list_class.from_arrays(offsets, selected, type=array.type)


def _narrow_hf_metadata(
    metadata: dict[bytes, bytes] | None, width: int
) -> dict[bytes, bytes] | None:
    """Update the state length a `datasets`-written parquet file records in its schema."""
    if not metadata or b"huggingface" not in metadata:
        return metadata
    hf = json.loads(metadata[b"huggingface"])
    feature = hf.get("info", {}).get("features", {}).get(STATE_KEY, {})
    if "length" in feature:
        feature["length"] = width
    return {**metadata, b"huggingface": json.dumps(hf).encode()}


def _rewrite_parquet(
    path: Path, columns: list[str], indices: list[int], width: int
) -> None:
    table = pq.read_table(path)
    for name in columns:
        position = table.schema.get_field_index(name)
        selected = _select_list_column(
            table.column(name), indices, width, f"{path} column `{name}`"
        )
        table = table.set_column(
            position, table.schema.field(position).with_type(selected.type), selected
        )
    table = table.replace_schema_metadata(
        _narrow_hf_metadata(table.schema.metadata, len(indices))
    )
    pq.write_table(table, path)


def _narrow_dataset(root: Path, names: list[str], indices: list[int]) -> None:
    width = len(names)
    info_path = root / "meta" / "info.json"
    info = _read_json(info_path)
    info["features"][STATE_KEY]["shape"] = [len(indices)]
    info["features"][STATE_KEY]["names"] = [names[i] for i in indices]
    _write_json(info_path, info)

    stats_path = root / "meta" / "stats.json"
    if stats_path.exists():
        stats = _read_json(stats_path)
        if STATE_KEY in stats:
            stats[STATE_KEY] = {
                stat: _select_stat(
                    stat, value, indices, width, f"meta/stats.json `{STATE_KEY}`"
                )
                for stat, value in stats[STATE_KEY].items()
            }
            _write_json(stats_path, stats)

    data_files = sorted((root / "data").rglob("*.parquet"))
    if not data_files:
        raise DatasetError("dataset has no data/**/*.parquet files")
    for path in data_files:
        if STATE_KEY not in pq.read_schema(path).names:
            raise DatasetError(f"{path} has no `{STATE_KEY}` column")
        _rewrite_parquet(path, [STATE_KEY], indices, width)

    for path in sorted((root / "meta" / "episodes").rglob("*.parquet")):
        columns = [
            name
            for name in pq.read_schema(path).names
            if name.startswith(EPISODE_STATS_PREFIX)
            and name.removeprefix(EPISODE_STATS_PREFIX) not in SCALAR_STATS
        ]
        _rewrite_parquet(path, columns, indices, width)


def probe_video(path: Path) -> dict:
    """Return the stream properties of the mp4 at `path`, keyed as LeRobot's `info` keys them."""
    try:
        import av
    except ImportError as error:
        raise DatasetError(
            "--like needs PyAV (`python3-av`) to probe the videos"
        ) from error
    with av.open(str(path)) as container:
        if not container.streams.video:
            raise DatasetError(f"{path} has no video stream")
        stream = container.streams.video[0]
        context = stream.codec_context
        # PyAV before 14 has no `canonical_name`; its decoder name matches it for h264 and hevc.
        codec = getattr(context.codec, "canonical_name", context.codec.name)
        return {
            "video.codec": codec,
            "video.pix_fmt": context.pix_fmt,
            "video.fps": float(stream.base_rate),
            "video.height": context.height,
            "video.width": context.width,
            "video.channels": len(av.VideoFormat(context.pix_fmt).components),
            "has_audio": bool(container.streams.audio),
        }


def _claimed_stream(key: str, feature: dict) -> dict:
    """Return the stream properties a video feature's metadata claims, keyed like `probe_video`."""
    shape = feature.get("shape")
    if not isinstance(shape, list) or len(shape) != 3:
        raise DatasetError(
            f"`{key}` shape must be [height, width, channels], got {shape!r}"
        )
    claims = dict(
        zip(("video.height", "video.width", "video.channels"), shape, strict=True)
    )
    for source in (feature.get("video_info") or {}, feature.get("info") or {}):
        for name, value in source.items():
            if name in PROBED_KEYS:
                claims[name] = float(value) if name == "video.fps" else value
    return claims


def _matched_video_feature(
    key: str,
    source: dict,
    reference: dict,
    videos: list[Path],
    probe: Callable[[Path], dict],
    log: Callable[[str], None],
) -> dict:
    """Return the reference's `key` feature, refusing if it misdescribes the source's `videos`.

    The `info` block keeps the reference's keys: stream properties are checked against every
    probed mp4, other non-encoder keys must equal the source's own metadata, and encoder
    settings are written as null because a file does not record them.
    """
    if not videos:
        raise DatasetError(f"source has no videos/{key}/**/*.mp4 to check")
    claims = _claimed_stream(key, reference)
    for path in videos:
        probed = probe(path)
        wrong = [
            f"{name} is {probed.get(name)!r}, the reference claims {value!r}"
            for name, value in claims.items()
            if probed.get(name) != value
        ]
        if wrong:
            raise DatasetError(
                f"{path} does not match the reference `{key}`: {'; '.join(wrong)}"
            )
    matched = copy.deepcopy(reference)
    if "info" not in reference:
        matched.pop("info", None)
        return matched
    attested = {**(source.get("video_info") or {}), **(source.get("info") or {})}
    unverified = [
        name
        for name, value in reference["info"].items()
        if name not in VIDEO_ENCODER_INFO_KEYS | PROBED_KEYS
        and attested.get(name, object()) != value
    ]
    if unverified:
        raise DatasetError(
            f"cannot confirm the reference `{key}` info {unverified} for the source videos"
        )
    matched["info"] = {
        name: None if name in VIDEO_ENCODER_INFO_KEYS else value
        for name, value in reference["info"].items()
    }
    log(
        f"{key}: {len(videos)} mp4 probed, matching "
        + ", ".join(f"{name}={value!r}" for name, value in claims.items())
        + f"; info {sorted(matched['info'])} written, encoder settings null"
    )
    return matched


def _like_features(
    src: Path,
    info: dict,
    reference_info: dict,
    probe: Callable[[Path], dict],
    log: Callable[[str], None],
) -> dict:
    """Return `info`'s features with the reference's metadata, refusing any real difference.

    Non-video features must agree in everything but a per-feature `fps` equal to the dataset
    fps; video features in everything but `info` (see `_matched_video_feature`).
    """
    fps = info.get("fps")
    problems = [
        f"`{key}` is {info.get(key)!r}, the reference has {reference_info.get(key)!r}"
        for key in MERGE_INFO_KEYS
        if info.get(key) != reference_info.get(key)
    ]
    features, reference = info["features"], reference_info["features"]
    if set(features) != set(reference):
        problems.append(
            f"source-only features {sorted(set(features) - set(reference))}, "
            f"reference-only features {sorted(set(reference) - set(features))}"
        )
    for key in [key for key in features if key in reference]:
        ours, theirs = features[key], reference[key]
        for field in ("dtype", "shape", "names"):
            if ours.get(field) != theirs.get(field):
                problems.append(
                    f"`{key}` {field} is {ours.get(field)!r}, "
                    f"the reference has {theirs.get(field)!r}"
                )
        ignored = {"info"} if ours.get("dtype") == "video" else {"fps"}
        differing = sorted(
            field
            for field in (set(ours) | set(theirs))
            - ignored
            - {"dtype", "shape", "names"}
            if ours.get(field) != theirs.get(field)
        )
        if differing:
            problems.append(f"`{key}` differs from the reference in {differing}")
        if "fps" in ignored:
            problems.extend(
                f"{side} `{key}` fps is {feature['fps']!r}, not the dataset fps {fps!r}"
                for side, feature in (("source", ours), ("reference", theirs))
                if "fps" in feature and feature["fps"] != fps
            )
    if problems:
        raise DatasetError(
            "source cannot be made like the reference: " + "; ".join(problems)
        )

    matched = {
        key: (
            _matched_video_feature(
                key,
                feature,
                reference[key],
                sorted((src / "videos" / key).rglob("*.mp4")),
                probe,
                log,
            )
            if feature["dtype"] == "video"
            else copy.deepcopy(reference[key])
        )
        for key, feature in features.items()
    }
    copied = [key for key in features if features[key] != matched[key]]
    log(f"feature metadata taken from the reference: {', '.join(copied) or 'none'}")
    return matched


def _episode_values(root: Path, key: str) -> dict[int, np.ndarray]:
    """Return `key`'s frames x width values for each episode index."""
    chunks: dict[int, list[np.ndarray]] = {}
    for path in sorted((root / "data").rglob("*.parquet")):
        table = pq.read_table(path, columns=["episode_index", key])
        episodes = table.column("episode_index").to_numpy()
        column = table.column(key).combine_chunks()
        if (
            pa.types.is_list(column.type)
            or pa.types.is_fixed_size_list(column.type)
            or pa.types.is_large_list(column.type)
        ):
            values = (
                column.flatten().to_numpy(zero_copy_only=False).reshape(len(column), -1)
            )
        else:
            values = column.to_numpy(zero_copy_only=False).reshape(-1, 1)
        for episode in np.unique(episodes):
            chunks.setdefault(int(episode), []).append(values[episodes == episode])
    return {episode: np.concatenate(parts) for episode, parts in chunks.items()}


def _backfill_quantiles(
    root: Path, reference: Path, log: Callable[[str], None]
) -> None:
    """Add the quantiles the reference has and `root` lacks, then require equal stat layouts.

    Each episode's quantiles are exact (`np.quantile`); the dataset's are the frame-count-weighted
    mean of its episodes', which is how LeRobot aggregates quantiles across episodes and datasets.
    """
    reference_stats_path = reference / "meta" / "stats.json"
    if not reference_stats_path.exists():
        return
    reference_stats = _read_json(reference_stats_path)
    stats_path = root / "meta" / "stats.json"
    stats = _read_json(stats_path) if stats_path.exists() else {}
    if set(stats) != set(reference_stats):
        raise DatasetError(
            f"meta/stats.json covers {sorted(stats)}, "
            f"the reference's covers {sorted(reference_stats)}"
        )

    episode_files = sorted((root / "meta" / "episodes").rglob("*.parquet"))
    reference_columns = {
        name
        for path in sorted((reference / "meta" / "episodes").rglob("*.parquet"))
        for name in pq.read_schema(path).names
    }
    for key, reference_stat in reference_stats.items():
        missing = [
            stat
            for stat in reference_stat
            if QUANTILE_STAT.fullmatch(stat) and stat not in stats[key]
        ]
        if not missing:
            continue
        per_episode = _episode_values(root, key)
        quantiles = {
            stat: {
                episode: np.quantile(
                    values, int(QUANTILE_STAT.fullmatch(stat)[1]) / 100, axis=0
                )
                for episode, values in per_episode.items()
            }
            for stat in missing
        }
        counts = {episode: len(values) for episode, values in per_episode.items()}
        total = sum(counts.values())
        for stat in missing:
            stats[key][stat] = (
                sum(counts[e] * q for e, q in quantiles[stat].items()) / total
            ).tolist()
        columns = [
            stat for stat in missing if f"stats/{key}/{stat}" in reference_columns
        ]
        for path in episode_files:
            table = pq.read_table(path)
            episodes = table.column("episode_index").to_pylist()
            for stat in columns:
                table = table.append_column(
                    f"stats/{key}/{stat}",
                    pa.array(
                        [quantiles[stat][e].tolist() for e in episodes],
                        type=pa.list_(pa.float64()),
                    ),
                )
            pq.write_table(table, path)
        log(
            f"{key}: computed {', '.join(missing)} from {total} frames"
            + (f"; per-episode columns {', '.join(columns)}" if columns else "")
        )
    _write_json(stats_path, stats)

    for key, reference_stat in reference_stats.items():
        if set(stats[key]) != set(reference_stat):
            raise DatasetError(
                f"meta/stats.json `{key}` has {sorted(stats[key])}, "
                f"the reference has {sorted(reference_stat)}"
            )
    columns = {name for path in episode_files for name in pq.read_schema(path).names}
    if columns != reference_columns:
        raise DatasetError(
            "meta/episodes columns differ from the reference's: "
            f"source-only {sorted(columns - reference_columns)}, "
            f"reference-only {sorted(reference_columns - columns)}"
        )


def select_state_joints(
    src: Path,
    dst: Path,
    joints: Sequence[str] | None = None,
    like: Path | None = None,
    probe: Callable[[Path], dict] = probe_video,
    log: Callable[[str], None] = lambda message: None,
) -> list[str]:
    """Write a copy of the dataset at `src` to `dst` whose state holds only `joints`.

    `joints` defaults to the `action` joint names. With `like`, the copy's feature metadata and
    statistics are matched to the dataset at `like` (see the module docstring), `probe` reads a
    video's stream properties, and `log` receives a line per change. Returns the kept joint names.
    """
    src, dst = Path(src), Path(dst)
    if dst.exists() or dst.is_symlink():
        raise FileExistsError(f"destination {dst} already exists")
    if dst.resolve().is_relative_to(src.resolve()):
        raise DatasetError(f"destination {dst} is inside the source dataset {src}")
    info = _read_json(src / "meta" / "info.json")
    names = _feature_names(info, STATE_KEY)
    joints = list(joints) if joints is not None else _feature_names(info, "action")
    indices = resolve_indices(names, joints)
    like_features = None
    if like is not None:
        like = Path(like)
        narrowed = copy.deepcopy(info)
        narrowed["features"][STATE_KEY]["shape"] = [len(indices)]
        narrowed["features"][STATE_KEY]["names"] = list(joints)
        like_features = _like_features(
            src, narrowed, _read_json(like / "meta" / "info.json"), probe, log
        )

    def link_or_copy(source: str, destination: str) -> None:
        if Path(source).relative_to(src).parts[0] == "videos":
            try:
                os.link(source, destination)
                return
            except OSError:
                pass
        shutil.copy2(source, destination)

    dst.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{dst.name}.", dir=dst.parent))
    try:
        shutil.copytree(src, staging, copy_function=link_or_copy, dirs_exist_ok=True)
        if indices != list(range(len(names))):
            _narrow_dataset(staging, names, indices)
        if like_features is not None:
            info_path = staging / "meta" / "info.json"
            staged_info = _read_json(info_path)
            staged_info["features"] = like_features
            _write_json(info_path, staged_info)
            _backfill_quantiles(staging, like, log)
        staging.rename(dst)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return joints


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", type=Path, help="LeRobot v3 dataset to read")
    parser.add_argument(
        "dst", type=Path, help="new dataset directory to write; must not exist"
    )
    parser.add_argument(
        "--joints",
        type=lambda value: [
            joint.strip() for joint in value.split(",") if joint.strip()
        ],
        help="comma-separated state joints to keep, in order (default: the `action` joints)",
    )
    parser.add_argument(
        "--like",
        type=Path,
        help="LeRobot v3 dataset to match feature metadata and statistics with, for merging",
    )
    args = parser.parse_args()
    try:
        kept = select_state_joints(
            args.src, args.dst, args.joints, like=args.like, log=print
        )
    except (DatasetError, FileExistsError) as error:
        sys.exit(f"error: {error}")
    print(f"wrote {args.dst} with {STATE_KEY} = [{', '.join(kept)}]")


if __name__ == "__main__":
    main()
