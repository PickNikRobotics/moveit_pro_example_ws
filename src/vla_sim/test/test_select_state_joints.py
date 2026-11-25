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

"""Tests for narrowing a LeRobot v3 dataset's `observation.state` to named joints."""

import json
from pathlib import Path
import re
import subprocess
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "script" / "select_state_joints.py"
sys.path.insert(0, str(SCRIPT.parent))

from select_state_joints import (  # noqa: E402
    DatasetError,
    probe_video,
    select_state_joints,
)

try:
    from lerobot.datasets.aggregate import aggregate_datasets
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    LeRobotDataset = None

try:
    import av
except ImportError:
    av = None

ACTION_JOINTS = [f"joint_{i}" for i in range(1, 8)] + ["robotiq_85_left_knuckle_joint"]
STATE_JOINTS = [f"joint_{i}" for i in range(1, 8)] + [
    f"robotiq_85_{side}_{link}_joint"
    for side in ("left", "right")
    for link in ("finger", "finger_tip", "inner_knuckle", "knuckle")
]
EPISODE_LENGTHS = [5, 7]
QUANTILES = {"q01": 0.01, "q10": 0.10, "q50": 0.50, "q90": 0.90, "q99": 0.99}
VIDEO_KEY = "observation.images.wrist"
VIDEO_INFO = {
    "video.fps": 10.0,
    "video.codec": "h264",
    "video.pix_fmt": "yuv420p",
    "video.is_depth_map": False,
    "has_audio": False,
}
# The encoder-settings `info` block of the published cube-stacking dataset's video features.
ENCODER_INFO = {
    "video.video_backend": "pyav",
    "video.extra_options": None,
    "video.g": None,
    "video.fast_decode": None,
    "video.preset": None,
    "video.crf": None,
}
# What `probe_video` reports for a video that matches VIDEO_INFO and the 4x4x3 video shape.
MATCHING_STREAM = {
    "video.codec": "h264",
    "video.pix_fmt": "yuv420p",
    "video.fps": 10.0,
    "video.height": 4,
    "video.width": 4,
    "video.channels": 3,
    "has_audio": False,
}


def compute_stats(values: np.ndarray, quantiles: bool) -> dict[str, list]:
    """Per-joint statistics in LeRobot's layout."""
    stats = {
        "min": values.min(axis=0).tolist(),
        "max": values.max(axis=0).tolist(),
        "mean": values.mean(axis=0).tolist(),
        "std": values.std(axis=0).tolist(),
        "count": [len(values)],
    }
    if quantiles:
        stats.update(
            {
                key: np.quantile(values, q, axis=0).tolist()
                for key, q in QUANTILES.items()
            }
        )
    return stats


def feature(names: list[str]) -> dict:
    """Describe a float32 joint-vector feature."""
    return {"dtype": "float32", "shape": [len(names)], "names": names, "fps": 10.0}


def scalar_feature(dtype: str, fps: bool = False) -> dict:
    """Describe a per-frame scalar feature, with the `fps` Trainer adds when `fps` is set."""
    described = {"dtype": dtype, "shape": [1], "names": None}
    if fps:
        described["fps"] = 10.0
    return described


def write_dataset(
    root: Path,
    with_video: bool = True,
    state_joints: list[str] = STATE_JOINTS,
    quantiles: bool = True,
    scalar_fps: bool = False,
    encoder_info: bool = False,
    episode_quantiles: bool = False,
) -> None:
    """Write a two-episode LeRobot v3 dataset, by default one whose state is wider than its action.

    `quantiles` and `episode_quantiles` add quantiles to `meta/stats.json` and to the per-episode
    statistics; `scalar_fps` and `encoder_info` add Trainer's per-frame `fps` and the published
    dataset's video `info` block.
    """
    rng = np.random.default_rng(0)
    total = sum(EPISODE_LENGTHS)
    state = rng.normal(size=(total, len(state_joints)))
    action = rng.normal(size=(total, len(ACTION_JOINTS)))
    episode_index = np.repeat(np.arange(len(EPISODE_LENGTHS)), EPISODE_LENGTHS)
    frame_index = np.concatenate([np.arange(n) for n in EPISODE_LENGTHS])

    features = {
        "observation.state": feature(state_joints),
        "action": feature(ACTION_JOINTS),
        "episode_index": scalar_feature("int64", scalar_fps),
        "frame_index": scalar_feature("int64", scalar_fps),
        "timestamp": scalar_feature("float32", scalar_fps),
        "index": scalar_feature("int64", scalar_fps),
        "task_index": scalar_feature("int64", scalar_fps),
    }
    if with_video:
        features[VIDEO_KEY] = {
            "dtype": "video",
            "shape": [4, 4, 3],
            "names": ["height", "width", "channel"],
            "video_info": dict(VIDEO_INFO),
        }
        if encoder_info:
            features[VIDEO_KEY]["info"] = dict(ENCODER_INFO)
        video = root / "videos" / VIDEO_KEY / "chunk-000" / "file-000.mp4"
        video.parent.mkdir(parents=True)
        video.write_bytes(b"not a real video")
    info = {
        "codebase_version": "v3.0",
        "robot_type": "kinova_gen3",
        "total_episodes": len(EPISODE_LENGTHS),
        "total_frames": total,
        "total_tasks": 1,
        "chunks_size": 1000,
        "fps": 10,
        "splits": {"train": f"0:{len(EPISODE_LENGTHS)}"},
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
        "data_files_size_in_mb": 100,
        "video_files_size_in_mb": 200,
        "features": features,
    }
    (root / "meta" / "episodes" / "chunk-000").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    (root / "meta" / "info.json").write_text(json.dumps(info, indent=2))
    (root / "meta" / "stats.json").write_text(
        json.dumps(
            {
                "observation.state": compute_stats(state, quantiles),
                "action": compute_stats(action, quantiles),
            }
        )
    )
    pq.write_table(
        pa.table({"task_index": [0], "task": ["stack the red cube on the green cube"]}),
        root / "meta" / "tasks.parquet",
    )
    pq.write_table(
        pa.table(
            {
                "episode_index": episode_index,
                "frame_index": frame_index,
                "index": np.arange(total),
                "task_index": np.zeros(total, dtype=np.int64),
                "timestamp": frame_index / 10.0,
                "observation.state": list(state),
                "action": list(action),
            }
        ),
        root / "data" / "chunk-000" / "file-000.parquet",
    )

    starts = np.cumsum([0] + EPISODE_LENGTHS)
    episodes = {
        "episode_index": list(range(len(EPISODE_LENGTHS))),
        "length": EPISODE_LENGTHS,
        "tasks": [["stack the red cube on the green cube"]] * len(EPISODE_LENGTHS),
        "data/chunk_index": [0] * len(EPISODE_LENGTHS),
        "data/file_index": [0] * len(EPISODE_LENGTHS),
        "dataset_from_index": starts[:-1].tolist(),
        "dataset_to_index": starts[1:].tolist(),
        "meta/episodes/chunk_index": [0] * len(EPISODE_LENGTHS),
        "meta/episodes/file_index": [0] * len(EPISODE_LENGTHS),
    }
    per_episode = [
        {
            key: compute_stats(values[start:end], episode_quantiles)
            for key, values in [("observation.state", state), ("action", action)]
        }
        for start, end in zip(starts[:-1], starts[1:])
    ]
    for key in ("observation.state", "action"):
        for stat in per_episode[0][key]:
            episodes[f"stats/{key}/{stat}"] = [
                episode[key][stat] for episode in per_episode
            ]
    pq.write_table(
        pa.table(episodes),
        root / "meta" / "episodes" / "chunk-000" / "file-000.parquet",
    )


def read_json(path: Path) -> dict:
    """Load a JSON file."""
    return json.loads(path.read_text())


def edit_info(root: Path, edit) -> None:
    """Apply `edit` to the dataset's `meta/info.json` content in place."""
    path = root / "meta" / "info.json"
    info = read_json(path)
    edit(info)
    path.write_text(json.dumps(info, indent=2))


def matching_probe(path: Path) -> dict:
    """Stand in for `probe_video` on the placeholder videos, reporting a matching stream."""
    return dict(MATCHING_STREAM)


def episode_columns(root: Path) -> set[str]:
    """Return the dataset's `meta/episodes` column names."""
    return set(
        pq.read_schema(
            root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
        ).names
    )


def state_values(root: Path) -> np.ndarray:
    """Return the dataset's state as a frames x joints array."""
    return np.array(
        pq.read_table(root / "data" / "chunk-000" / "file-000.parquet")
        .column("observation.state")
        .to_pylist()
    )


@pytest.fixture
def source(tmp_path: Path) -> Path:
    """Write a synthetic dataset with a 15-joint state and an 8-joint action."""
    root = tmp_path / "source"
    write_dataset(root)
    return root


@pytest.fixture
def trainer(tmp_path: Path) -> Path:
    """Write a dataset shaped like a Trainer conversion: wide state, scalar fps, no quantiles."""
    root = tmp_path / "trainer"
    write_dataset(root, quantiles=False, scalar_fps=True)
    return root


@pytest.fixture
def reference(tmp_path: Path) -> Path:
    """Write a dataset shaped like the published one: 8-joint state, video `info`, quantiles."""
    root = tmp_path / "reference"
    write_dataset(root, state_joints=ACTION_JOINTS, encoder_info=True)
    return root


def test_default_selection_keeps_action_joints_in_action_order(
    source: Path, tmp_path: Path
) -> None:
    """The narrowed state is float32 [8] with the `action` names, like the published dataset."""
    destination = tmp_path / "narrowed"
    assert select_state_joints(source, destination) == ACTION_JOINTS

    features = read_json(destination / "meta" / "info.json")["features"]
    assert features["observation.state"] == feature(ACTION_JOINTS)
    indices = [STATE_JOINTS.index(joint) for joint in ACTION_JOINTS]
    np.testing.assert_array_equal(
        state_values(destination), state_values(source)[:, indices]
    )
    data_type = (
        pq.read_schema(destination / "data" / "chunk-000" / "file-000.parquet")
        .field("observation.state")
        .type
    )
    assert data_type == pa.list_(pa.float64()), "the state column keeps its Arrow type"


def test_untouched_features_and_files_match_the_source(
    source: Path, tmp_path: Path
) -> None:
    """Only `observation.state` changes; action, other metadata and videos are carried over."""
    destination = tmp_path / "narrowed"
    select_state_joints(source, destination)

    source_info = read_json(source / "meta" / "info.json")
    narrowed_info = read_json(destination / "meta" / "info.json")
    source_info["features"].pop("observation.state")
    narrowed_info["features"].pop("observation.state")
    assert narrowed_info == source_info
    data = "data/chunk-000/file-000.parquet"
    assert (
        pq.read_table(destination / data)
        .drop_columns(["observation.state"])
        .equals(pq.read_table(source / data).drop_columns(["observation.state"]))
    )
    assert (
        read_json(destination / "meta" / "stats.json")["action"]
        == read_json(source / "meta" / "stats.json")["action"]
    )
    assert (destination / "meta" / "tasks.parquet").read_bytes() == (
        source / "meta" / "tasks.parquet"
    ).read_bytes()
    video = Path("videos") / VIDEO_KEY / "chunk-000" / "file-000.mp4"
    assert (destination / video).stat().st_ino == (
        source / video
    ).stat().st_ino, "videos are hard-linked"


def test_statistics_equal_a_recomputation_on_the_kept_joints(
    source: Path, tmp_path: Path
) -> None:
    """Narrowed dataset and per-episode statistics, quantiles included, match the narrowed data."""
    destination = tmp_path / "narrowed"
    select_state_joints(source, destination)
    state = state_values(destination)

    stats = read_json(destination / "meta" / "stats.json")["observation.state"]
    expected = compute_stats(state, quantiles=True)
    assert stats.keys() == expected.keys()
    for stat, value in expected.items():
        np.testing.assert_allclose(stats[stat], value, err_msg=stat)

    episodes = pq.read_table(
        destination / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    for episode in episodes:
        rows = state[range(episode["dataset_from_index"], episode["dataset_to_index"])]
        for stat, value in compute_stats(rows, quantiles=False).items():
            np.testing.assert_allclose(
                episode[f"stats/observation.state/{stat}"], value, err_msg=stat
            )


def test_explicit_joints_select_by_name_in_the_given_order(
    source: Path, tmp_path: Path
) -> None:
    """`joints` picks columns by name, so a reordered list reorders the state."""
    destination = tmp_path / "narrowed"
    joints = ["robotiq_85_right_knuckle_joint", "joint_3"]
    select_state_joints(source, destination, joints)

    assert (
        read_json(destination / "meta" / "info.json")["features"]["observation.state"][
            "names"
        ]
        == joints
    )
    np.testing.assert_array_equal(
        state_values(destination), state_values(source)[:, [14, 2]]
    )


def test_missing_joint_is_named_and_nothing_is_written(
    source: Path, tmp_path: Path
) -> None:
    """An absent joint fails with its name and leaves no destination or staging directory."""
    destination = tmp_path / "narrowed"
    with pytest.raises(DatasetError, match="no joint named wrist_roll_joint"):
        select_state_joints(source, destination, ["joint_1", "wrist_roll_joint"])
    assert sorted(path.name for path in tmp_path.iterdir()) == ["source"]


def test_existing_destination_is_refused(source: Path, tmp_path: Path) -> None:
    """An existing destination is left untouched."""
    destination = tmp_path / "narrowed"
    destination.mkdir()
    (destination / "keep").write_text("mine")
    with pytest.raises(FileExistsError, match="already exists"):
        select_state_joints(source, destination)
    assert [path.name for path in destination.iterdir()] == ["keep"]


def test_already_narrow_dataset_is_copied_byte_for_byte(tmp_path: Path) -> None:
    """A dataset whose state already holds exactly the joints is copied unchanged."""
    source = tmp_path / "source"
    write_dataset(source, state_joints=ACTION_JOINTS)
    destination = tmp_path / "copy"
    select_state_joints(source, destination)

    source_files = sorted(
        path.relative_to(source) for path in source.rglob("*") if path.is_file()
    )
    assert source_files == sorted(
        path.relative_to(destination)
        for path in destination.rglob("*")
        if path.is_file()
    )
    for relative in source_files:
        assert (destination / relative).read_bytes() == (
            source / relative
        ).read_bytes(), relative


def test_cli_reports_a_missing_joint(source: Path, tmp_path: Path) -> None:
    """The command exits non-zero with the missing joint in its message."""
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(source),
            str(tmp_path / "narrowed"),
            "--joints",
            "joint_1,nope",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "no joint named nope" in result.stderr


@pytest.mark.skipif(LeRobotDataset is None, reason="lerobot is not installed")
def test_lerobot_loads_the_narrowed_dataset(tmp_path: Path) -> None:
    """The result loads in LeRobot, and each frame's state has the kept joints' values."""
    source = tmp_path / "source"
    write_dataset(source, with_video=False)
    destination = tmp_path / "narrowed"
    select_state_joints(source, destination)

    dataset = LeRobotDataset(repo_id="local/narrowed", root=destination)
    assert dataset.meta.features["observation.state"]["names"] == ACTION_JOINTS
    assert dataset.num_episodes == len(EPISODE_LENGTHS)
    frame = dataset[len(dataset) - 1]
    assert tuple(frame["observation.state"].shape) == (len(ACTION_JOINTS),)
    np.testing.assert_allclose(
        frame["observation.state"].numpy(), state_values(destination)[-1], rtol=1e-6
    )


def test_like_takes_scalar_and_video_metadata_from_the_reference(
    trainer: Path, reference: Path, tmp_path: Path
) -> None:
    """Features end up equal to the reference's, except video encoder settings, which are null."""
    destination = tmp_path / "narrowed"
    probed = []

    def probe(path: Path) -> dict:
        probed.append(path.relative_to(trainer))
        return matching_probe(path)

    select_state_joints(trainer, destination, like=reference, probe=probe)

    features = read_json(destination / "meta" / "info.json")["features"]
    expected = read_json(reference / "meta" / "info.json")["features"]
    expected[VIDEO_KEY]["info"] = dict.fromkeys(ENCODER_INFO)
    assert features == expected
    assert probed == [Path("videos") / VIDEO_KEY / "chunk-000" / "file-000.mp4"]


def test_like_backfills_dataset_quantiles_lerobot_style(
    trainer: Path, reference: Path, tmp_path: Path
) -> None:
    """Missing quantiles are frame-weighted means of exact per-episode ones; nothing else moves."""
    destination = tmp_path / "narrowed"
    select_state_joints(trainer, destination, like=reference, probe=matching_probe)

    stats = read_json(destination / "meta" / "stats.json")
    reference_stats = read_json(reference / "meta" / "stats.json")
    source_stats = read_json(trainer / "meta" / "stats.json")
    data = pq.read_table(destination / "data" / "chunk-000" / "file-000.parquet")
    starts = np.cumsum([0] + EPISODE_LENGTHS)
    for key in ("observation.state", "action"):
        assert stats[key].keys() == reference_stats[key].keys()
        values = np.array(data.column(key).to_pylist())
        for stat, q in QUANTILES.items():
            per_episode = [
                np.quantile(values[start:end], q, axis=0)
                for start, end in zip(starts[:-1], starts[1:])
            ]
            np.testing.assert_allclose(
                stats[key][stat],
                np.average(per_episode, axis=0, weights=EPISODE_LENGTHS),
                err_msg=f"{key} {stat}",
            )
    assert stats["action"]["mean"] == source_stats["action"]["mean"]
    assert episode_columns(destination) == episode_columns(
        reference
    ), "per-episode columns follow the reference, which has no per-episode quantiles"


def test_like_adds_per_episode_quantiles_the_reference_has(
    trainer: Path, tmp_path: Path
) -> None:
    """When the reference's episodes carry quantile columns, the output gets them too."""
    reference = tmp_path / "reference"
    write_dataset(
        reference, state_joints=ACTION_JOINTS, encoder_info=True, episode_quantiles=True
    )
    destination = tmp_path / "narrowed"
    select_state_joints(trainer, destination, like=reference, probe=matching_probe)

    assert episode_columns(destination) == episode_columns(reference)
    state = state_values(destination)
    episodes = pq.read_table(
        destination / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    for episode in episodes:
        rows = state[range(episode["dataset_from_index"], episode["dataset_to_index"])]
        for stat, q in QUANTILES.items():
            np.testing.assert_allclose(
                episode[f"stats/observation.state/{stat}"],
                np.quantile(rows, q, axis=0),
                err_msg=stat,
            )


@pytest.mark.parametrize(
    "edit, message",
    [
        (
            lambda info: info["features"]["action"].update(dtype="float64"),
            "`action` dtype is 'float32', the reference has 'float64'",
        ),
        (
            lambda info: info["features"]["action"]["names"].reverse(),
            "`action` names",
        ),
        (
            lambda info: info["features"].update(
                {"observation.effort": feature(ACTION_JOINTS)}
            ),
            "reference-only features ['observation.effort']",
        ),
        (
            lambda info: info.update(robot_type="kinova_gen3_7dof"),
            "`robot_type` is 'kinova_gen3', the reference has 'kinova_gen3_7dof'",
        ),
        (
            lambda info: info["features"]["timestamp"].update(fps=30.0),
            "reference `timestamp` fps is 30.0, not the dataset fps 10",
        ),
        (
            lambda info: info["features"]["action"].update(unit="rad"),
            "`action` differs from the reference in ['unit']",
        ),
        (
            lambda info: info["features"][VIDEO_KEY]["video_info"].update(
                {"video.codec": "av1"}
            ),
            f"`{VIDEO_KEY}` differs from the reference in ['video_info']",
        ),
    ],
)
def test_like_refuses_a_reference_whose_features_disagree(
    trainer: Path, reference: Path, tmp_path: Path, edit, message: str
) -> None:
    """A feature difference beyond scalar fps and video `info` is named; nothing is written."""
    edit_info(reference, edit)
    with pytest.raises(DatasetError, match=re.escape(message)):
        select_state_joints(
            trainer, tmp_path / "narrowed", like=reference, probe=matching_probe
        )
    assert not (tmp_path / "narrowed").exists()
    assert not list(tmp_path.glob(".narrowed.*")), "no staging directory is left"


def test_like_refuses_a_state_selection_the_reference_does_not_have(
    trainer: Path, reference: Path, tmp_path: Path
) -> None:
    """Joints that differ from the reference's state are caught before anything is copied."""
    with pytest.raises(DatasetError, match="`observation.state` shape is \\[2\\]"):
        select_state_joints(
            trainer,
            tmp_path / "narrowed",
            ["joint_1", "joint_2"],
            like=reference,
            probe=matching_probe,
        )
    assert not (tmp_path / "narrowed").exists()


@pytest.mark.parametrize(
    "stream, message",
    [
        (
            {"video.pix_fmt": "yuv444p"},
            "video.pix_fmt is 'yuv444p', the reference claims 'yuv420p'",
        ),
        ({"video.fps": 30.0}, "video.fps is 30.0, the reference claims 10.0"),
        ({"video.width": 8}, "video.width is 8, the reference claims 4"),
        ({"video.codec": "hevc"}, "video.codec is 'hevc', the reference claims 'h264'"),
    ],
)
def test_like_refuses_videos_the_reference_misdescribes(
    trainer: Path, reference: Path, tmp_path: Path, stream: dict, message: str
) -> None:
    """A probed stream property that contradicts the reference's metadata is refused."""
    with pytest.raises(DatasetError, match=re.escape(message)):
        select_state_joints(
            trainer,
            tmp_path / "narrowed",
            like=reference,
            probe=lambda path: {**MATCHING_STREAM, **stream},
        )
    assert not (tmp_path / "narrowed").exists()


def test_like_refuses_reference_info_it_cannot_confirm(
    trainer: Path, reference: Path, tmp_path: Path
) -> None:
    """A reference `info` claim that is neither probed nor in the source's metadata is refused."""
    edit_info(
        reference,
        lambda info: info["features"][VIDEO_KEY]["info"].update(depth_unit="mm"),
    )
    with pytest.raises(DatasetError, match="cannot confirm .*depth_unit"):
        select_state_joints(
            trainer, tmp_path / "narrowed", like=reference, probe=matching_probe
        )


def test_like_refuses_stats_it_cannot_match(
    trainer: Path, reference: Path, tmp_path: Path
) -> None:
    """A missing statistic other than a quantile, here `std`, cannot be backfilled."""
    stats_path = trainer / "meta" / "stats.json"
    stats = read_json(stats_path)
    del stats["action"]["std"]
    stats_path.write_text(json.dumps(stats))
    with pytest.raises(DatasetError, match="`action` has .*the reference has"):
        select_state_joints(
            trainer, tmp_path / "narrowed", like=reference, probe=matching_probe
        )
    assert not (tmp_path / "narrowed").exists()


def test_cli_like_reports_a_disagreeing_reference(tmp_path: Path) -> None:
    """The command exits non-zero naming the disagreement."""
    source, reference = tmp_path / "source", tmp_path / "reference"
    write_dataset(source, with_video=False, quantiles=False, scalar_fps=True)
    write_dataset(reference, with_video=False, state_joints=ACTION_JOINTS)
    edit_info(reference, lambda info: info.update(fps=30))
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(source),
            str(tmp_path / "narrowed"),
            "--like",
            str(reference),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "`fps` is 10, the reference has 30" in result.stderr


@pytest.mark.skipif(av is None, reason="PyAV is not installed")
def test_probe_video_reads_an_encoded_mp4(tmp_path: Path) -> None:
    """`probe_video` reports the codec, pixel format, rate and size an mp4 was written with."""
    path = tmp_path / "clip.mp4"
    with av.open(str(path), "w") as container:
        stream = container.add_stream("mpeg4", rate=10)
        stream.width, stream.height, stream.pix_fmt = 32, 16, "yuv420p"
        for _ in range(5):
            frame = av.VideoFrame.from_ndarray(
                np.zeros((16, 32, 3), dtype=np.uint8), format="rgb24"
            )
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)

    assert probe_video(path) == {
        "video.codec": "mpeg4",
        "video.pix_fmt": "yuv420p",
        "video.fps": 10.0,
        "video.height": 16,
        "video.width": 32,
        "video.channels": 3,
        "has_audio": False,
    }


@pytest.mark.skipif(LeRobotDataset is None, reason="lerobot is not installed")
def test_lerobot_merges_a_like_copy_and_keeps_quantiles(tmp_path: Path) -> None:
    """The `--like` copy merges through LeRobot's aggregation, and the result keeps quantiles."""
    source, reference = tmp_path / "source", tmp_path / "reference"
    write_dataset(source, with_video=False, quantiles=False, scalar_fps=True)
    write_dataset(reference, with_video=False, state_joints=ACTION_JOINTS)
    destination = tmp_path / "narrowed"
    select_state_joints(source, destination, like=reference)

    merged = tmp_path / "merged"
    aggregate_datasets(
        repo_ids=["local/reference", "local/narrowed"],
        aggr_repo_id="local/merged",
        roots=[reference, destination],
        aggr_root=merged,
    )
    dataset = LeRobotDataset(repo_id="local/merged", root=merged)
    assert dataset.num_episodes == 2 * len(EPISODE_LENGTHS)
    assert len(dataset.meta.tasks) == 1, "the shared prompt maps to one task"
    for key in ("observation.state", "action"):
        assert set(QUANTILES) <= set(dataset.meta.stats[key]), key
    frame = dataset[len(dataset) - 1]
    assert tuple(frame["observation.state"].shape) == (len(ACTION_JOINTS),)
    np.testing.assert_allclose(
        frame["observation.state"].numpy(), state_values(destination)[-1], rtol=1e-6
    )
