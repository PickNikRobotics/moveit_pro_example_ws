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

"""Tests for the generated cube layouts, their keyframes, and the Objectives that record them."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "script"))

import generate_cube_stack_layouts as gen  # noqa: E402

SCENE_PATH = gen.PACKAGE_DIR / "description" / "mujoco" / "cube_stack_scene.xml"


@pytest.fixture(scope="module")
def generated_keys() -> list[ET.Element]:
    """The keys of the committed keyframes_generated.xml."""
    return gen.read_keys(gen.GENERATED_KEYFRAMES_PATH)


@pytest.fixture(scope="module")
def generated_layouts(generated_keys: list[ET.Element]) -> np.ndarray:
    """(N, 3, 3) per-cube x, y, yaw of the committed generated keys."""
    return _as_array(
        [
            gen.layout_from_qpos([float(v) for v in key.get("qpos").split()])
            for key in generated_keys
        ]
    )


def _as_array(layouts: list[gen.Layout]) -> np.ndarray:
    return np.array(
        [
            [(x, y, yaw) for (x, y), yaw in zip(layout.xy, layout.yaws)]
            for layout in layouts
        ]
    )


def _near_duplicate_pairs(
    a: np.ndarray, b: np.ndarray, pos_tol: float, yaw_tol: float
) -> np.ndarray:
    """(i, j) with every cube of a[i] within pos_tol and yaw_tol (modulo a quarter turn) of the same cube of b[j]."""
    pairs = []
    chunk = 100
    for start in range(0, len(a), chunk):
        rows = a[start : start + chunk]
        position_gap = np.linalg.norm(rows[:, None, :, :2] - b[None, :, :, :2], axis=3)
        yaw_gap = np.abs(rows[:, None, :, 2] - b[None, :, :, 2]) % (math.pi / 2)
        yaw_gap = np.minimum(yaw_gap, math.pi / 2 - yaw_gap)
        hits = np.argwhere(
            np.all((position_gap < pos_tol) & (yaw_gap < yaw_tol), axis=2)
        )
        hits[:, 0] += start
        pairs.append(hits)
    return np.concatenate(pairs)


def _key_prompts() -> dict[str, str]:
    """Each generated key's name and the prompt in the comment after it."""
    prompts = {}
    name = None
    for line in gen.GENERATED_KEYFRAMES_PATH.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith('name="'):
            name = stripped.removeprefix('name="').removesuffix('"')
        elif stripped.startswith("<!-- stack the") and name is not None:
            prompts[name] = stripped.removeprefix("<!-- ").removesuffix(" -->")
            name = None
    return prompts


# Values the C++ sampleCubeStackTrial gives: the golden trials of vla_sim_behaviors/test/test_cube_stack_trial.cpp, and
# trial 23 at seed 0, whose layout is drawn 9 times before the spacing holds, printed from a build of
# cube_stack_trial.cpp.
@pytest.mark.parametrize(
    ("trial_index", "seed", "pair", "placements"),
    [
        (
            0,
            0,
            ("blue", "green"),
            [
                (0.5281189396924197, 0.14991254958771968, 4.440711148879874),
                (0.5884322025837883, 0.09390988978498296, 9.005780407485755),
                (0.5321143358207409, -0.11753503561011229, 74.80121565750997),
            ],
        ),
        (
            1,
            0,
            ("green", "blue"),
            [
                (0.5820171461395504, 0.14201054954803025, 57.398653370885164),
                (0.45435316182243946, 0.0016000624273574127, 44.74942559437963),
                (0.5663645384917858, -0.11602451392336896, 78.990721669634),
            ],
        ),
        (
            0,
            1,
            ("blue", "green"),
            [
                (0.4918343944378798, 0.09072457894575317, 28.74802111640812),
                (0.5956163338327466, 0.040878737757964606, 33.77139403718489),
                (0.5173104755829288, -0.03867272334104406, 84.76452250443823),
            ],
        ),
        (
            2**31 - 1,
            -(2**31),
            ("green", "blue"),
            [
                (0.5776177169784368, -0.0333083162606195, 27.102855555117216),
                (0.4493729193865127, 0.14588365930180966, 34.11512038980131),
                (0.5315697553038266, -0.10149997202525238, 28.202126892633494),
            ],
        ),
        (
            23,
            0,
            ("blue", "red"),
            [
                (0.53861737383195007, 0.061540129064098004, 53.442353721761812),
                (0.52161490105549768, -0.097768308671687301, 6.9071982220346317),
                (0.42130259128890912, 0.15815306896381257, 64.512345344783583),
            ],
        ),
    ],
)
def test_evaluation_trial_matches_the_cpp_sampler(
    trial_index: int,
    seed: int,
    pair: tuple[str, str],
    placements: list[tuple[float, float, float]],
) -> None:
    """The replica gives the C++ sampler's pair and exact doubles, so the excluded trial layouts are the real ones."""
    actual_pair, actual_placements = gen.sample_evaluation_trial(trial_index, seed)
    assert actual_pair == pair
    assert actual_placements == placements


def test_negative_trial_index_is_rejected() -> None:
    """The replica rejects the trial indices the C++ sampler rejects."""
    with pytest.raises(ValueError, match="trial_index is -1"):
        gen.sample_evaluation_trial(-1, 0)


def test_generation_is_deterministic_for_a_seed() -> None:
    """The same seed gives the same layouts, and another seed gives other layouts."""
    first = gen.generate_layouts(5, 7, [])
    assert gen.generate_layouts(5, 7, []) == first
    assert gen.generate_layouts(5, 8, []) != first


def test_committed_files_match_the_generator() -> None:
    """keyframes_generated.xml and the six Objectives are exactly what the script writes with its defaults."""
    _, keyframes_text, objectives = gen.render_outputs(
        gen.DEFAULT_COUNT, gen.DEFAULT_SEED
    )
    assert (
        gen.GENERATED_KEYFRAMES_PATH.read_text() == keyframes_text
    ), "rerun script/generate_cube_stack_layouts.py"
    for filename, text in objectives.items():
        assert (
            gen.OBJECTIVES_DIR / filename
        ).read_text() == text, f"rerun the script to update {filename}"
    committed = {
        path.name
        for path in gen.OBJECTIVES_DIR.glob("record_generated_cube_stack_*.xml")
    }
    assert committed == set(objectives)


def test_each_prompt_has_the_default_count_of_keys(
    generated_keys: list[ET.Element],
) -> None:
    """Every pair gets DEFAULT_COUNT keys named gen_<held>_on_<target>_0 onward, and no name repeats."""
    names = [key.get("name") for key in generated_keys]
    assert len(names) == len(set(names))
    assert len(names) == gen.DEFAULT_COUNT * len(gen.CUBE_PAIRS)
    for held, target in gen.CUBE_PAIRS:
        prefix = f"gen_{held}_on_{target}_"
        assert [name for name in names if name.startswith(prefix)] == [
            f"{prefix}{number}" for number in range(gen.DEFAULT_COUNT)
        ]


def test_generated_layouts_stay_in_the_training_area(
    generated_layouts: np.ndarray,
) -> None:
    """Every cube center and yaw lies in the sampler's area, up to the 6-decimal rounding of the keys."""
    rounding = 1e-6
    assert generated_layouts[:, :, 0].min() >= gen.MIN_X - rounding
    assert generated_layouts[:, :, 0].max() <= gen.MAX_X + rounding
    assert generated_layouts[:, :, 1].min() >= gen.MIN_Y - rounding
    assert generated_layouts[:, :, 1].max() <= gen.MAX_Y + rounding
    assert generated_layouts[:, :, 2].min() >= math.radians(gen.MIN_YAW_DEG) - rounding
    assert generated_layouts[:, :, 2].max() <= math.radians(gen.MAX_YAW_DEG) + rounding


def test_generated_cube_centers_keep_the_minimum_spacing(
    generated_layouts: np.ndarray,
) -> None:
    """Every pair of cube centers is at least MIN_CENTER_SPACING apart, so no two cubes overlap."""
    xy = generated_layouts[:, :, :2]
    for i, j in ((0, 1), (0, 2), (1, 2)):
        gaps = np.linalg.norm(xy[:, i] - xy[:, j], axis=1)
        assert gaps.min() >= gen.MIN_CENTER_SPACING - 2e-6


def test_generated_layouts_avoid_the_eval_and_train_keys_and_the_evaluation_trials(
    generated_layouts: np.ndarray,
) -> None:
    """No generated key is a near-duplicate of an eval_* or train_* key, an evaluation trial, or another generated key."""
    keys = gen.read_keys(gen.KEYFRAMES_PATH)
    eval_keys = [key for key in keys if key.get("name").startswith("eval_")]
    train_keys = [key for key in keys if key.get("name").startswith("train_")]
    assert (len(eval_keys), len(train_keys)) == (150, 360)
    references = {
        "eval": _as_array(
            [
                gen.layout_from_qpos([float(v) for v in k.get("qpos").split()])
                for k in eval_keys
            ]
        ),
        "train": _as_array(
            [
                gen.layout_from_qpos([float(v) for v in k.get("qpos").split()])
                for k in train_keys
            ]
        ),
        "evaluation trial": _as_array(gen.evaluation_trial_layouts()),
    }
    for label, reference in references.items():
        hits = _near_duplicate_pairs(
            generated_layouts, reference, gen.DEFAULT_POS_TOL, gen.DEFAULT_YAW_TOL
        )
        assert (
            len(hits) == 0
        ), f"generated keys near a {label} layout: {hits[:5].tolist()}"
    self_hits = _near_duplicate_pairs(
        generated_layouts, generated_layouts, gen.DEFAULT_POS_TOL, gen.DEFAULT_YAW_TOL
    )
    assert (
        self_hits[:, 0] == self_hits[:, 1]
    ).all(), "two generated keys are near-duplicates"


def test_near_duplicate_index_treats_a_quarter_turn_as_the_same_layout() -> None:
    """A copy turned by almost 90 degrees is a duplicate; a copy moved by 3 cm is not."""
    layout = gen.Layout(
        xy=((0.45, 0.0), (0.5, 0.1), (0.55, -0.1)), yaws=(0.01, 0.5, 1.0)
    )
    index = gen.NearDuplicateIndex(gen.DEFAULT_POS_TOL, gen.DEFAULT_YAW_TOL)
    index.add(layout)
    turned = gen.Layout(xy=layout.xy, yaws=(math.pi / 2 - 0.01, 0.5, 1.0))
    moved = gen.Layout(xy=((0.48, 0.0), (0.5, 0.1), (0.55, -0.1)), yaws=layout.yaws)
    assert index.matches(turned)
    assert not index.matches(moved)


def test_generated_keys_use_the_scene_qpos_format(
    generated_keys: list[ET.Element],
) -> None:
    """Every key shares the default robot pose and ctrl, and gives each cube a unit quaternion about z only."""
    default = next(
        key for key in gen.read_keys(gen.KEYFRAMES_PATH) if key.get("name") == "default"
    )
    robot_qpos = default.get("qpos").split()[: gen.ROBOT_QPOS_SIZE]
    for key in generated_keys:
        qpos = key.get("qpos").split()
        assert len(qpos) == gen.QPOS_SIZE, key.get("name")
        assert qpos[: gen.ROBOT_QPOS_SIZE] == robot_qpos, key.get("name")
        assert key.get("ctrl") == default.get("ctrl"), key.get("name")
        for cube in range(len(gen.COLORS)):
            base = gen.ROBOT_QPOS_SIZE + cube * gen.CUBE_QPOS_SIZE
            _, _, z, qw, qx, qy, qz = qpos[base : base + gen.CUBE_QPOS_SIZE]
            assert (z, qx, qy) == ("0.115000", "0", "0"), key.get("name")
            assert math.isclose(
                float(qw) ** 2 + float(qz) ** 2, 1.0, abs_tol=2e-6
            ), key.get("name")


def test_scene_includes_the_generated_keys_after_the_default_key() -> None:
    """The simulator starts from key 0, so keyframes.xml, whose first key is `default`, is included first."""
    includes = [
        element.get("file")
        for element in ET.parse(SCENE_PATH).getroot().findall("include")
    ]
    assert includes.index("keyframes.xml") < includes.index("keyframes_generated.xml")
    assert gen.read_keys(gen.KEYFRAMES_PATH)[0].get("name") == "default"


@pytest.mark.parametrize(("held", "target"), gen.CUBE_PAIRS)
def test_objective_records_its_prompts_generated_keys(
    held: str, target: str, generated_keys: list[ET.Element]
) -> None:
    """Each wrapper passes its prompt's gen_* keys, in file order, and its prompt, cube frames, and dataset."""
    root = ET.parse(gen.OBJECTIVES_DIR / gen.objective_filename(held, target)).getroot()
    assert root.get("main_tree_to_execute") == gen.objective_name(held, target)
    call = root.find("./BehaviorTree//SubTree[@ID='Record Cube-Stack Layouts']")
    assert call is not None
    prefix = f"gen_{held}_on_{target}_"
    expected_names = [
        key.get("name") for key in generated_keys if key.get("name").startswith(prefix)
    ]
    assert call.get("keyframe_names").split(";") == expected_names
    assert call.get("task") == gen.prompt_for(held, target)
    assert call.get("held_cube_frame") == f"cube_{held}_tf"
    assert call.get("target_cube_frame") == f"cube_{target}_tf"
    assert call.get("dataset_name") == f"cubestack_gen_{held}_on_{target}"
    prompts = [
        key_prompt
        for name, key_prompt in _key_prompts().items()
        if name.startswith(prefix)
    ]
    assert set(prompts) == {gen.prompt_for(held, target)}


def test_layouts_objective_forwards_every_port_to_the_episode() -> None:
    """Record Cube-Stack Layouts declares the ports the wrappers set and loops Record Cube-Stack Episode over them."""
    root = ET.parse(gen.OBJECTIVES_DIR / "record_cube_stack_layouts.xml").getroot()
    ports = {
        port.get("name") for port in root.findall("./TreeNodesModel/SubTree/input_port")
    }
    assert ports == {
        "keyframe_names",
        "task",
        "held_cube_frame",
        "target_cube_frame",
        "dataset_name",
    }
    loop = root.find("./BehaviorTree//Decorator[@ID='LoopString']")
    assert loop.get("queue") == "{keyframe_names}"
    episode = loop.find("SubTree[@ID='Record Cube-Stack Episode']")
    for port in ports - {"keyframe_names"}:
        assert episode.get(port) == f"{{{port}}}"
    assert episode.get("keyframe_name") == loop.get("value")
