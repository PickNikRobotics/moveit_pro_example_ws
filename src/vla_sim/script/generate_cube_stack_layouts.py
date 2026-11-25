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

"""Generate cube layouts for recording scripted-oracle demonstrations beyond the 60 training layouts per prompt.

Every layout is drawn from the same area, spacing, and yaw range as the evaluation trials' sampler (sampleCubeStackTrial in vla_sim_behaviors/src/cube_stack_trial.cpp). A layout is dropped when every cube is within --pos-tol and --yaw-tol of the same cube in an `eval_*` or `train_*` key of keyframes.xml, an evaluation trial at seed 0, or an earlier generated layout.

Writes the keyframes to description/mujoco/keyframes_generated.xml, one `Record Generated Cube-Stack <Held> On <Target>` Objective per prompt that records them, and, with --layouts-out, one JSON line per layout in the train_layouts.jsonl schema, keyed by keyframe name instead of episode_index because a discarded attempt shifts the episode numbers. Rerun it after changing the area, the seed, or the count, so the keyframes and the Objectives stay in sync.
"""

import argparse
import json
import math
import random
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PACKAGE_DIR = Path(__file__).resolve().parent.parent
KEYFRAMES_PATH = PACKAGE_DIR / "description" / "mujoco" / "keyframes.xml"
GENERATED_KEYFRAMES_PATH = (
    PACKAGE_DIR / "description" / "mujoco" / "keyframes_generated.xml"
)
OBJECTIVES_DIR = PACKAGE_DIR / "objectives"

DEFAULT_SEED = 1
DEFAULT_COUNT = 300
DEFAULT_POS_TOL = 0.02
DEFAULT_YAW_TOL = 0.1

# These constants, the pair order, and sample_evaluation_trial replicate cube_stack_trial.cpp. Keep them in sync with
# it: test_generate_cube_stack_layouts.py pins values the C++ sampler produces.
MIN_X = 0.42
MAX_X = 0.60
MIN_Y = -0.16
MAX_Y = 0.16
MIN_YAW_DEG = 0.0
MAX_YAW_DEG = 90.0
MIN_CENTER_SPACING = 0.07
COLORS = ("red", "green", "blue")
CUBE_PAIRS = (
    ("blue", "green"),
    ("green", "blue"),
    ("red", "green"),
    ("green", "red"),
    ("red", "blue"),
    ("blue", "red"),
)

# The seed the evaluation Objectives sample trials at (objectives/sample_a_cube_stacking_trial.xml), and how many of
# its trials, from trial_index 0, the generated layouts avoid.
EVALUATION_SEED = 0
EVALUATION_TRIAL_COUNT = 300

CUBE_CENTER_Z = 0.115
ROBOT_QPOS_SIZE = 15
CUBE_QPOS_SIZE = 7
QPOS_SIZE = ROBOT_QPOS_SIZE + len(COLORS) * CUBE_QPOS_SIZE
# Turning a cube by a quarter turn leaves the same layout.
YAW_PERIOD = math.pi / 2

_UINT32 = 0xFFFFFFFF
_MT_STATE_SIZE = 624


@dataclass(frozen=True)
class Layout:
    """Cube centers in mj_world (meters) and yaws (radians), indexed red, green, blue."""

    xy: tuple[tuple[float, float], ...]
    yaws: tuple[float, ...]


@dataclass(frozen=True)
class GeneratedLayout:
    """A generated layout, the prompt it is recorded with, and its keyframe name."""

    keyframe: str
    held: str
    target: str
    layout: Layout


def prompt_for(held: str, target: str) -> str:
    """The instruction the dataset labels an episode with."""
    return f"stack the {held} cube on the {target} cube"


def slug_for(held: str, target: str) -> str:
    """The `<held>_on_<target>` part of keyframe, file, and dataset names."""
    return f"{held}_on_{target}"


def _seed_seq_generate(values: list[int], count: int) -> list[int]:
    """std::seed_seq::generate, which the C++ standard fixes word for word."""
    words = [0x8B8B8B8B] * count
    seed_count = len(values)
    tail = (
        11
        if count >= 623
        else (
            7
            if count >= 68
            else 5 if count >= 39 else 3 if count >= 7 else (count - 1) // 2
        )
    )
    p = (count - tail) // 2
    q = p + tail
    total = max(seed_count + 1, count)
    for k in range(total):
        mixed = words[k % count] ^ words[(k + p) % count] ^ words[(k - 1) % count]
        r1 = (1664525 * (mixed ^ (mixed >> 27))) & _UINT32
        if k == 0:
            r2 = r1 + seed_count
        elif k <= seed_count:
            r2 = r1 + k % count + values[k - 1]
        else:
            r2 = r1 + k % count
        r2 &= _UINT32
        words[(k + p) % count] = (words[(k + p) % count] + r1) & _UINT32
        words[(k + q) % count] = (words[(k + q) % count] + r2) & _UINT32
        words[k % count] = r2
    for k in range(total, total + count):
        summed = (
            words[k % count] + words[(k + p) % count] + words[(k - 1) % count]
        ) & _UINT32
        r3 = (1566083941 * (summed ^ (summed >> 27))) & _UINT32
        r4 = (r3 - k % count) & _UINT32
        words[(k + p) % count] ^= r3
        words[(k + q) % count] ^= r4
        words[k % count] = r4
    return words


def _mt19937_from_seed_seq(values: list[int]) -> random.Random:
    """An MT19937 engine in the state `std::mt19937 engine{std::seed_seq{values...}}` starts in."""
    state = _seed_seq_generate([value & _UINT32 for value in values], _MT_STATE_SIZE)
    if state[0] & 0x80000000 == 0 and not any(state[1:]):
        state[0] = 0x80000000
    engine = random.Random()
    engine.setstate((3, (*state, _MT_STATE_SIZE), None))
    return engine


def _draw_unit_interval(engine: random.Random) -> float:
    """drawUnitInterval: a double in [0, 1) from two 32-bit draws."""
    high_bits = engine.getrandbits(32) >> 5
    low_bits = engine.getrandbits(32) >> 6
    return (high_bits * 67108864.0 + low_bits) / 9007199254740992.0


def _draw_in_range(engine: random.Random, low: float, high: float) -> float:
    return low + _draw_unit_interval(engine) * (high - low)


def _centers_are_spaced_apart(xy: list[tuple[float, float]]) -> bool:
    for i in range(len(xy)):
        for j in range(i + 1, len(xy)):
            dx = xy[i][0] - xy[j][0]
            dy = xy[i][1] - xy[j][1]
            if dx * dx + dy * dy < MIN_CENTER_SPACING * MIN_CENTER_SPACING:
                return False
    return True


def _draw_layout(draw) -> tuple[list[tuple[float, float]], list[float]]:
    """Cube centers and yaws in degrees, in sampleCubeStackTrial's draw order. `draw(low, high)` gives one number."""
    while True:
        xy = []
        for _ in COLORS:
            x = draw(MIN_X, MAX_X)
            y = draw(MIN_Y, MAX_Y)
            xy.append((x, y))
        if _centers_are_spaced_apart(xy):
            break
    yaws_deg = [draw(MIN_YAW_DEG, MAX_YAW_DEG) for _ in COLORS]
    return xy, yaws_deg


def sample_evaluation_trial(
    trial_index: int, seed: int
) -> tuple[tuple[str, str], list[tuple[float, float, float]]]:
    """The (held, target) pair and per-cube (x, y, yaw_deg) that sampleCubeStackTrial gives."""
    if trial_index < 0:
        raise ValueError(f"trial_index is {trial_index}, but it must be 0 or greater.")
    engine = _mt19937_from_seed_seq([seed, trial_index])
    xy, yaws_deg = _draw_layout(lambda low, high: _draw_in_range(engine, low, high))
    pair = CUBE_PAIRS[trial_index % len(CUBE_PAIRS)]
    return pair, [(x, y, yaw) for (x, y), yaw in zip(xy, yaws_deg)]


def evaluation_trial_layouts(
    seed: int = EVALUATION_SEED, count: int = EVALUATION_TRIAL_COUNT
) -> list[Layout]:
    """The layouts of evaluation trials 0 to count - 1 at seed."""
    layouts = []
    for trial_index in range(count):
        _, placements = sample_evaluation_trial(trial_index, seed)
        layouts.append(
            Layout(
                xy=tuple((x, y) for x, y, _ in placements),
                yaws=tuple(math.radians(yaw) for _, _, yaw in placements),
            )
        )
    return layouts


def layout_from_qpos(qpos: list[float]) -> Layout:
    """The cube layout of a scene qpos, whose cube quaternions (w, x, y, z) turn about z only."""
    if len(qpos) != QPOS_SIZE:
        raise ValueError(f"qpos has {len(qpos)} values, but the scene has {QPOS_SIZE}.")
    xy = []
    yaws = []
    for cube in range(len(COLORS)):
        base = ROBOT_QPOS_SIZE + cube * CUBE_QPOS_SIZE
        x, y, _, qw, _, _, qz = qpos[base : base + CUBE_QPOS_SIZE]
        xy.append((x, y))
        yaws.append(2.0 * math.atan2(qz, qw))
    return Layout(xy=tuple(xy), yaws=tuple(yaws))


def read_keys(path: Path) -> list[ET.Element]:
    """The <key> elements of a MuJoCo keyframe file, in file order."""
    return ET.parse(path).getroot().findall("./keyframe/key")


def read_reference_keys(path: Path) -> tuple[list[str], str, list[Layout]]:
    """The shared robot qpos prefix and ctrl of the `default` key, and the layouts of the `eval_*` and `train_*` keys."""
    keys = read_keys(path)
    by_name = {key.get("name"): key for key in keys}
    if "default" not in by_name:
        raise ValueError(f"{path} has no `default` key.")
    robot_qpos = by_name["default"].get("qpos").split()[:ROBOT_QPOS_SIZE]
    ctrl = by_name["default"].get("ctrl")
    layouts = []
    for key in keys:
        name = key.get("name")
        if not re.fullmatch(r"(eval|train)_\d+", name):
            continue
        qpos = key.get("qpos").split()
        if qpos[:ROBOT_QPOS_SIZE] != robot_qpos or key.get("ctrl") != ctrl:
            raise ValueError(
                f"Key `{name}` in {path} does not share the `default` robot pose."
            )
        layouts.append(layout_from_qpos([float(value) for value in qpos]))
    if not layouts:
        raise ValueError(f"{path} has no `eval_*` or `train_*` keys.")
    return robot_qpos, ctrl, layouts


class NearDuplicateIndex:
    """Finds a stored layout in which every cube is within pos_tol and yaw_tol of the same cube of a query."""

    def __init__(self, pos_tol: float, yaw_tol: float) -> None:
        self._pos_tol = pos_tol
        self._yaw_tol = yaw_tol
        self._xy = np.zeros((0, len(COLORS), 2))
        self._yaws = np.zeros((0, len(COLORS)))

    def add(self, layout: Layout) -> None:
        self._xy = np.concatenate([self._xy, np.asarray(layout.xy)[None]])
        self._yaws = np.concatenate([self._yaws, np.asarray(layout.yaws)[None]])

    def matches(self, layout: Layout) -> bool:
        position_gap = np.linalg.norm(self._xy - np.asarray(layout.xy)[None], axis=2)
        yaw_gap = np.abs(self._yaws - np.asarray(layout.yaws)[None]) % YAW_PERIOD
        yaw_gap = np.minimum(yaw_gap, YAW_PERIOD - yaw_gap)
        close = (position_gap < self._pos_tol) & (yaw_gap < self._yaw_tol)
        return bool(np.any(np.all(close, axis=1)))


def generate_layouts(
    count: int,
    seed: int,
    excluded: list[Layout],
    pos_tol: float = DEFAULT_POS_TOL,
    yaw_tol: float = DEFAULT_YAW_TOL,
) -> list[GeneratedLayout]:
    """`count` layouts for each pair in CUBE_PAIRS order, none a near-duplicate of `excluded` or of each other."""
    if count < 0:
        raise ValueError(f"count is {count}, but it must be 0 or greater.")
    # random.Random.random() keeps its sequence for a given seed across Python versions.
    engine = random.Random(seed)
    index = NearDuplicateIndex(pos_tol, yaw_tol)
    for layout in excluded:
        index.add(layout)
    generated = []
    for held, target in CUBE_PAIRS:
        for number in range(count):
            while True:
                xy, yaws_deg = _draw_layout(
                    lambda low, high: low + engine.random() * (high - low)
                )
                layout = Layout(
                    xy=tuple(xy), yaws=tuple(math.radians(yaw) for yaw in yaws_deg)
                )
                if not index.matches(layout):
                    break
            index.add(layout)
            name = f"gen_{slug_for(held, target)}_{number}"
            generated.append(
                GeneratedLayout(keyframe=name, held=held, target=target, layout=layout)
            )
    return generated


def keyframe_qpos(robot_qpos: list[str], layout: Layout) -> str:
    """The scene qpos of a layout, written like the keys in keyframes.xml."""
    values = list(robot_qpos)
    for (x, y), yaw in zip(layout.xy, layout.yaws):
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        values += [
            f"{x:.6f}",
            f"{y:.6f}",
            f"{CUBE_CENTER_Z:.6f}",
            f"{qw:.6f}",
            "0",
            "0",
            f"{qz:.6f}",
        ]
    return " ".join(values)


def render_keyframes(
    generated: list[GeneratedLayout], robot_qpos: list[str], ctrl: str, seed: int
) -> str:
    """keyframes_generated.xml, with the prompt of each key in a comment after it, as in keyframes.xml."""
    lines = [
        "<mujoco>",
        f"  <!-- Written by script/generate_cube_stack_layouts.py with seed {seed}; rerun it instead of editing this file. Each prompt's `gen_<held>_on_<target>_*` keys are the layouts its `Record Generated Cube-Stack <Held> On <Target>` Objective records. The qpos layout is the one keyframes.xml describes. -->",
        "  <keyframe>",
    ]
    for entry in generated:
        lines += [
            "    <key",
            f'      name="{entry.keyframe}"',
            f'      qpos="{keyframe_qpos(robot_qpos, entry.layout)}"',
            f'      ctrl="{ctrl}"',
            "    />",
            f"    <!-- {prompt_for(entry.held, entry.target)} -->",
        ]
    lines += ["  </keyframe>", "</mujoco>", ""]
    return "\n".join(lines)


def objective_name(held: str, target: str) -> str:
    return f"Record Generated Cube-Stack {held.capitalize()} On {target.capitalize()}"


def objective_filename(held: str, target: str) -> str:
    return f"record_generated_cube_stack_{slug_for(held, target)}.xml"


def render_objective(held: str, target: str, keyframes: list[str]) -> str:
    """The Objective that records one prompt's generated layouts into cubestack_gen_<held>_on_<target>."""
    name = objective_name(held, target)
    dataset = f"cubestack_gen_{slug_for(held, target)}"
    description = (
        f"Record {len(keyframes)} oracle demonstrations of stacking the {held} cube on the {target} cube from"
        " generated layouts into one Trainer dataset and convert it to a LeRobot dataset."
    )
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<root
  BTCPP_format="4"
  main_tree_to_execute="{name}"
>
  <!-- Written by script/generate_cube_stack_layouts.py; rerun it instead of editing this file. keyframe_names lists this prompt's keys in description/mujoco/keyframes_generated.xml. -->
  <BehaviorTree
    ID="{name}"
    _description="{description}"
    _favorite="false"
  >
    <Control ID="Sequence" name="root">
      <SubTree
        ID="Record Cube-Stack Layouts"
        _collapsed="true"
        keyframe_names="{";".join(keyframes)}"
        task="{prompt_for(held, target)}"
        held_cube_frame="cube_{held}_tf"
        target_cube_frame="cube_{target}_tf"
        dataset_name="{dataset}"
      />
    </Control>
  </BehaviorTree>

  <TreeNodesModel>
    <SubTree ID="{name}">
      <MetadataFields>
        <Metadata runnable="true" />
        <Metadata subcategory="Application - Data Recording" />
      </MetadataFields>
    </SubTree>
  </TreeNodesModel>
</root>
"""


def render_outputs(
    count: int,
    seed: int,
    keyframes_path: Path = KEYFRAMES_PATH,
    pos_tol: float = DEFAULT_POS_TOL,
    yaw_tol: float = DEFAULT_YAW_TOL,
) -> tuple[list[GeneratedLayout], str, dict[str, str]]:
    """The generated layouts, the keyframe file text, and each Objective file's text by file name."""
    robot_qpos, ctrl, reference_layouts = read_reference_keys(keyframes_path)
    excluded = reference_layouts + evaluation_trial_layouts()
    generated = generate_layouts(count, seed, excluded, pos_tol, yaw_tol)
    objectives = {}
    for held, target in CUBE_PAIRS:
        names = [
            entry.keyframe
            for entry in generated
            if (entry.held, entry.target) == (held, target)
        ]
        objectives[objective_filename(held, target)] = render_objective(
            held, target, names
        )
    return generated, render_keyframes(generated, robot_qpos, ctrl, seed), objectives


def layout_record(entry: GeneratedLayout, seed: int) -> dict:
    """A train_layouts.jsonl line for a generated layout, with its keyframe name and the seed for episode_index."""
    return {
        "keyframe": entry.keyframe,
        "held": COLORS.index(entry.held),
        "target": COLORS.index(entry.target),
        "prompt": prompt_for(entry.held, entry.target),
        "positions": [[x, y, CUBE_CENTER_Z] for x, y in entry.layout.xy],
        "yaws": list(entry.layout.yaws),
        "max_yaw": math.radians(MAX_YAW_DEG),
        "seed": seed,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--count", type=int, default=DEFAULT_COUNT, help="layouts per prompt"
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--pos-tol", type=float, default=DEFAULT_POS_TOL, help="meters, per cube center"
    )
    parser.add_argument(
        "--yaw-tol",
        type=float,
        default=DEFAULT_YAW_TOL,
        help="radians, per cube, modulo 90 degrees",
    )
    parser.add_argument(
        "--keyframes",
        type=Path,
        default=KEYFRAMES_PATH,
        help="the eval_* and train_* keys",
    )
    parser.add_argument("--keyframes-out", type=Path, default=GENERATED_KEYFRAMES_PATH)
    parser.add_argument("--objectives-dir", type=Path, default=OBJECTIVES_DIR)
    parser.add_argument(
        "--layouts-out", type=Path, help="also write the layouts as JSON lines"
    )
    args = parser.parse_args()

    generated, keyframes_text, objectives = render_outputs(
        args.count, args.seed, args.keyframes, args.pos_tol, args.yaw_tol
    )
    args.keyframes_out.write_text(keyframes_text)
    for filename, text in objectives.items():
        (args.objectives_dir / filename).write_text(text)
    if args.layouts_out is not None:
        with args.layouts_out.open("w") as stream:
            for entry in generated:
                stream.write(json.dumps(layout_record(entry, args.seed)) + "\n")
    print(
        f"Wrote {len(generated)} layouts to {args.keyframes_out} and {len(objectives)} Objectives."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
