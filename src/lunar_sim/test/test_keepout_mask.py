#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#    * Neither the name of the PickNik Inc. nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Contract tests for lunar_sim's generated semantic keep-out map."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
GENERATOR_PATH = PACKAGE / "description" / "generate_keepout_mask.py"
MAPS = PACKAGE / "maps"
SPEC = importlib.util.spec_from_file_location("generate_keepout_mask", GENERATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


def _committed():
    metadata = yaml.safe_load((MAPS / "lunar_keepout_mask.yaml").read_text())
    image = np.array(Image.open(MAPS / metadata["image"]))
    return image, metadata


def _pixel_for_world(image, metadata, world_x, world_y):
    spawn, _ = GENERATOR._spawn_pose()
    map_x, map_y = world_x - spawn[0], world_y - spawn[1]
    col = math.floor((map_x - metadata["origin"][0]) / metadata["resolution"])
    row_from_bottom = math.floor(
        (map_y - metadata["origin"][1]) / metadata["resolution"]
    )
    height, width = image.shape
    row = height - 1 - row_from_bottom
    assert 0 <= row < height and 0 <= col < width
    return row, col


def _is_keepout(image, metadata, world_x, world_y):
    return image[_pixel_for_world(image, metadata, world_x, world_y)] == 0


def test_committed_mask_matches_generator() -> None:
    """The checked-in mask remains byte-for-byte derived from scene geometry."""
    generated_image, generated_metadata = GENERATOR.generate()
    committed_image, committed_metadata = _committed()
    assert committed_metadata == generated_metadata
    np.testing.assert_array_equal(committed_image, generated_image)


def test_mask_uses_binary_trinary_encoding() -> None:
    """The PGM/YAML pair maps black to lethal and white to free in Nav2."""
    image, metadata = _committed()
    assert set(np.unique(image)) == {0, 254}
    assert metadata == {
        "image": "lunar_keepout_mask.pgm",
        "mode": "trinary",
        "resolution": 0.05,
        "origin": [-19.7, -20.5, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }


@pytest.mark.parametrize(
    ("world_x", "world_y"),
    [
        (-1.0, -5.25),  # camera mast base
        (7.0, 6.6),  # landing pad and lander
        (-2.0, 6.6),  # habitat
        (-11.93, 6.5),  # tilted solar array
        (12.02, 3.0),  # antenna
        (-5.5, 6.6),  # vertical solar mast
        (-5.0, 8.4),  # cargo
        (-11.4, -4.0),  # power unit
        (-4.0, -11.82),  # logistics
        (13.0, 0.0),  # east berm safety strip
        (-3.10, 17.0),  # east access-road wall safety strip
    ],
)
def test_fixed_hazards_and_wall_strips_are_keepout(world_x, world_y) -> None:
    """Representative fixed structures and authored wall feet are lethal."""
    image, metadata = _committed()
    assert _is_keepout(image, metadata, world_x, world_y)


@pytest.mark.parametrize(
    ("world_x", "world_y"),
    [
        (-0.30, 0.50),  # spawn
        (5.0, -5.0),  # ordinary natural terrain
        (-4.5, 17.0),  # access lane
        (-4.5, 21.0),  # open road end and far horizon
        (10.54, -9.5),  # garage interior
    ],
)
def test_traversable_areas_stay_free(world_x, world_y) -> None:
    """Spawn, natural terrain, road, and garage interior remain traversable."""
    image, metadata = _committed()
    assert not _is_keepout(image, metadata, world_x, world_y)


def test_each_fixed_structure_polygon_is_covered_after_clearance_growth() -> None:
    """Every selected source polygon contributes lethal cells to the generated mask."""
    image, metadata = _committed()
    for polygon in GENERATOR.fixed_structure_polygons():
        center = polygon.mean(axis=0)
        assert _is_keepout(image, metadata, *center)


def test_structure_clearance_extends_to_rover_radius_but_not_beyond() -> None:
    """The mast mask grows by the configured all-yaw rover clearance, without unbounded growth."""
    image, metadata = _committed()
    # The mast base's east edge is world x=-0.65. Its full y span avoids the narrow post above it.
    edge_x, world_y = -0.65, -5.60
    clearance = GENERATOR._footprint_clearance_m()
    assert _is_keepout(image, metadata, edge_x + clearance - 0.05, world_y)
    assert not _is_keepout(image, metadata, edge_x + clearance + 0.10, world_y)


@pytest.mark.parametrize("world_x", [12.7, -5.7, -3.3])
def test_wall_strips_grow_by_rover_clearance(world_x) -> None:
    """Berm and both road walls reserve clearance while leaving the lane open."""
    image, metadata = GENERATOR.generate()
    clearance = GENERATOR._footprint_clearance_m()
    direction = -1 if world_x in (12.7, -3.3) else 1
    world_y = 0.0 if world_x == 12.7 else 17.0
    assert _is_keepout(
        image, metadata, world_x + direction * (clearance - 0.05), world_y
    )
    assert not _is_keepout(
        image, metadata, world_x + direction * (clearance + 0.10), world_y
    )


@pytest.mark.parametrize(
    "world_x,world_y", [(-20.01, 0), (20, 0), (0, -20.01), (0, 24)]
)
def test_world_pixel_rejects_outside_mask(world_x, world_y) -> None:
    """Points outside any mask edge cannot wrap to another image pixel."""
    image, metadata = _committed()
    with pytest.raises(AssertionError):
        _is_keepout(image, metadata, world_x, world_y)


@pytest.mark.parametrize("kind", ["cylinder", "capsule"])
@pytest.mark.parametrize(
    "orientation", ['euler="0 1.5707963267948966 0"', 'fromto="-1 0 0 1 0 0"']
)
def test_horizontal_round_geom_projection(kind, orientation) -> None:
    """Horizontal cylinders include their length and capsules include end caps."""
    body = ET.fromstring(
        f'<body><geom type="{kind}" size="0.2 1" {orientation}/></body>'
    )
    polygon = GENERATOR._body_geom_polygons(body)[0]
    extent_x = 1.2 if kind == "capsule" else 1.0
    np.testing.assert_allclose(polygon.min(axis=0), [-extent_x, -0.2], atol=0.001)
    np.testing.assert_allclose(polygon.max(axis=0), [extent_x, 0.2], atol=0.001)


@pytest.mark.parametrize(
    "orientation",
    [
        'fromto="0 0 0 0 0 0"',
        'quat="1 0 0 0"',
        'axisangle="0 1 0 1"',
        'xyaxes="1 0 0 0 1 0"',
        'zaxis="1 0 0"',
    ],
)
def test_round_geom_rejects_invalid_or_unsupported_orientation(orientation) -> None:
    """Unsupported orientations and degenerate axes fail instead of under-masking."""
    body = ET.fromstring(
        f'<body><geom name="hazard" type="cylinder" size="0.2 1" {orientation}/></body>'
    )
    with pytest.raises(ValueError, match="hazard"):
        GENERATOR._body_geom_polygons(body)


@pytest.mark.parametrize("kind", ["cylinder", "capsule"])
@pytest.mark.parametrize("roll", [0.0, math.pi / 4])
def test_vertical_and_tilted_round_geom_extent(kind, roll) -> None:
    """Vertical and rolled round geoms retain the analytic projected extent."""
    body = ET.fromstring(
        f'<body><geom type="{kind}" size="0.2 1" euler="{roll} 0 0"/></body>'
    )
    polygon = GENERATOR._body_geom_polygons(body)[0]
    extent_y = math.sin(roll) + 0.2 * (1.0 if kind == "capsule" else math.cos(roll))
    np.testing.assert_allclose(polygon.min(axis=0), [-0.2, -extent_y], atol=0.001)
    np.testing.assert_allclose(polygon.max(axis=0), [0.2, extent_y], atol=0.001)
