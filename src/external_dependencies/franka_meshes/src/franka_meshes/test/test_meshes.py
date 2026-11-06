"""Checks the meshes install where package:// references expect them, and hold real geometry."""

import struct
import sys
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
MESHES = PACKAGE / "meshes"

sys.path.insert(0, str(PACKAGE / "script"))

import generate_collision_decomposition as generator  # noqa: E402

# Modified by PickNik Inc.: resolve franka_description the same way the generator does, so this
# test passes both in the upstream workspace and in a workspace that vendors this one.
SUBMODULE = generator.SOURCE_MESHES

# Each source mesh and the number of convex parts committed for it.
SOURCES = {
    "robots/fr3v2/collision/link3.stl": 8,
    "robots/fr3v2/collision/link4.stl": 8,
    "robots/fr3v2/collision/link5.stl": 6,
    "accessories/franka_head_v0_2/collision/franka_head.stl": 3,
    "accessories/fr3_duo_mount_v0_3/collision/fr3_duo_mount.stl": 4,
}
VISUAL_COUNT = 13
STL_HEADER_NOTICE = "Modified by PickNik Inc."

# Float32 round-off in a binary STL, which is what a truly convex part shows.
CONVEX_TOLERANCE = 1e-5
# The decomposition is approximate, so a part may reach slightly past the source mesh.
COVERAGE_TOLERANCE = 0.0015
# How far outside every part a source vertex may sit before it counts as uncovered. Well above
# what the committed parts need, and far below the size of a hole a lost part would leave.
CONTAINMENT_TOLERANCE = 0.00025


def collision_parts():
    return sorted(MESHES.rglob("*_convex_*.stl"))


def visual_meshes():
    return sorted(MESHES.rglob("*_visual.obj"))


def read_stl(path):
    """The 80-byte header and the (normal, v0, v1, v2) tuples of a binary STL."""
    raw = path.read_bytes()
    assert len(raw) > 84, f"{path.name} is too short to be a binary STL"
    count = struct.unpack("<I", raw[80:84])[0]
    assert len(raw) == 84 + 50 * count, (
        f"{path.name} is {len(raw)} bytes, not the {84 + 50 * count} its "
        f"{count} triangles need: a Git LFS pointer or a truncated file"
    )
    triangles = []
    for index in range(count):
        offset = 84 + 50 * index
        values = struct.unpack("<12f", raw[offset : offset + 48])
        triangles.append((values[0:3], values[3:6], values[6:9], values[9:12]))
    return raw[:80], triangles


def stl_vertices(path):
    return [vertex for _, *corners in read_stl(path)[1] for vertex in corners]


def bounds(vertices):
    return [
        (min(v[axis] for v in vertices), max(v[axis] for v in vertices))
        for axis in range(3)
    ]


def face_planes(path):
    """One convex part as unit face planes and its bounds, for a point-inside test."""
    planes = []
    for normal, corner, _, _ in read_stl(path)[1]:
        length = sum(value * value for value in normal) ** 0.5
        if length == 0.0:
            continue
        unit = [value / length for value in normal]
        planes.append((unit, sum(unit[axis] * corner[axis] for axis in range(3))))
    return planes, bounds(stl_vertices(path))


def distance_outside(vertex, planes):
    """How far a vertex sits outside a convex part: at most zero when it is inside."""
    worst = 0.0
    for unit, plane in planes:
        beyond = sum(unit[axis] * vertex[axis] for axis in range(3)) - plane
        worst = max(worst, beyond)
    return worst


def test_every_mesh_is_committed():
    assert len(collision_parts()) == sum(SOURCES.values())
    assert len(visual_meshes()) == VISUAL_COUNT


@pytest.mark.parametrize(
    "mesh", collision_parts() + visual_meshes(), ids=lambda p: p.name
)
def test_install_puts_the_mesh_where_package_urls_expect_it(mesh):
    """A package:// reference resolves to share/franka_meshes, at the source-tree path."""
    share = pytest.importorskip("ament_index_python.packages")
    try:
        installed = Path(share.get_package_share_directory("franka_meshes"))
    except share.PackageNotFoundError:
        pytest.skip("franka_meshes is not installed in this environment")
    relative = mesh.relative_to(PACKAGE)
    assert (
        installed / relative
    ).is_file(), f"package://franka_meshes/{relative} is missing"


@pytest.mark.parametrize("part", collision_parts(), ids=lambda p: p.name)
def test_collision_part_is_convex_and_marked(part):
    """MuJoCo hulls every collision mesh, so a part is only useful while it is convex."""
    header, triangles = read_stl(part)
    assert STL_HEADER_NOTICE in header.rstrip(b"\0").decode("ascii", "replace")
    vertices = {vertex for _, *corners in triangles for vertex in corners}
    for normal, corner, _, _ in triangles:
        length = sum(value * value for value in normal) ** 0.5
        if length == 0.0:
            continue
        unit = [value / length for value in normal]
        plane = sum(unit[axis] * corner[axis] for axis in range(3))
        outside = max(
            sum(unit[axis] * vertex[axis] for axis in range(3)) - plane
            for vertex in vertices
        )
        assert outside < CONVEX_TOLERANCE, f"{part.name} is not convex"


@pytest.mark.parametrize("mesh", visual_meshes(), ids=lambda p: p.name)
def test_visual_mesh_parses(mesh):
    """Catches a Git LFS pointer file, which parses as no geometry at all."""
    vertices = faces = 0
    highest = 0
    for line in mesh.read_text().splitlines():
        if line.startswith("v "):
            vertices += 1
        elif line.startswith("f "):
            faces += 1
            for token in line.split()[1:]:
                index = int(token.split("/")[0])
                assert index >= 1, f"{mesh.name} has a non-positive vertex index"
                highest = max(highest, index)
    assert vertices and faces, f"{mesh.name} holds no geometry: a Git LFS pointer file?"
    assert highest <= vertices, f"{mesh.name} indexes a vertex it does not define"


def test_generator_targets_the_same_source_meshes():
    assert generator.SOURCE_MESHES == SUBMODULE
    assert generator.MAX_PARTS.keys() == SOURCES.keys()


@pytest.mark.parametrize("source, count", SOURCES.items())
def test_generator_writes_the_committed_parts(source, count):
    """Binds the generator's output paths to the committed files, so a rerun overwrites them."""
    assert count <= generator.MAX_PARTS[source], "more parts than the generator's cap"
    expected = generator.part_paths(source, count)
    stem = Path(source).stem
    assert sorted(expected) == sorted(MESHES.rglob(f"{stem}_convex_*.stl"))


@pytest.mark.parametrize("source", SOURCES)
def test_generator_resolves_its_input(source):
    if not (SUBMODULE / source).is_file():
        pytest.skip("the franka_description submodule is not checked out")
    assert generator.source_path(source).is_file()


@pytest.mark.parametrize("source, count", SOURCES.items())
def test_every_source_vertex_lies_inside_a_part(source, count):
    """Catches a hole the bounds check cannot see, such as a lost interior part."""
    if not (SUBMODULE / source).is_file():
        pytest.skip("the franka_description submodule is not checked out")
    stem = Path(source).stem
    parts = [face_planes(path) for path in sorted(MESHES.rglob(f"{stem}_convex_*.stl"))]
    worst = 0.0
    culprit = None
    for vertex in set(stl_vertices(SUBMODULE / source)):
        nearest = min(
            (
                distance_outside(vertex, planes)
                for planes, box in parts
                # Only a part whose bounds reach the vertex can contain it.
                if all(
                    box[axis][0] - CONTAINMENT_TOLERANCE
                    <= vertex[axis]
                    <= box[axis][1] + CONTAINMENT_TOLERANCE
                    for axis in range(3)
                )
            ),
            default=float("inf"),
        )
        if nearest > worst:
            worst, culprit = nearest, vertex
    reach = (
        "beyond the bounds of"
        if worst == float("inf")
        else f"{worst * 1000:.3f} mm outside"
    )
    assert worst < CONTAINMENT_TOLERANCE, (
        f"{stem}: the source vertex {culprit} sits {reach} every one of the "
        f"{len(parts)} committed parts, so the decomposition has a hole there"
    )


@pytest.mark.parametrize("source, count", SOURCES.items())
def test_parts_cover_their_source_mesh(source, count):
    """Binds each part set to the submodule mesh it was decomposed from."""
    if not (SUBMODULE / source).is_file():
        pytest.skip("the franka_description submodule is not checked out")
    stem = Path(source).stem
    parts = sorted(MESHES.rglob(f"{stem}_convex_*.stl"))
    assert len(parts) == count
    union = [vertex for part in parts for vertex in stl_vertices(part)]
    for axis, (expected, measured) in enumerate(
        zip(bounds(stl_vertices(SUBMODULE / source)), bounds(union))
    ):
        for want, got in zip(expected, measured):
            assert abs(want - got) < COVERAGE_TOLERANCE, (
                f"{stem}: the parts reach {got:.4f} on axis {axis} where the "
                f"source mesh reaches {want:.4f}"
            )
