# franka_meshes

**PickNik mesh assets for Franka robots, owned by this workspace rather than by the vendored upstream description.**

Two sets of files, both derived from Franka Robotics' `franka_description` geometry and both authored by PickNik:

| Set | Where | What it is for |
| --- | --- | --- |
| `*_convex_NN.stl` under `meshes/*/collision/` | 29 files, five source meshes | a convex decomposition, so MuJoCo stops colliding a concave link as its filled-in hull |
| `*_visual.obj` under `meshes/*/visual/` | 13 files | OBJ copies of the visual meshes, because MuJoCo has no COLLADA decoder |

This is not a MoveIt Pro configuration package: it has no `config/config.yaml` and is never named by `MOVEIT_CONFIG_PACKAGE`. It installs mesh files and nothing else.

## Paths

A file keeps `franka_description`'s own directory. The file name changes, because a file here is derived from an upstream mesh rather than a copy of it: a collision part adds a `_convex_NN` suffix, and a visual mesh becomes `_visual.obj`.

```
package://franka_description/meshes/robots/fr3v2/collision/link3.stl        <- upstream, unchanged
package://franka_meshes/meshes/robots/fr3v2/collision/link3_convex_00.stl   <- one convex part of it

package://franka_description/meshes/robots/fr3v2/visual/link3.dae           <- upstream, unchanged
package://franka_meshes/meshes/robots/fr3v2/visual/link3_visual.obj         <- the same mesh as OBJ
```

So a reference changes the package name and the file name, never the directory. Keep that rule when you add a file, and do not flatten into an `assets/` folder. `test/test_meshes.py` checks that every mesh installs at its source-tree path, so a `package://` reference to it resolves.

## Why the collision parts exist

MuJoCo collides a mesh as its convex hull, so a link whose collision mesh is far from convex collides as the filled-in shape. The Franka head was the worst case: its hull is about five times the mesh's own volume, which matters for contact and for head-to-arm self-collision. Each part here is already convex, so MuJoCo's own hulling changes nothing and the union of the parts follows the real shape. Effective collision volume, as a multiple of the true mesh volume:

| Source mesh | Single hull | Parts | With parts |
| --- | --- | --- | --- |
| `accessories/franka_head_v0_2/collision/franka_head.stl` | 5.16x | 3 | 1.106x |
| `robots/fr3v2/collision/link5.stl` | 1.79x | 6 | 1.015x |
| `accessories/fr3_duo_mount_v0_3/collision/fr3_duo_mount.stl` | 1.45x | 4 | 0.998x |
| `robots/fr3v2/collision/link3.stl` | 1.35x | 8 | 1.027x |
| `robots/fr3v2/collision/link4.stl` | 1.35x | 8 | 1.024x |

A URDF needs none of this: MoveIt collides the mesh itself, so it keeps loading the single upstream mesh. Only an MJCF should load the parts, each as its own `geom` on the same body.

## Regenerating the collision parts

`script/generate_collision_decomposition.py` writes them. It reads the five source meshes from the `franka_description` submodule, which it never writes, and writes each part into this package at the source mesh's own relative path, so a rerun overwrites the committed files in place. Its seed and decomposition settings are fixed, so a rerun reproduces them byte for byte.

Run every command below from the workspace root, the same directory the checks further down use:

```bash
python3 src/franka_meshes/script/generate_collision_decomposition.py --paths   # resolve the paths only
python3 src/franka_meshes/script/generate_collision_decomposition.py --check   # committed files against a fresh run
python3 src/franka_meshes/script/generate_collision_decomposition.py           # rewrite them
```

Run it by hand when a source mesh changes, never at build or launch time. It needs `trimesh`, `coacd` and `scipy`, which are deliberately not workspace dependencies, so it is not installed with the package; `--paths` needs none of them and checks only that the submodule is checked out and the output paths resolve.

`--check` fails, and names the file, both when a committed part no longer matches a fresh run and when a part is left over from an earlier run that produced more parts. A plain rewrite deletes those leftovers. After a rerun, run the tests below: they check the parts are still convex and still cover their source mesh.

## Why it is a package and not files in the submodule

`src/external_dependencies/franka_description` is a submodule of Franka Robotics' own repository. A parent repository cannot add files inside a submodule, and a PickNik file placed inside any vendored upstream tree is liable to be removed by a vendored-refresh tool whose discard path runs `git restore` and `git clean -fd` over that directory, because upstream does not have the file. A package in this workspace's own tree is outside that blast radius.

## Licensing

The meshes are derived from Apache-2.0 geometry, so the Apache License, Version 2.0 applies to them; the text is in `meshes/LICENSE-franka_description`. The package's own build, test and documentation files are BSD 3-Clause, as in the repository `LICENSE`. `NOTICE` holds PickNik's attribution for the derivation work, Franka Robotics' copyright notice, the provenance commits, and the source mesh of every file. Read it before you copy anything out of this package.

## Checking it without a robot

```bash
colcon build --paths src/franka_meshes --build-base /tmp/b --install-base /tmp/i
colcon test  --paths src/franka_meshes --build-base /tmp/b --install-base /tmp/i --event-handlers console_direct+
```

`test/test_meshes.py` needs no robot, no ROS graph and no network. It checks that every mesh installs where a `package://` reference expects it, that each collision part really is convex and carries the PickNik STL header notice, that every vertex of a source mesh lies inside one of its parts and the part union keeps the source mesh's bounds, and that no file is a Git LFS pointer instead of geometry. The meshes are stored in Git LFS: run `git lfs install` before you clone, or those last checks fail.
