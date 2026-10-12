# Converted meshes

MuJoCo cannot load COLLADA, so these STL files are conversions of
`rebot_bench_base_config`'s COLLADA visuals, made with trimesh (pycollada):
each file's scene graph is flattened into the DAE's own frame, in metres, and
split into one binary STL per embedded material. Geometry is otherwise
unchanged; the base package's files stay the source.

| Files | Source in `rebot_bench_base_config/description/assets/` | Licence |
| --- | --- | --- |
| `printed/<part>_purple.stl`, `printed/<part>_white.stl` | `printed/<part>.dae` | CERN-OHL-W-2.0, see that package's `description/LICENSE` and `description/NOTICE.md` |
| `realsense/d435.stl` | `realsense/d435.dae` | Apache-2.0, see that package's `description/assets/realsense/LICENSE` and `UPSTREAM_NOTICE.txt` |

`_purple` takes the embedded colour sRGB (91, 44, 143), `_white` sRGB
(242, 242, 242); `rebot_assets.xml` applies them as MuJoCo materials.
