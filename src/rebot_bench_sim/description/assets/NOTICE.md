# Converted meshes

MuJoCo cannot load COLLADA, so these STL files are conversions of
this package's COLLADA visuals, made with trimesh (pycollada):
each file's scene graph is flattened into the DAE's own frame, in metres, and
split into one binary STL per embedded material. Geometry is otherwise
unchanged; the local DAE files stay the source.

| Files | Source in `description/assets/` | Licence |
| --- | --- | --- |
| `printed/<part>_purple.stl`, `printed/<part>_white.stl` | `printed/<part>.dae` | CERN-OHL-W-2.0, see `description/LICENSE` and `description/NOTICE.md` |

`_purple` takes the embedded colour sRGB (91, 44, 143), `_white` sRGB
(242, 242, 242); `rebot_assets.xml` applies them as MuJoCo materials.
