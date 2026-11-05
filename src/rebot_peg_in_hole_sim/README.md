# reBot peg-in-hole scene

This overlay starts the native MuJoCo bench twin with the printed peg jig and
the gray bracket in their observed positions. The bracket is upright in its
holder, separate from the jig. The fixture geometry comes from the existing
[factory_sim printing models](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/main/src/picknik_accessories/printing/factory_sim/3mf).
No geometry scan is required.

Select `rebot_peg_in_hole_sim` as the MoveIt Pro configuration after building
the workspace. The package generates its MJCF and URDF together from
[scene_snapshot.json](config/scene_snapshot.json). It inherits the bench robot,
controllers and Objectives from `rebot_bench_sim`, with a MuJoCo-only hardware
description. It adds no physical motion or collection Objectives.

## Registration

The snapshot was captured on 2026-10-14 at 03:05:38 UTC. World coordinates are
metres in the printed frame on the mat: red +X, green +Y, blue +Z; z = 0 is the
mat top. Fresh OAK-D and aimed D435 images observe the same 7-by-5 ChArUco
board (35 mm squares, 26 mm markers, DICT_4X4_50). Their corner reprojection
errors are 0.277 px RMS over 18 corners and 0.256 px over 21 corners.

Four visible CAD apertures register the jig to the table plane with 1.27 mm
RMS residual. The upright bracket uses an aligned-depth face plane (1.18 mm
RMS over 1,340 points) and three manually identified CAD apertures (2.32 mm
RMS, 4.06 px reprojection). The desk webcam confirms the upright arrangement;
its attempted intrinsic fit was rejected and supplies no metric pose.

| Feature | World position, metres |
| --- | --- |
| Peg 1 tip | 0.214815, 0.078176, 0.087000 |
| Peg 2 tip | 0.219793, 0.142985, 0.087000 |
| Hole 1 face center | 0.391853, 0.269452, 0.071935 |
| Hole 2 face center | 0.456628, 0.266848, 0.067195 |

These are CAD feature positions transformed by the fitted fixture poses, not
four independent probe measurements. Printed-axis registration, manual
centers, partial occlusion and depth systematic errors leave several
millimetres of absolute uncertainty, potentially greater. The retained robot
base registration is provisional; its z axis is tilted 4.3 degrees from the mat
normal. The wrist mount is fitted from that base, the nominal arm kinematics
and a single camera observation, so it absorbs their combined error: the
fitted `d435_mount_joint` is 50.5 mm and 6.8 degrees from the base config's
nominal mount, which would put the optical frame 54.5 mm from the observed one.
The simulated D435 therefore matches the physical camera only at the
observation joints; at other arm poses, the wrist view can be off by
centimetres. The fit corrects only the camera. The twin gripper still sits at
the provisional base plus nominal kinematics, while the fixtures are
registered from the cameras, so the gripper's position relative to the pegs
and holes is unvalidated at every pose, the observation joints included, and
can be off by centimetres. Separating base from mount error needs a multi-pose
hand-eye calibration, which this snapshot does not include.

The mat texture has 12.7 mm pitch; its phase is illustrative. An aligned-depth
foam sample puts the approximate front at y = 0.4324 m (5.79 mm plane RMS over
26,008 points), and the virtual mat extends to that edge. The opposite edge,
length, foam relief and robot placement remain approximate bench context; the
black holder is not modeled. The synthetic board records the observed
registration, rather than reusing an earlier board pose.

The held capture's mat depth comparison has 5.63 mm RMS residual for the OAK
scaled depth and 3.03 mm for D435 aligned depth; median world heights are
+7.78 mm and -5.01 mm. The OAK scale ratio is 0.960064. These discrepancies
are retained in the snapshot as calibration evidence, rather than treated as
millimetre-accurate table or wrist calibration.

## Cameras and scope of proof

Both simulated cameras render 640-by-480 RGB and registered depth, with matching
optical-frame sites and native `CameraInfo`. MuJoCo's native publisher emits a
centered principal point and square pixels. Compare physical images after
undistorting and recentering to that intrinsic matrix; crop 40 rows at each end
of the scene render for comparison to the physical 640-by-400 OAK stream.
The D435 depth topic is `/wrist_mounted_camera/aligned_depth_to_color/image_raw`.

The fixtures are fixed visual meshes with collision disabled. MuJoCo's convex
mesh collision would close the concave apertures. Their simulator sites expose
fixture origins, peg tips and hole centers for a cold-start truth check.
Successful rendering, stable robot servos and exact site coordinates establish
that the intended scene was loaded; they do not validate contact, insertion,
fixture masses, friction, grasping or episode collection.

## Source geometry and validation

[provenance.json](description/assets/provenance.json) records source and converted
mesh SHA-256 values, native metre bounds and triangle counts. Regenerate the
binary STL files with:

```sh
python3 src/rebot_peg_in_hole_sim/scripts/convert_factory_meshes.py \
  src/picknik_accessories/printing/factory_sim/3mf \
  src/rebot_peg_in_hole_sim/description/assets
```

The converter accepts only the existing metre-unit, single-mesh, identity-build
sources. It preserves their assembly coordinates, including the post offset;
the two jig posts use the bracket's corresponding CAD aperture centers.

```sh
colcon build --packages-up-to rebot_peg_in_hole_sim
colcon test --packages-select rebot_peg_in_hole_sim
colcon test-result --all --verbose
```

The tests load the compiled model, compare fixture truth and calibrated camera
poses, check URDF/MJCF agreement at multiple joint configurations, verify mesh
bounds and provenance, and exercise the real configuration loader.

See [the cold-start render sheet](docs/cold-proof.md) for observed native-stack
registration and its limits.
