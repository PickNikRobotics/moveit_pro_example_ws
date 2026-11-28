# Cold-start proof

The complete native MuJoCo stack cold-started at
2026-10-14T03:59:15.464082474Z using an existing image with digest
`sha256:ea7c92a8a832e5a9b651e79c37ed124b36e1b32f634494849747a4b79d733fc6`.
It used private source/build/install copies, an isolated ROS domain and no
hardware devices. Native health reported `ok`/`Idle`, with both cameras
publishing RGB and depth. The physical bench remained home with all seven
motors disabled; power and CAN remained on.

![Cold-start registration: physical images left, native renders right](registration-sheet.png)

The left images are the fresh 03:05:38 UTC camera observation, undistorted and
recentered to the native simulator intrinsics. Scene views use a 640-by-400
crop of the 640-by-480 native frame. Green marks decoded board corners; red
crosses project fixture CAD features through the simulator's published camera
truth. Colors, lighting, foam relief, the holder and surrounding clutter are
not photometric replicas.

| Check | Result |
| --- | --- |
| Scene board raster versus physical corners | 16 shared corners, 0.343 px RMS, 0.838 px max |
| Wrist board raster versus physical corners | 21 shared corners, 1.301 px RMS, 1.511 px max |
| Jig CAD projection versus measured apertures | 4 features, 0.591 px RMS, 1.028 px max |
| Bracket CAD projection versus annotated apertures | 3 features, 4.130 px RMS, 4.579 px max |
| Six native fixture sites versus frozen fitted CAD poses | Max translation error 1.25e-16 m |
| Native camera sites versus separately loaded model at published joints | Translation below 1.3e-16 m; rotation matrix difference below 1.5e-15 |
| Compiled MJCF and real configuration loader tests | 18 passed |
| Fresh physical scene stability after proof | 4 aperture centers changed 0.174 px RMS, 0.224 px max |

The fixture check reads `jig_origin`, `bracket_origin`, `peg_1_tip`,
`peg_2_tip`, `hole_1_center` and `hole_2_center` from the native
`MujocoSystem` TF stream. It does not substitute requested poses or URDF
estimates for simulator truth. The near-zero numerical errors prove that the
cold-loaded model contains the fitted transforms. Physical accuracy remains
limited by the measurements described in the package README.

The twin first started in its zeroed rest keyframe. Its existing trajectory
controller then replayed the camera observation joint configuration, returning
status 4 / error code 0. The replay clamps the small negative measured J2/J3
angles to their simulated lower limits; the resulting wrist camera differs
from the observation by 0.328 mm and 0.110 degrees. This explains part of the
wrist board residual. No additional physical movement occurred during proof.

These are registration residuals against the same observation used to fit the
scene. In particular, the wrist residual is measured at the single pose that
fitted the D435 mount. That fit is 50.5 mm and 6.8 degrees from the nominal
mount and absorbs the provisional base's 4.3 degree tilt, so the residual says
nothing about the wrist view at other arm poses (see the package README). The
known source geometry supplies occluded features. The board render
check independently decodes the raster and caught a mirrored texture during
development; the corrected scene decodes normally after the cold restart.

The fixtures are static visual meshes with collision disabled. Neither the
controller's successful camera replay nor exact site truth proves successful
peg insertion, contacts, grasping, masses or friction. No stack/unstack or
peg-in-hole episode collection was performed.

[cold-proof.json](cold-proof.json) contains the numeric comparisons, native
joint positions, intrinsics and qualification. Mesh source hashes and bounds
are in the package's geometry provenance manifest.
