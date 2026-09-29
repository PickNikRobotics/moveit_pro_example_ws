# Factory sim config

A MoveIt Pro configuration for a Fanuc LR Mate 200iD.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## Model licensing and provenance

The ONNX model files in [`models/`](models/) are exports of Meta's SAM 2.1 model and are distributed under the [Apache License 2.0](models/LICENSE), with upstream attribution in [`models/NOTICE`](models/NOTICE). If you redistribute the models or derivative works, include the license text and the NOTICE file with them. All other files in this package are governed by the package's [BSD 3-Clause License](LICENSE).

The models were exported by PickNik in December 2025 from Meta's SAM 2.1 `sam2.1_hiera_large` checkpoint ([facebookresearch/sam2](https://github.com/facebookresearch/sam2)) using `torch.onnx.export` (PyTorch 2.6.0, per the ONNX producer metadata), split into three graphs (image encoder, prompt encoder, decoder) so image embeddings can be cached across prompts. No training or fine-tuning was performed. The export scripts and the exact checkpoint revision were not preserved.

| File | SHA-256 |
| --- | --- |
| `sam2.1_hiera_l_image_encoder.onnx` | `cbdbe1e1ad3b616985d0f9c071a5948f8f8342d8e55e98857159405678b70cec` |
| `sam2.1_prompt_encoder.onnx` | `eb2130a99f74bb2f6a430c0f1fe6af291b45ead4bf0beee715330c3def901206` |
| `sam2.1_decoder.onnx` | `e11609a8d694f2186f5974cf0d86e1588dbbb126088c4a7849adbc546b284e3a` |

## Articulated tools with AttachURDF

This section is a reproduction for a feature request. It is not a working
example yet.

A tool changer swaps tools on one flange. Some of those tools have joints of
their own, for example a gripper's fingers. Each such tool also has its own
driver, which publishes the tool's joint states. The goal is that attaching
the tool with `AttachURDF` makes MoveIt Pro treat its joints like the robot's
own: in the robot state, in TF, in the 3D view and in collision checking, all
following the tool's joint states.

### What is here

| Path | What |
| --- | --- |
| `description/articulated_tools/parallel_gripper.urdf` | Parallel-jaw gripper. Two prismatic fingers, the right one mimics the left. One actuated joint. 0 m open, 0.04 m closed. |
| `description/articulated_tools/angular_gripper.urdf` | Angular gripper. Two independent revolute fingers. 0 rad open, 0.38 rad closed. |
| `bags/gripper_joint_states/` | 12 s of `sensor_msgs/JointState` on `/joint_states`, 50 Hz per gripper, both grippers. Each 6 s cycle holds open for 2 s, closes over 1 s, holds closed for 2 s, opens over 1 s. |
| `scripts/generate_gripper_joint_state_bag.py` | Writes that bag. |
| Objective `Attach Parallel Gripper` | `AddURDF` at `tool0`, then `AttachURDF` to `tool0`. |
| Objective `Attach Angular Gripper` | The same for the angular gripper. |
| Objective `Detach Articulated Grippers` | `DetachURDF` and `RemoveURDFFromScene` for both, and removes the test part. |
| Objective `Check Gripper Finger Collision` | Adds a 30 x 40 x 40 mm test part between the fingers, 70 to 110 mm out from the flange. Then collision checks the current state against the current planning scene with `ValidateTrajectory`. Succeeds if nothing touches. |

Both URDFs carry a `ros2_control` block with `mock_components/GenericSystem`
hardware. The blocks were checked outside MoveIt Pro, each with a standalone
jazzy `ros2_control_node`, `robot_state_publisher` and joint_state_broadcaster:
each block loads, activates, and publishes the joint names that are in the
bag.

The fingers touch the test part from 0.025 m (parallel) or 0.227 rad
(angular). Fully open, they clear it by 25 mm (parallel) or 14 mm (angular).
The Objectives are skipped in `test/objectives_integration_test.py`, because
they need the bag and leave a tool or the test part in the scene.

### How to run it

1. `moveit_pro build`, then `moveit_pro run -c factory_sim`.
2. Run `Check Gripper Finger Collision` with no tool attached. It succeeds.
   This is the control: the test part touches nothing on the bare arm.
3. Run `Attach Parallel Gripper`.
4. Open a shell in the runtime container with `moveit_pro shell -s runtime`
   and play the bag, looping:

   ```bash
   ros2 bag play --loop $USER_WS/src/factory_sim/bags/gripper_joint_states
   ```

5. Watch the 3D view, `/joint_states`, TF and the planning scene.
6. To leave the fingers closed, stop the loop and play the closed slice once.
   Then run `Check Gripper Finger Collision`.

   ```bash
   ros2 bag play $USER_WS/src/factory_sim/bags/gripper_joint_states --start-offset 3.5 --playback-duration 0.5
   ```

   For open fingers, use `--start-offset 0.5` instead.
7. Repeat from step 3 with `Attach Angular Gripper`.
8. `Detach Articulated Grippers` cleans up.

### Expected behaviour

With a gripper attached and the bag playing:

- The finger joints join the robot state. They appear in the robot state that
  `/get_planning_scene` returns and in the UI's joint list, with the bag's
  values.
- Every gripper link has a TF frame under `tool0`. The finger frames move with
  the bag.
- The 3D view draws the gripper from its URDF, and the fingers open and close
  with the bag.
- Collision checking uses the current finger positions, per link. With the
  fingers closed, `Check Gripper Finger Collision` fails with a finger against
  `gripper_test_part`. With them open, it succeeds.
- `DetachURDF` takes the finger joints and frames away again.
- Optionally, the tool's `ros2_control` block is brought up when the tool is
  attached, so its driver runs and its controllers can be loaded, and it is
  shut down on detach. At the least, `AddURDF` should say it ignored the block
  instead of dropping it without a word.

### What MoveIt Pro 10.1.0 actually does

Run on 2026-09-29 with MoveIt Pro 10.1.0 (`picknikciuser/moveit-pro:10.1.0-jazzy`),
following the steps above for both grippers.

| With a gripper attached and the bag playing | Result |
| --- | --- |
| Collision shapes in `/get_planning_scene` | Follow the bag. Exact during the holds, within one 0.2 s sample while moving. |
| `/monitored_planning_scene` | Its diffs carry the moving shapes, about 4 per second. |
| Collision check, fingers closed | Fails: `Colliding objects: gripper_test_part - parallel_gripper` (or `angular_gripper`). |
| Collision check, fingers open | Succeeds. |
| Finger joints in the robot state (`/get_planning_scene`, UI Joint Monitor) | No. Only `joint_1` to `joint_6`. |
| TF frames for the gripper links | None. |
| 3D view | Draws the collision shapes in one colour. The fingers do not move with the bag. The view keeps the finger positions from its last full redraw, for example a page reload. |
| Collision rules | One ACM entry, named after the tool. A contact names the tool, never a finger. |
| `ros2_control` block | Ignored. No hardware component, no controller, and no log message. |
| Finger states on `/parallel_gripper/joint_states` | Ignored. The capability only listens on `joint_states`. |
| `DetachURDF`, `RemoveURDFFromScene` | Work. |

So the tool's collision geometry does follow `/joint_states`, as the
`AttachURDF` description says. Nothing else about the tool's joints reaches
MoveIt Pro.

The capability behind these Behaviors is `URDFPlanningSceneCapability` in
move_group. Its code is the same in 10.0.0, 10.1.0 and `v10.1`. It parses the
URDF with urdfdom, which skips `<ros2_control>` without a message. It builds
the tool a private robot model, attaches the whole tool as one collision body,
and copies matching `joint_states` positions into that private model.
