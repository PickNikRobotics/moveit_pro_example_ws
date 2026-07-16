# VO/VSLAM evaluation harness

Evaluation-only ROS 2 stereo visual-odometry harness for `moveit_pro#20110`.
It preserves the measured Python ALIKED + LightGlue path while establishing a
ROS contract reusable by later candidates and a production C++ implementation.

## Contract

Required inputs are synchronized rectified `sensor_msgs/Image` and
`sensor_msgs/CameraInfo` on `/stereo/{left,right}/{image_rect,camera_info}`.
Calibration and metric baseline come from the two `CameraInfo.p` matrices.
Outputs are `/vo/odometry` and `/vo/diagnostics`. The node never publishes TF;
authoritative `odom -> base_link` ownership belongs to `moveit_pro#20111`.

Odometry covariance is explicitly unavailable (all entries are `NaN`, with
`covariance_status=unknown` in diagnostics) and must not be fused directly.
Diagnostic confidence v0 reports measured latency, keypoints, stereo/temporal matches,
PnP points/inliers/ratio, reprojection residuals, unsynchronized messages, state, and
failure.
`unsynchronized_messages` is the current received-minus-synchronized backlog/drop proxy;
queued messages can temporarily increase it, so it is not an exact cumulative drop count.

## Runtimes

- `tensorrt_fp32_tf32` (default): corrected resize-1024 practical target.
- `tensorrt_fp32_strict`: TensorRT parity reference.
- `eager`: PyTorch reference/fallback.
- `tensorrt_fp16`: optional; not the quality default.

TensorRT modes require a compatible fixed-shape ALIKED engine, CUDA, TensorRT,
PyTorch, and LightGlue. Eager requires PyTorch and LightGlue. Missing dependencies
or engines fail explicitly; there is no fake fallback. TensorRT engines are executable
artifacts and must come from a trusted source. Each engine requires a JSON sidecar that
binds `schema_version`, `engine_sha256`, `precision`, `preprocess_resize`, and
`score_threshold`; the adapter verifies all fields before deserialization.

## Run

```bash
ros2 launch vo_vslam_evaluation_harness evaluation.launch.py
ros2 run vo_vslam_evaluation_harness tartanair_stereo_playback --ros-args \
  -p dataset_root:=/path/to/sequence
ros2 launch vo_vslam_evaluation_harness record_evaluation.launch.py \
  output:=vo_evaluation_bag
```

Playback expects `image_left/*.png`, `image_right/*.png`, `timestamps.txt`, and
`pose_left.txt` with one `x y z qx qy qz qw` row per frame. Playback intrinsics,
baseline, frame rate, and frame IDs are ROS parameters and must match the data.

## Verify

```bash
colcon test --packages-select vo_vslam_evaluation_harness
python3 scripts/smoke_runtime_adapter.py \
  --runtime fp32_tf32 --engine /path/to/aliked.engine \
  --engine-metadata /path/to/aliked.engine.json \
  --left /path/to/left.png --right /path/to/right.png
python3 scripts/run_sequence_smoke.py \
  --runtime fp32_tf32 --engine /path/to/aliked.engine \
  --engine-metadata /path/to/aliked.engine.json \
  --dataset-root /path/to/sequence --output /path/to/summary.json \
  --fx 320 --fy 320 --cx 320 --cy 240 --baseline-m 0.25
```

The launch integration test publishes synchronized images and `CameraInfo`, then
verifies source timestamps, unknown covariance, and explicit missing-engine failure
diagnostics. The non-ROS sequence smoke uses deterministic index time for tracker
diagnostics; it does not substitute generated timing for the ROS playback contract.

## Limitations

Frame-to-frame stereo PnP only: no bundle adjustment, loop closure, relocalization,
IMU/wheel/lidar fusion, calibrated covariance, or authoritative TF. Optional
sensor topics are recorded for later evaluation but are not consumed yet.
