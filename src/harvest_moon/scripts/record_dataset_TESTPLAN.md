# `record_dataset.py` — Smoke Test Plan

First-time test plan for `record_dataset.py`. Run this with the system fully up
(Teensy in RUNNING mode, cameras streaming via `moveit_pro run`).

## Steps

1. Start the system in one shell:
   ```
   moveit_pro run
   ```
   Wait for the cameras to come up (Basler topics publishing, ZED node running).

2. Open a second shell into the running container:
   ```
   moveit_pro shell
   ```

3. Run a 10-second timed capture:
   ```
   python3 ~/user_ws/src/harvest_moon/scripts/record_dataset.py --label smoke --duration 10
   ```

   Expected output:
   - "Checking topics are publishing..." (no missing-topic warnings if defaults are right)
   - Session directory printed
   - Periodic `[ X.X s ]  Y MB` updates every 5 s
   - "Stopped after ~10 s" + bag size summary

4. Confirm the session contents:
   ```
   ls /mnt/ssd/datasets/smoke_*/
   ```
   Should show `bag/` directory + `session.json`.

5. Inspect the bag:
   ```
   ros2 bag info /mnt/ssd/datasets/smoke_*/bag
   ```
   Should list every topic from `DEFAULT_TOPICS` with non-zero message counts.

6. Inspect the session metadata:
   ```
   cat /mnt/ssd/datasets/smoke_*/session.json
   ```
   Should show label, timestamp, topics, duration_s, size_mb.

## Things to verify and report back

- **Did the ZED node come up cleanly?** This run uses
  `depth_mode: 'ULTRA'` (changed from `'NONE'`). Watch the ZED launch
  log for any TensorRT or model-loading errors — `ULTRA` should NOT
  need TensorRT, but if something's off the ZED node will fail at
  startup.
- **Are all default topics actually publishing?** The recorder
  pre-flight check will warn if any are missing. Likely candidates if
  any are wrong: the depth/pointcloud/confidence/disparity/IMU topic
  names. ZED wrapper conventions vary by version — note any warnings.
- **Storage rate**: at full default settings (NATIVE HD1200), expect
  **~470 MB/s**. A 10-second smoke test should produce ~4.5 GB. If
  much smaller, some ZED stream isn't publishing. If much larger,
  something's recording more than expected.
- **Did the Jetson keep up?** With ZED depth + pointcloud at 15 Hz,
  CPU could saturate. Check `ros2 topic hz /zed_x/zed_node/point_cloud/cloud_registered`
  during the run — should be near 15 Hz. If lower, drop the ZED to
  `depth_mode: 'QUALITY'` or `'PERFORMANCE'`.
- **Clean shutdown**: re-run without `--duration`, let it run a few
  seconds, hit Ctrl+C. It should print "stopping cleanly" and exit
  within ~15 s.

## If a topic warning fires for a ZED topic

The exact ZED topic names depend on `zed_wrapper` version. To list
what's actually publishing:
```
ros2 topic list | grep zed
```
Compare against `DEFAULT_TOPICS` in `record_dataset.py`. Update the
list if names differ.

## Cleanup after smoke test

Smoke test bags are ~4.5 GB for a 10-second run (full ZED depth +
pointcloud at native HD1200). Delete after confirming things work:
```
rm -rf /mnt/ssd/datasets/smoke_*
```

## After the smoke test

Before relaunching, remember to **rebuild** the workspace so the YAML
change takes effect:
```
cd ~/user_ws
colcon build --packages-select harvest_moon
source install/setup.bash
```
Then `moveit_pro run` to start with the new ZED config.
