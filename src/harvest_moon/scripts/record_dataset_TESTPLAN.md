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
   ls ~/user_ws/datasets/smoke_*/
   ```
   Should show `bag/` directory + `session.json`.

5. Inspect the bag:
   ```
   ros2 bag info ~/user_ws/datasets/smoke_*/bag
   ```
   Should list every topic from `DEFAULT_TOPICS` with non-zero message counts.

6. Inspect the session metadata:
   ```
   cat ~/user_ws/datasets/smoke_*/session.json
   ```
   Should show label, timestamp, topics, duration_s, size_mb.

## Things to verify and report back

- **Are all default topics actually publishing?** If the pre-flight check
  warns about missing topics, note which ones — most likely the ZED ones
  need their names corrected (e.g. `image_rect_color` may not be the right
  topic name for the wrapper config we're running).
- **Storage rate**: at default settings, expect ~240–300 MB/s. The 10-second
  smoke test should produce ~2.5–3 GB. If much smaller or much larger,
  something's off (topics missing, or recording more than expected).
- **Clean shutdown**: re-run without `--duration`, let it run a few seconds,
  hit Ctrl+C. It should print "stopping cleanly" and exit within ~15 s.

## Cleanup after smoke test

Smoke test bags are large — delete after confirming things work:
```
rm -rf ~/user_ws/datasets/smoke_*
```
