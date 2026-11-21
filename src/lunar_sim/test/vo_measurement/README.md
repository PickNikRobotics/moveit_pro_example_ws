# Measure the VO Result

`VO Estimation Run` estimates and publishes camera motion; it does not visualize or save the trajectory or calculate drift. These optional scripts measure the VO output against MuJoCo ground truth. They are not part of the Objective.

## Customer Workflow

1. Start `lunar_sim` normally with VO enabled and your model bundle configured. Connect the MoveIt Pro Desktop App.
2. Open an interactive terminal in the running Runtime's ROS environment, using the same ROS domain and DDS configuration. Run:

   ```bash
   python3 "$USER_WS/src/lunar_sim/test/vo_measurement/measure_vo_square.py"
   ```

3. Wait for **Ready**, then run `VO Estimation Run` in the Desktop App.
4. After the Objective finishes, press **Enter** in the measurement terminal. The script prints JSON metrics and acceptance violations. A failed or cancelled run may report insufficient route coverage; inspect the Objective result separately.

The script subscribes to VO, ground truth, commands, and TF. It never launches a Runtime, sends motion commands, starts an Objective, or cancels one. Ctrl-C stops only measurement. Keep other Objectives and manual driving inactive during recording so they cannot contaminate the measured route.

Add `--plot vo_route.svg` to save an overhead ground-truth/VO overlay and a finish close-up from the same paired samples used for metrics. The plot uses simulation world XY, aligns VO at the first motion pose only, and does not fit scale or render terrain. Existing plot files are not overwritten. Redirect stdout to `vo_metrics.json` to retain metrics. See the [setup guide and example plot](https://docs.picknik.ai/how_to/mobile_navigation/stereo_visual_odometry/#measure-and-save-the-result).

The default recording limit is 300 seconds; set `--timeout SECONDS` to change it. This does not extend the Objective's tracking or motion timeouts. A slow CPU may fail to maintain tracking: measuring the output does not make inference faster.

## Files

- `measure_vo_square.py`: read-only recorder for an existing Runtime.
- `trajectory_metrics.py`: compares source-stamped VO with camera-frame simulation truth, without fitting scale or the trajectory.
- `trajectory_plot.py`: saves the paired world-frame camera paths and a finish close-up as SVG without extra plotting dependencies.
- `tf_lookup.py`: waits for the physical camera mounting transform while spinning the listener.

Metrics include physical-camera distance coverage, end-point position drift, orientation error in radians, position-error RMSE/p95, and output gaps. These are measurements, not the Objective's return value. The optional SVG saves the plotted paths; JSON saves metrics, not raw samples.
