# Measure the VO Result

The Objective runs the experiment; these optional scripts measure the VO output against MuJoCo ground truth. They are not part of the Objective.

## Customer Workflow

1. Start `lunar_sim` normally with VO enabled and your model bundle configured. Connect the MoveIt Pro Desktop App.
2. Open an interactive terminal in the running Runtime's ROS environment, using the same ROS domain and DDS configuration. Run:

   ```bash
   python3 "$USER_WS/src/lunar_sim/test/vo_measurement/measure_vo_square.py"
   ```

3. Wait for **Ready**, then run `Estimate Visual Odometry Square` in the Desktop App.
4. After the Objective finishes, press **Enter** in the measurement terminal. The script prints JSON metrics and acceptance violations. A failed or cancelled run may report insufficient route coverage; inspect the Objective result separately.

The script subscribes to VO, ground truth, commands, and TF. It never launches a Runtime, sends motion commands, starts an Objective, or cancels one. Ctrl-C stops only measurement. Keep other Objectives and manual driving inactive during recording so they cannot contaminate the measured route.

The default recording limit is 300 seconds; set `--timeout SECONDS` to change it. This does not extend the Objective's tracking or motion timeouts. A slow CPU may fail to maintain tracking: measuring the output does not make inference faster.

## Files

- `measure_vo_square.py`: read-only recorder for an existing Runtime.
- `trajectory_metrics.py`: compares source-stamped VO with camera-frame simulation truth, without fitting scale or the trajectory.
- `tf_lookup.py`: waits for the physical camera mounting transform while spinning the listener.
- `test_trajectory_metrics.py`, `test_tf_lookup.py`: developer regressions for the measurement helpers; neither runs inference nor belongs to the customer workflow.

Metrics include physical-camera distance coverage, end-point position drift, orientation error in radians, position-error RMSE/p95, and output gaps. These are measurements, not the Objective's return value. Save terminal output or record a bag if you need retained results.
