# Retained completed-baseline evidence

[`lunar_mapping_baseline.tar.gz`](lunar_mapping_baseline.tar.gz) contains the
accepted completed run (2026-11-11 01:43:21–01:54:33 UTC), not a rejected tuning
candidate. Extract it with `tar -xzf lunar_mapping_baseline.tar.gz` in an empty
directory; `SHA256SUMS.json` records each member's SHA-256.

- `final/map.{pgm,yaml,png}`: original occupancy output. The PNG is byte-identical
  to `../lunar_map.png`; the original YAML loads the PGM with the same placement.
- `final/posegraph.{data,posegraph}`: paired slam_toolbox serialized graph.
- `trajectory.csv`: timestamp-matched truth, live SLAM and fuse odometry poses.
- `segments.jsonl`: all 11 original goals, the explicit tilt stop, clearance
  detour and successful retry; `analysis-output.json` retains coverage/error
  results, including comparison against the earlier acceptance reference.
- `final/graph-{scans.csv,sensor.txt}` and `tf-audit.json`: recorded mapper
  input/transform evidence.
- `config-snapshot/`: effective mapping, navigation, fuse and controller
  configuration and experimental launch wiring, retained as evidence only.
  These files are not loaded by the product launch.
- `beluga/`: saved-map validation metrics, matched correction summary,
  costmap comparison and measured trajectory; see the parent README for scope.

[`clicked_point_validation.json`](clicked_point_validation.json) records the
2026-11-12 bounded real Objective/UI-protocol check, matched-time localization
and TF measurements, concurrent costmap comparisons and failed return/cold-repeat
outcomes. Its correction metrics use sampled TF pairs applied to the same odometry
pose; they are not exact AMCL update intervals. Raw observations and reusable
observer/driver scripts remain in the task evidence store, not product launch.

The independently retained original report, full raw observations, binary
identities, scene snapshots and rejected candidates remain in the task evidence
store. This compact bundle is not a claim that identical timing, trajectories
or startup behavior can be reproduced on a different simulator build or host.
The annotated image is the original mapping-time figure; its “Beluga suitability
unvalidated” caption describes that stage, before the subsequent live validation.
