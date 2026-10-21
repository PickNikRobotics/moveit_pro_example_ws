# FashionStar reader dependencies

The workspace Dockerfile installs the pinned `motorbridge-smart-servo==0.0.4`
SDK from `config/leader-requirements.txt` into the Python used by ROS. This is
the FashionStar SDK used by Seeed's LeRobot leader plugin. The native wheel
provides the serial transport and declares no Python runtime dependencies;
there is no LeRobot, Torch, NumPy replacement, or separate Python environment.
Linux x86_64 and aarch64 wheels are published for Python >=3.9.

[MotorBridge Smart Servo 0.0.4](https://pypi.org/project/motorbridge-smart-servo/0.0.4/)
is published by tianrking under MIT. The installed wheel includes license
metadata and a CycloneDX SBOM for the native extension. It is installed as an
unmodified dependency rather than copying a platform-specific binary into this
robot configuration.

The reader follows the stored-origin and nearest-turn angle conventions in
[Seeed's plugin at 8514914a645f6f5cb23e941d12ceaf85bc6e5c79](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102/tree/8514914a645f6f5cb23e941d12ceaf85bc6e5c79).
That source is Copyright (c) 2026 Jack Shao, MIT licensed, with no NOTICE.
The reader uses its published format and angle convention; it does not ship the
LeRobot wrapper. The existing bench calibration values are preserved in
`config/leader_calibration/rebot_arm_102_leader.json`, installed with the package
and selected by `rebot_leader_calibration_file` in YAML. This avoids depending on
the host cache being mounted in the backend container. Another installation
must select its own existing file; the reader requires `--calibration-file` and
reads it without modification. Keep that file available to the runtime user.

Seeed's calibration writes origin points into the servos and records zero
`homing_offset`/`drive_mode`; the reader validates these and all seven ids and
finite, ordered ranges. Nonzero legacy offset/drive conventions refuse startup
rather than guessing their units or direction. The stored +/-90 degree ranges
are placeholders in Seeed's calibration routine: just as the original bridge,
the reader unwraps and clamps with the configured full motor ranges, preserving
approved motion. Startup never recalibrates, writes origins, resets turn
counters, or saves parameters. It pings the seven leader servos and selects
unloaded mode before taking raw angle samples. Failed reads emit an error,
never a cached pose. Closing or failed startup always releases the serial bus.
