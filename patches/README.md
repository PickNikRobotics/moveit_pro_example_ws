# Patches for external src/ packages

  The packages under `src/pylon-ros-camera` and `src/zed-ros2-wrapper` come from
  upstream repos (Basler, Stereolabs) and are not tracked in this repo. Local
  fixes that we need are captured here as patch files.

  ## Applying on a fresh checkout

  After cloning the upstream repos into `src/`, run from the repo root:

      cd src/pylon-ros-camera && git apply ../../patches/pylon-ros-camera-disable-blaze.patch
      cd ../zed-ros2-wrapper && git apply ../../patches/zed-ros2-wrapper-parameter-value-int.patch

  ## Regenerating a patch

  If you make new changes inside one of the external repos:

      (cd src/pylon-ros-camera && git diff) > patches/pylon-ros-camera-disable-blaze.patch
