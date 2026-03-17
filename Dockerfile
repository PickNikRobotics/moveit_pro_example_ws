# Docker image for extending MoveIt Pro with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Pro release to build on top of.
ARG MOVEIT_PRO_BASE_IMAGE=picknikciuser/moveit-studio:${MOVEIT_DOCKER_TAG:-main}-${MOVEIT_ROS_DISTRO:-humble}
ARG USERNAME=moveit-pro-user
ARG USER_UID=1000
ARG USER_GID=1000

##################################################
# Starting from the specified MoveIt Pro release #
##################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_PRO_BASE_IMAGE} AS base

# Create a non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    # Handle pre-existing GID/UID in base image (common with Jazzy/24.04)
    existing_user=$(getent passwd $USER_UID | cut -d: -f1 || true) && \
    existing_group=$(getent group $USER_GID | cut -d: -f1 || true) && \
    if [ -n "$existing_group" ] && [ "$existing_group" != "${USERNAME}" ]; then \
      groupmod -n ${USERNAME} "$existing_group"; \
    elif [ -z "$existing_group" ]; then \
      groupadd --gid $USER_GID ${USERNAME}; \
    fi && \
    if [ -n "$existing_user" ] && [ "$existing_user" != "${USERNAME}" ]; then \
      usermod -l ${USERNAME} -d /home/${USERNAME} "$existing_user"; \
    elif [ -z "$existing_user" ]; then \
      useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME}; \
    fi && \
    usermod -s /bin/bash ${USERNAME} && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    cp -r /etc/skel/. /home/${USERNAME} && \
    mkdir -p \
      /home/${USERNAME}/.ccache \
      /home/${USERNAME}/.config \
      /home/${USERNAME}/.ignition \
      /home/${USERNAME}/.colcon \
      /home/${USERNAME}/.ros && \
    chown -R $USER_UID:$USER_GID /home/${USERNAME} /opt/overlay_ws/

# Add user to dialout group to enable communication with serial USB devices (gripper, FTS, ...)
# Add user to video group to enable communication with cameras
RUN usermod -aG dialout,video ${USERNAME}

# Add user to the render group for GPU access (AMD ROCm / OpenGL)
# Use host GID for /dev/kfd — check `stat -c '%g' /dev/kfd` on your host.
RUN (getent group render > /dev/null 2>&1 && groupmod -g 992 render || groupadd -g 992 render) && \
    usermod -a -G render ${USERNAME}

# Add user to the realtime group to enable RT limits
RUN (getent group realtime > /dev/null 2>&1 || groupadd realtime) && \
    usermod -a -G realtime ${USERNAME}

# Install ROCm 7.2 runtime for AMD GPU acceleration (OpenCL + HIP + MIGraphX)
# Auto-detects Ubuntu codename (jammy for 22.04, noble for 24.04)
# hadolint ignore=DL3008,DL3009
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME") && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
      wget gnupg2 && \
    mkdir -p --mode=0755 /etc/apt/keyrings && \
    wget -qO - https://repo.radeon.com/rocm/rocm.gpg.key | \
      gpg --dearmor -o /etc/apt/keyrings/rocm.gpg && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/rocm.gpg] https://repo.radeon.com/rocm/apt/7.2 ${CODENAME} main" \
      > /etc/apt/sources.list.d/rocm.list && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/rocm.gpg] https://repo.radeon.com/amdgpu/latest/ubuntu ${CODENAME} main" \
      > /etc/apt/sources.list.d/amdgpu.list && \
    printf 'Package: *\nPin: release o=repo.radeon.com\nPin-Priority: 600\n' \
      > /etc/apt/preferences.d/rocm-pin-600 && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
      rocm-hip-runtime \
      rocm-opencl-runtime \
      migraphx \
      rocminfo \
      half && \
    apt-get clean

ENV PATH="/opt/rocm/bin:${PATH}"
ENV LD_LIBRARY_PATH="/opt/rocm/lib:${LD_LIBRARY_PATH}"

# Install MIGraphX-enabled ONNX Runtime for Python and C++ GPU inference.
# Strategy: install onnxruntime-migraphx (ORT 1.24.2 with MIGraphX EP), then binary-patch
# its libonnxruntime.so version tag from VERS_1.24.2 to VERS_1.23.2 so the base image's
# C++ vision behaviors (compiled against ORT 1.23.2) can link against it.
# hadolint ignore=SC2155,DL3013
RUN pip3 uninstall -y onnxruntime onnxruntime-gpu 2>/dev/null || true; \
    pip3 install --no-cache-dir onnxruntime-migraphx && \
    export ORT_CAPI=$(python3 -c "import onnxruntime; import os; print(os.path.join(os.path.dirname(onnxruntime.__file__), 'capi'))") && \
    # Patch the version tag IN PLACE so cmake (which uses pip's cmake config) also
    # finds the patched version. Both strings are 11 bytes — safe in-place replacement.
    python3 -c "import struct; f=open('$ORT_CAPI/libonnxruntime.so.1.24.2','r+b'); d=f.read(); d=d.replace(b'VERS_1.24.2',b'VERS_1.23.2'); d=d.replace(struct.pack('<I',0x027bf282),struct.pack('<I',0x027bf382),1); f.seek(0); f.write(d); f.truncate(); f.close()" && \
    # Also install patched copies to /usr/lib for system-wide discovery
    cp "$ORT_CAPI/libonnxruntime.so.1.24.2" /usr/lib/libonnxruntime.so.1 && \
    cp "$ORT_CAPI/libonnxruntime_providers_migraphx.so" /usr/lib/ && \
    cp "$ORT_CAPI/libonnxruntime_providers_shared.so" /usr/lib/ && \
    ln -sf /usr/lib/libonnxruntime.so.1 /usr/lib/libonnxruntime.so && \
    ldconfig

# Ensure the patched ORT is always loaded first, even when colcon's setup.bash
# prepends user workspace lib dirs to LD_LIBRARY_PATH with unpatched copies.
ENV LD_PRELOAD=/usr/lib/libonnxruntime.so.1

# Fix ORT library symlinks in the overlay install so the patched (MIGraphX-enabled)
# ORT is found at runtime. Also ensure the MIGraphX provider .so is discoverable
# via the overlay lib path, since ORT dlopen()s it relative to libonnxruntime.so.
RUN for pkg_lib in $(find /opt/overlay_ws/install -type d -name lib); do \
      ln -sf /usr/lib/libonnxruntime.so.1 "$pkg_lib/libonnxruntime.so.1" 2>/dev/null || true; \
      ln -sf /usr/lib/libonnxruntime_providers_migraphx.so "$pkg_lib/libonnxruntime_providers_migraphx.so" 2>/dev/null || true; \
      ln -sf /usr/lib/libonnxruntime_providers_shared.so "$pkg_lib/libonnxruntime_providers_shared.so" 2>/dev/null || true; \
    done && \
    ldconfig

# Install additional dependencies
# NOTE: The /opt/overlay_ws folder contains MoveIt Pro binary packages and the source file.
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,target=${USER_WS}/src,source=./src \
    . /opt/overlay_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src \
      --skip-keys="phoebe_sim lab_sim_behaviors clearpath_controller_interface"

# Set up colcon defaults for the new user
USER ${USERNAME}
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
COPY colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml

# hadolint ignore=DL3002
USER root

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
FROM base AS user-overlay-dev

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        gdb \
        nano \
        tmux \
        mesa-utils

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]

#########################################
# Target for compiled, deployable image #
#########################################
FROM base AS user-overlay

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Compile the workspace
WORKDIR $USER_WS

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]
