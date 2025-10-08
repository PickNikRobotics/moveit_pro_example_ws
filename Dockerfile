# Docker image for extending MoveIt Pro with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Pro release to build on top of.
ARG MOVEIT_STUDIO_BASE_IMAGE=picknikciuser/moveit-studio:${STUDIO_DOCKER_TAG:-main}
ARG USERNAME=studio-user
ARG USER_UID=1000
ARG USER_GID=1000

##################################################
# Starting from the specified MoveIt Pro release #
##################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_STUDIO_BASE_IMAGE} AS base

# Create a non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Set real time limits
# Ensure the directory exists
RUN mkdir -p /etc/security
# Copy the custom limits configuration into the container
COPY src/moveit_pro_franka_configs/franka_arm_hw/config/rt_limits.conf /etc/security/limits.conf

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME} && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends sudo wget && \
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

# Install onnx dependencies
RUN wget -O onnxruntime_gpu-1.19.0-cp310-cp310-linux_aarch64.whl https://nvidia.box.com/shared/static/6l0u97rj80ifwkk8rqbzj1try89fk26z.whl -q --show-progress --progress=dot:giga && \
    pip install onnxruntime_gpu-1.19.0-cp310-cp310-linux_aarch64.whl && \
    rm onnxruntime_gpu-1.19.0-cp310-cp310-linux_aarch64.whl

# Allow for ld to find the onnx runtime libraries.
ENV LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/onnxruntime/capi:$LD_LIBRARY_PATH

# jetson repo source
COPY nvidia-l4t-apt-source.list /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
# apt update to add jetson repos, then install NVIDIA dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
      apt-get update && apt-get install -q -y --no-install-recommends \
      libcudnn9-cuda-12 \
      libcudnn9-dev-cuda-12 \
      libcublas-12-6 \
      cuda-cudart-12-6 \
      libcurand-12-6 \
      libcufft-12-6 \
      libnvinfer10 \
      libnvinfer-plugin10 \
      libnvonnxparsers10 \
      libtree

# Add user to dialout group to enable communication with serial USB devices (gripper, FTS, ...)
# Add user to video group to enable communication with cameras
RUN usermod -aG dialout,video ${USERNAME}

# Add user to the realtime group to enable RT limits
RUN groupadd realtime && \
    usermod -a -G realtime ${USERNAME}

# Install additional dependencies
# You can also add any necessary apt-get install, pip install, etc. commands at this point.
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
      --skip-keys 'ign_ros2_control ros_gz_sim ros_gz ignition-gazebo6'

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
        tmux

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
