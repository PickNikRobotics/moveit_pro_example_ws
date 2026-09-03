# Docker image for extending MoveIt Pro with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Pro release to build on top of.
ARG MOVEIT_PRO_BASE_IMAGE=picknikciuser/moveit-pro:${MOVEIT_DOCKER_TAG:-main}-${MOVEIT_ROS_DISTRO:-jazzy}
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

# Ubuntu 24.04 images include user `ubuntu` with UID/GID 1000.
# Remove it before creating the workspace user with the host UID/GID.
RUN if id -u ubuntu > /dev/null 2>&1; then userdel -r ubuntu; fi

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME} && \
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

# Add user to the realtime group to enable RT limits
RUN groupadd realtime && \
    usermod -a -G realtime ${USERNAME}

# Install nav2_mppi_controller 1.3.13 from the ros2-testing apt repository.
#
# The base image resolves every ROS package from a dated snapshot of the Jazzy
# archive (snapshots.ros.org, see moveit_pro's Dockerfile), which still carries
# nav2 1.3.12. hangar_sim needs the MPPI `open_loop` parameter (moveit_pro issue
# #21202), which upstream added in nav2 1.3.13 (ros-navigation/navigation2#6336),
# and 1.3.13 has so far only reached ros2-testing. This replaces the PickNik-built
# package pin that moveit_pro#21347 tried.
#
# The repository is configured only for the duration of this RUN, following the
# base image's own pattern for one-off sources (add, install, hold, remove):
#
# - `Package: *` from packages.ros.org is pinned to -1, which apt treats as
#   "never install", so every dependency, nav2 or otherwise, resolves from the
#   snapshot. The one build named below is pinned to 1001 so apt accepts it.
# - `apt-mark hold` is what keeps the package at 1.3.13 afterwards; the rosdep
#   pass later in this stage and any apt run in a derived image leave it alone.
# - The source, the preferences file, and the key are removed at the end, so no
#   derived image inherits a pre-release repository or a pin on packages.ros.org.
#
# Only nav2_mppi_controller needs to move. Between 1.3.12 and 1.3.13 the only
# header change in a package MPPI links against is nav2_costmap_2d's
# costmap_2d_publisher.hpp, which gained a member; MPPI reaches that class only
# through Costmap2DROS's unique_ptr, and costmap_2d_ros.hpp is unchanged.
# nav2_core changed only its package.xml, so the pluginlib vtable matches, and
# the deb's ROS dependencies are unversioned. Re-check on any version bump with
#   nm -DC --undefined-only /opt/ros/jazzy/lib/libmppi_controller.so | grep nav2
# which today lists only Costmap2D accessors and FootprintCollisionChecker.
#
# The build IDs below are exact, per arch, and hand-copied from the index.
# ros2-testing keeps only its newest build of a version, so an upstream rebuild
# of nav2 makes this RUN fail at `apt-get install` until the IDs are refreshed.
# That is deliberate: it is the only signal that the pinned build changed.
# Remove this block and docker/keys/ once the base image's snapshot carries
# nav2 >= 1.3.13. The image does not export ROS_DISTRO, so the block sources the
# overlay to read it and refuses to build on anything but jazzy.
#
# Key: Open Robotics apt signing key, fingerprint
# C1CF 6E31 E6BA DE88 68B1 72B4 F42E D6FB AB17 C654, exported ASCII-armored from
# https://raw.githubusercontent.com/ros/rosdistro/master/ros.key.
COPY docker/keys/ros-archive-keyring.asc /tmp/ros2-testing-archive-keyring.asc
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/overlay_ws/install/setup.sh && \
    if [ "${ROS_DISTRO}" != "jazzy" ]; then \
        echo "ERROR: the nav2_mppi_controller ros2-testing install is written for jazzy/noble," \
             "got ROS_DISTRO='${ROS_DISTRO}'. Drop or retarget this block for other distros." >&2; \
        exit 1; \
    fi && \
    case "$(dpkg --print-architecture)" in \
        amd64) NAV2_MPPI_VERSION="1.3.13-1noble.20260831.141656" ;; \
        arm64) NAV2_MPPI_VERSION="1.3.13-1noble.20260831.142352" ;; \
        *) echo "ERROR: no pinned nav2_mppi_controller build for $(dpkg --print-architecture)" >&2; exit 1 ;; \
    esac && \
    gpg --dearmor --yes -o /usr/share/keyrings/ros2-testing-archive-keyring.gpg /tmp/ros2-testing-archive-keyring.asc && \
    printf '%s\n' \
        'Types: deb' \
        'URIs: http://packages.ros.org/ros2-testing/ubuntu' \
        'Suites: noble' \
        'Components: main' \
        'Signed-By: /usr/share/keyrings/ros2-testing-archive-keyring.gpg' \
        > /etc/apt/sources.list.d/ros2-testing.sources && \
    printf '%s\n' \
        'Package: *' \
        'Pin: origin "packages.ros.org"' \
        'Pin-Priority: -1' \
        '' \
        'Package: ros-jazzy-nav2-mppi-controller' \
        "Pin: version ${NAV2_MPPI_VERSION}" \
        'Pin-Priority: 1001' \
        > /etc/apt/preferences.d/nav2-mppi-ros2-testing.pref && \
    apt-get update && \
    apt-get install -y --no-install-recommends "ros-jazzy-nav2-mppi-controller=${NAV2_MPPI_VERSION}" && \
    installed="$(dpkg-query -W -f='${Version}' ros-jazzy-nav2-mppi-controller)" && \
    if [ "${installed}" != "${NAV2_MPPI_VERSION}" ]; then \
        echo "ERROR: ros-jazzy-nav2-mppi-controller is ${installed}, expected ${NAV2_MPPI_VERSION}" >&2; \
        exit 1; \
    fi && \
    apt-mark hold ros-jazzy-nav2-mppi-controller && \
    rm /etc/apt/sources.list.d/ros2-testing.sources \
       /etc/apt/preferences.d/nav2-mppi-ros2-testing.pref \
       /usr/share/keyrings/ros2-testing-archive-keyring.gpg \
       /tmp/ros2-testing-archive-keyring.asc

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
      --ignore-src

# Python dependencies for the C0 Kinova rollout client (scripts/kinova/rollout.py).
# The torch-free real-Kinova rollout client runs in the agent_bridge container and uses
# `websockets` (sync) + `msgpack` to talk to the policy server. These are installed via pip
# (rather than a rosdep key) because the Ubuntu Jammy apt `python3-websockets` is 9.1, which
# does not satisfy the required `websockets>=13`. Baking them in here replaces the previous
# ephemeral `pip install` workaround inside the running container.
# hadolint ignore=DL3013
RUN python3 -m pip install --no-cache-dir \
      "websockets>=13" \
      "msgpack>=1.0"

# Set up colcon defaults for the new user
USER ${USERNAME}
# Vendor the colcon mixin/metadata repositories into the image over git rather than
# letting colcon fetch each file from raw.githubusercontent.com. The CDN rate-limits
# (HTTP 429), colcon's load_url() only retries 503 and socket timeouts, and
# `colcon mixin update` fails if any single file in the index fails -- so one throttled
# request out of 19 breaks the whole image build.
#
# The clones are kept rather than deleted: `colcon mixin add` persists the index URL, and
# a later unqualified `colcon mixin update` re-reads every registered source. Keeping them
# also takes the CDN off the build path entirely. Combined size is ~164 KB without .git.
RUN mkdir -p /home/${USERNAME}/.colcon/repositories && \
    git clone --depth 1 \
      https://github.com/colcon/colcon-mixin-repository /home/${USERNAME}/.colcon/repositories/mixin && \
    git clone --depth 1 \
      https://github.com/colcon/colcon-metadata-repository /home/${USERNAME}/.colcon/repositories/metadata && \
    rm -rf /home/${USERNAME}/.colcon/repositories/mixin/.git /home/${USERNAME}/.colcon/repositories/metadata/.git && \
    colcon mixin add default file:///home/${USERNAME}/.colcon/repositories/mixin/index.yaml && \
    colcon mixin update && \
    colcon metadata add default file:///home/${USERNAME}/.colcon/repositories/metadata/index.yaml && \
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
