# syntax=docker/dockerfile:1
# The syntax line matches moveit_pro's Dockerfile. It keeps `ADD --checksum`
# (below) working on engines whose built-in frontend predates it (Docker < 25).
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

######################################################
# Pinned nav2_mppi_controller 1.3.13 debs (see base) #
######################################################
# Verified downloads only; the stage is bind-mounted into the install step below
# and never becomes part of the image. Both arches are fetched (under 1 MB) so
# the stage does not depend on TARGETARCH; the cost is that a build for either
# arch also needs the other arch's object to verify.
FROM scratch AS nav2-mppi-debs
ADD --chmod=644 --checksum=sha256:1d0e6b4dd0cb1e132a63a7084bfe53e1a07d595c25e78d44b8e078f1403a9737 \
    https://download.picknik.ai/third-party/ros-jazzy-nav2-mppi-controller/ros-jazzy-nav2-mppi-controller_1.3.13-1noble.20260903.032340_amd64.deb \
    /amd64.deb
ADD --chmod=644 --checksum=sha256:b671c1385a1d8ebcd0fdc76b1b601dfb9b46d4e340de6bdb67b8f279a31b123a \
    https://download.picknik.ai/third-party/ros-jazzy-nav2-mppi-controller/ros-jazzy-nav2-mppi-controller_1.3.13-1noble.20260903.005819_arm64.deb \
    /arm64.deb

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

<<<<<<< HEAD
=======
# Install nav2_mppi_controller 1.3.13 from PickNik's release bucket.
#
# The base image resolves every ROS package from a dated snapshot of the Jazzy
# archive (snapshots.ros.org, see moveit_pro's Dockerfile), which still carries
# nav2 1.3.12. hangar_sim needs the MPPI `open_loop` parameter (moveit_pro issue
# #21202), which upstream added in nav2 1.3.13 (ros-navigation/navigation2#6336).
# 1.3.13 has so far only reached the ros2-testing apt repository, which keeps
# only its newest build of each version and rebuilds without notice, so nothing
# there can be pinned durably. The exact debs in the nav2-mppi-debs stage above
# were copied from ros2-testing (2026-09-03 builds) to
# s3://moveit-pro-releases/third-party/ros-jazzy-nav2-mppi-controller/, public
# at https://download.picknik.ai/<key>. Objects there are never overwritten or
# expired (the bucket has no lifecycle rule; PickNik owns it), and ADD verifies
# each download against its pinned sha256, so the build fails on any change to
# the hosted file. The same prefix holds
# ros-jazzy-nav2-mppi-controller_1.3.13-SHA256SUMS with the two digests.
#
# How the debs were produced, and how to replace them:
#   1. Read Version, Filename and SHA256 for ros-jazzy-nav2-mppi-controller from
#      http://packages.ros.org/ros2-testing/ubuntu/dists/noble/main/binary-<arch>/Packages.gz
#      (the index is covered by the ros2-testing Release signature).
#   2. curl -fO http://packages.ros.org/ros2-testing/ubuntu/<Filename>; sha256sum
#      the file and compare with the index's SHA256.
#   3. aws s3 cp <file> s3://moveit-pro-releases/third-party/ros-jazzy-nav2-mppi-controller/
#      (never overwrite an existing key), refresh the SHA256SUMS object, then
#      update the two ADD lines above (URL and --checksum).
#   4. Redo the ABI checks below against the snapshot's nav2.
#
# `apt-get install <path>.deb` resolves the deb's dependencies from the
# configured snapshot like any other install, and `apt-mark hold` keeps the
# package at 1.3.13 through the rosdep pass later in this stage and any apt run
# in a derived image. No extra apt source, preference, or key is involved. The
# base image tag floats, so the RUN first asks apt whether the snapshot already
# offers >= 1.3.13; if it does, the block is obsolete and skips itself with a
# notice instead of holding the package below the snapshot.
#
# Only nav2_mppi_controller needs to move. Between 1.3.12 and 1.3.13 the only
# header change in a package MPPI links against is nav2_costmap_2d's
# costmap_2d_publisher.hpp, which gained a member; MPPI reaches that class only
# through Costmap2DROS's unique_ptr, and costmap_2d_ros.hpp is unchanged.
# nav2_core changed only its package.xml, so the pluginlib vtable matches, and
# the deb's ROS dependencies are unversioned. The symbols MPPI imports from nav2
# (`nm -DC --undefined-only /opt/ros/jazzy/lib/libmppi_*.so | grep nav2`) are
# Costmap2D accessors, FootprintCollisionChecker, and InflationLayer typeinfo,
# none of which changed. Because the snapshot underneath can still move, the RUN
# also runs `ldd -r` on both shipped libraries (libmppi_controller.so and
# libmppi_critics.so; the latter is not a DT_NEEDED of the former) and fails on
# any undefined symbol or missing library, using the LD_LIBRARY_PATH that
# sourcing the overlay sets up.
#
# Remove this block and the nav2-mppi-debs stage once the base image's snapshot
# carries nav2 >= 1.3.13 (the notice below says when). The image does not export
# ROS_DISTRO, so the block sources the overlay to read it and refuses to build
# on anything but jazzy.
# DL3008: the install is of a local deb whose exact build is pinned by checksum.
# hadolint ignore=SC1091,DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,from=nav2-mppi-debs,target=/tmp/nav2-mppi-debs \
    . /opt/overlay_ws/install/setup.sh && \
    if [ "${ROS_DISTRO}" != "jazzy" ]; then \
        echo "ERROR: the nav2_mppi_controller install is written for jazzy/noble," \
             "got ROS_DISTRO='${ROS_DISTRO}'. Drop or retarget this block for other distros." >&2; \
        exit 1; \
    fi && \
    apt-get update && \
    if apt-get satisfy -s -q "ros-jazzy-nav2-mppi-controller (>= 1.3.13)" > /dev/null 2>&1; then \
        echo "NOTICE: apt already offers ros-jazzy-nav2-mppi-controller >= 1.3.13;" \
             "the pinned-deb block in this Dockerfile is obsolete and was skipped. Remove it." >&2; \
        exit 0; \
    fi && \
    deb="/tmp/nav2-mppi-debs/$(dpkg --print-architecture).deb" && \
    if [ ! -f "${deb}" ]; then \
        echo "ERROR: no nav2_mppi_controller deb for $(dpkg --print-architecture) in the nav2-mppi-debs stage;" \
             "add one to s3://moveit-pro-releases/third-party/ros-jazzy-nav2-mppi-controller/ (see the comment above)." >&2; \
        exit 1; \
    fi && \
    apt-get install -y --no-install-recommends "${deb}" && \
    apt-mark hold ros-jazzy-nav2-mppi-controller && \
    ldd_out="$(mktemp)" && \
    dpkg -L ros-jazzy-nav2-mppi-controller > "${ldd_out}" && \
    if ! shipped_libs="$(grep -E '\.so(\.[0-9]+)*$' "${ldd_out}")"; then \
        echo "ERROR: ros-jazzy-nav2-mppi-controller ships no shared library to check" >&2; \
        exit 1; \
    fi && \
    for so in ${shipped_libs}; do \
        if ! ldd -r "${so}" > "${ldd_out}" 2>&1 || \
           grep -qE 'undefined symbol|not found' "${ldd_out}"; then \
            cat "${ldd_out}" >&2; \
            echo "ERROR: ${so} does not resolve against the image's ROS libraries (see above)" >&2; \
            exit 1; \
        fi; \
    done && \
    rm -f "${ldd_out}"

>>>>>>> 0e45115 (fix(docker): install nav2_mppi_controller 1.3.13 from PickNik's release bucket)
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
