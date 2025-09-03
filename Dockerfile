# Set desired ROS distribution, this image currently only supports humble.
ARG ROS_DISTRO=humble

# This layer grabs package manifests from the src directory for preserving rosdep installs.
# This can significantly speed up rebuilds for the base package when src contents have changed.
FROM alpine:latest AS package-manifests

# Copy in the src directory, then remove everything that isn't a manifest or an ignore file.
COPY src/ /src/
RUN find /src -type f ! -name "package.xml" ! -name "COLCON_IGNORE" -delete && \
    find /src -type d -empty -delete

# Throw away for an empty source directory
RUN mkdir -p /src

# Using the pre-compiled ROS images as the base.
FROM ros:${ROS_DISTRO} AS er4-dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Overridable non root user information, this can be annoying for non humble ROS base images, which
# may already have a non-root user created.
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=er4-user

# Define the install location for the developing application
ENV ER4_WS="/home/er4-user/ws"

# DEBIAN_FRONTEND is set as an ARG instead of ENV variable so it doesn't persist in the image after build
ARG DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -q -y \
    bash-completion \
    ccache \
    gdb \
    gdbserver \
    git \
    less \
    python3-colcon-clean \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    software-properties-common \
    terminator \
    tmux \
    vim \
    xterm \
    wget

# Add a non-root user with provided user details
RUN groupadd -g ${USER_GID} ${USERNAME} \
    && useradd -l -u ${USER_UID} -g ${USER_GID} --create-home -m -s /bin/bash -G sudo,adm,dialout,dip,plugdev,video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p \
        /home/${USERNAME}/.ccache \
        /home/${USERNAME}/.colcon \
        /home/${USERNAME}/.ros \
        /home/${USERNAME}/.bash \
        ${ER4_WS}

# Setup the install directory and copy the workspace to it.
# We could alternatively copy package manifests to preserve the layer cache if the build duration becomes too onerous.
WORKDIR  ${ER4_WS}
RUN mkdir src build install log

# Copy package manifests for installing rosdeps
COPY --chown=${USERNAME}:${USERNAME} --from=package-manifests /src/ ./src

# Install rosdeps
# Init is unnecessary if using the ROS base image
# RUN sudo rosdep init && rosdep update --rosdistro ${ROS_DISTRO}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install -iy --from-paths src

# Install extra ROS deps
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -q -y \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp

# Configure and install MuJoCo using the defaults for the MuJoCo drivers.
# We use MuJoCo in many systems so we just install the drivers in the base workspace.
# The install is CPU dependent, this works with `x86_64` and `arm64` chips, TBD on others.
ARG MUJOCO_VERSION=3.3.4
ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
RUN mkdir -p ${MUJOCO_DIR} && sudo chown -R ${USERNAME}:${USERNAME} ${MUJOCO_DIR}
RUN CPU_ARCH=$(uname -m); \
    wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz && \
    tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}") && \
    rm "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz"

# Install MuJoCo specific pip dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pip install mujoco obj2mjcf

# There's no build for arm64 on linux, so just ignore failures here if that's the case
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pip install bpy==4.0.0 --extra-index-url https://download.blender.org/pypi/ || true

# Copy in the remainder of the src directory
COPY src/ src/
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}

# Setup colcon default mixins and add default settings
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update || true
RUN colcon metadata add default  \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update || true

# Configure pyassimp, which has some unique problems on aarch machines.
# To address this, we have adjusted the $LD_LIBRARY_PATH in the entrypoint to ensure
# the required path is available to the python module to load the library.
RUN pip3 install pyassimp==4.1.3

# Install useful tooling
RUN pip3 install ipython

# Fix rosdep permissions and ensure sudo while we're at it
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update --rosdistro ${ROS_DISTRO}

# copy in configs for different features
COPY --chown=${USERNAME}:${USERNAME} config/colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml
COPY --chown=${USERNAME}:${USERNAME} config/terminator_config /home/${USERNAME}/.config/terminator/config

# Setup entrypoint and ensure it's added to ~/.bashrc
COPY scripts/entrypoint.sh /entrypoint.sh
RUN echo "source /entrypoint.sh" >> ~/.bashrc

# Make it obvious when operating in a container
RUN echo "PS1=\"${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\](docker):\[\033[01;34m\]\w\[\033[00m\]\$ \"" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

# Source built dev image for automated testing.
FROM er4-dev AS er4-dev-source

RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build
