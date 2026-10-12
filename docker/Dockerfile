# syntax=docker/dockerfile:1
# Apt lock: docker/apt-lock.txt is generated from the sim image (a superset of ci). After building sim, from the repo root:
#   docker build -f docker/Dockerfile --target sim -t planner-sim . && \
#   { echo "# apt lock generated from planner-sim on $(date -u +%F); regenerate: see docker/Dockerfile header"; \
#     docker run --rm --entrypoint dpkg-query planner-sim -W -f='${Package}=${Version}\n' | LC_ALL=C sort; } > docker/apt-lock.txt
# docker/check-lock.sh <image> fails when an image holds a package/version absent from the lock.
# Base: ros:humble-ros-base-jammy, resolved 2026-10-05.
FROM ros@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19 AS base

# Stage ci: build + test environment (dependencies only; sources are mounted at /ws/src).
FROM base AS ci
# Ubuntu and ROS packages come from dated snapshots so rebuilds resolve identical versions.
RUN sed -i 's,http://\(archive\|security\)\.ubuntu\.com/ubuntu/,https://snapshot.ubuntu.com/ubuntu/20261012T000000Z/,' /etc/apt/sources.list \
    && ! grep -v '^#' /etc/apt/sources.list | grep -q '//\(archive\|security\)\.ubuntu'
RUN export GNUPGHOME="$(mktemp -d)" K=4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA \
    && gpg --batch --keyserver hkps://keyserver.ubuntu.com --recv-keys "$K" \
    && gpg --batch --export "$K" > /usr/share/keyrings/ros-snapshot.gpg \
    && rm -rf "$GNUPGHOME" \
    && rm -f /etc/apt/sources.list.d/ros2.sources \
    && echo "deb [signed-by=/usr/share/keyrings/ros-snapshot.gpg] http://snapshots.ros.org/humble/2026-09-14/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2-snapshot.list
# Layers run from least to most likely to change.
# 1. Toolchain and build tools (git: FetchContent clones RapidCheck).
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        g++-12 \
        gcc-12 \
        git \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
# 2. System libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libabsl-dev \
        libcurl4-openssl-dev \
        libeigen3-dev \
        libgtest-dev \
        libopencv-dev \
        libyaml-cpp-dev \
        nlohmann-json3-dev \
        python3-matplotlib \
        python3-numpy \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*
# 3. ROS packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        ros-humble-bondcpp \
        ros-humble-controller-manager \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-launch-testing-ros \
        ros-humble-lifecycle-msgs \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-slam-toolbox \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-ros \
        ros-humble-topic-tools \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*
ENV CC=gcc-12 CXX=g++-12 MAKEFLAGS=-j2
RUN useradd -m -u 1000 dev && mkdir -p /ws/src /ws/build /ws/install /ws/log && chown -R dev:dev /ws
WORKDIR /ws
USER dev
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]

# Stage sim: adds Gazebo Fortress + visualization; software rendering only.
FROM ci AS sim
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1-mesa-dri \
        mesa-utils \
        ros-humble-ign-ros2-control \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-interfaces \
        ros-humble-ros-gz-sim \
        ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*
ENV LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
USER dev
