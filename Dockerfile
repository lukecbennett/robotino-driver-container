## Build for amd64 via the workflow's platforms: linux/amd64
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget gnupg lsb-release locales tzdata git build-essential \
 && rm -rf /var/lib/apt/lists/* \
 && locale-gen en_US.UTF-8 || true

# ROS 2 Jazzy on Noble (amd64)
RUN mkdir -p /usr/share/keyrings && \
    set -euo pipefail; \
    curl -fsSL https://raw.githubusercontent.com/ros-infrastructure/ros-apt-source/refs/heads/master/ros.key \
      | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# Init rosdep (inside the image)
RUN rosdep init && rosdep update

# Robotino API2 runtime deps (Qt/Boost)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5t64 libqt5network5t64 libqt5xml5t64 libqt5dbus5t64 \
    libqt5gui5t64  libqt5widgets5t64 libqt5concurrent5t64 \
    libqt5x11extras5 libqt5serialport5 \
    libboost-system1.83.0 libboost-thread1.83.0 \
 && rm -rf /var/lib/apt/lists/*

# Download & install API2/rec-rpc debs (no ADD from remote!)
WORKDIR /tmp/robotino_debs
RUN set -euo pipefail; \
    curl -fL --retry 5 -o rec-rpc_1.6.1_amd64.deb \
      https://packages2.openrobotino.org/pool/buster/main/r/rec-rpc/rec-rpc_1.6.1_amd64.deb && \
    curl -fL --retry 5 -o robotino-api2_1.1.14_amd64.deb \
      https://packages2.openrobotino.org/pool/buster/main/r/robotino-api2/robotino-api2_1.1.14_amd64.deb && \
    curl -fL --retry 5 -o robotino-dev_1.0.3_amd64.deb \
      https://packages2.openrobotino.org/pool/buster/main/r/robotino-dev/robotino-dev_1.0.3_amd64.deb && \
    dpkg -i ./*.deb || apt-get -f install -y && rm -rf /var/lib/apt/lists/*

# Driver sources
WORKDIR /opt/ros2_robotino_ws/src
RUN git clone https://github.com/robocup-logistics/ros2-robotino.git

# Build the driver
SHELL ["/bin/bash","-lc"]
WORKDIR /opt/ros2_robotino_ws
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Entrypoint that sources overlays
RUN bash -lc 'printf "%s\n%s\n" \
  "source /opt/ros/jazzy/setup.bash" \
  "source /opt/ros2_robotino_ws/install/setup.bash" > /entrypoint_ros.sh' \
  && chmod +x /entrypoint_ros.sh

ENTRYPOINT ["/bin/bash","-lc","source /entrypoint_ros.sh && exec \"$@\""]
CMD ["bash"]
