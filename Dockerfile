# amd64 image; we build the driver inside
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Basics
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release locales tzdata git build-essential \
    && rm -rf /var/lib/apt/lists/* \
 && locale-gen en_US.UTF-8 || true

# ROS 2 Jazzy (noble)
RUN mkdir -p /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros-infrastructure/ros-apt-source/refs/heads/master/ros.key \
      | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list' && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Qt/Boost needed by API2
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5t64 libqt5network5t64 libqt5xml5t64 libqt5dbus5t64 \
    libqt5gui5t64  libqt5widgets5t64 libqt5concurrent5t64 \
    libqt5x11extras5 libqt5serialport5 \
    libboost-system1.83.0 libboost-thread1.83.0 \
  && rm -rf /var/lib/apt/lists/*

# API2/rec-rpc (amd64)
WORKDIR /tmp/robotino_debs
ADD https://packages2.openrobotino.org/pool/buster/main/r/rec-rpc/rec-rpc_1.6.1_amd64.deb ./rec-rpc_1.6.1_amd64.deb
ADD https://packages2.openrobotino.org/pool/buster/main/r/robotino-api2/robotino-api2_1.1.14_amd64.deb ./robotino-api2_1.1.14_amd64.deb
ADD https://packages2.openrobotino.org/pool/buster/main/r/robotino-dev/robotino-dev_1.0.3_amd64.deb ./robotino-dev_1.0.3_amd64.deb
RUN dpkg -i *.deb || apt-get -f install -y && rm -rf /var/lib/apt/lists/*

# Driver sources
WORKDIR /opt/ros2_robotino_ws/src
RUN git clone https://github.com/robocup-logistics/ros2-robotino.git

# Build
SHELL ["/bin/bash", "-lc"]
WORKDIR /opt/ros2_robotino_ws
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Entrypoint that sources overlays
RUN bash -lc 'printf "%s\n%s\n%s\n" \
  "source /opt/ros/jazzy/setup.bash" \
  "source /opt/ros2_robotino_ws/install/setup.bash" \
  ' > /entrypoint_ros.sh && chmod +x /entrypoint_ros.sh

ENTRYPOINT ["/bin/bash","-lc","source /entrypoint_ros.sh && exec \"$@\""]
CMD ["bash"]
