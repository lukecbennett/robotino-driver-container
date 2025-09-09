# Don’t hardcode --platform here; pass it at build time instead
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash","-lc"]

# Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 ca-certificates lsb-release locales tzdata \
    build-essential git python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Locale (quiet if already present)
RUN locale-gen en_US.UTF-8 || true
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ROS 2 repo key + list (properly dearmored key)
RUN mkdir -p /usr/share/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros-infrastructure/ros-apt-source/refs/heads/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS 2 + colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Runtime deps for API2 (Qt5/Boost) on Ubuntu 24.04 (t64 variants)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5t64 libqt5network5t64 libqt5xml5t64 libqt5dbus5t64 \
    libqt5gui5t64  libqt5widgets5t64 libqt5concurrent5t64 \
    libqt5x11extras5 libqt5serialport5 \
    libboost-system1.83.0 libboost-thread1.83.0 \
 && rm -rf /var/lib/apt/lists/*

# ---- Robotino API2 debs (pass URLs as build-args; use curl -fSL) ----
ARG REC_RPC_URL
ARG API2_URL
ARG DAEMONS_URL
ARG DEV_URL

RUN set -euo pipefail \
 && mkdir -p /tmp/robotino_debs && cd /tmp/robotino_debs \
 && echo "Fetching Robotino .debs..." \
 && curl -fSL "$REC_RPC_URL" -o rec-rpc.deb \
 && curl -fSL "$API2_URL"  -o robotino-api2.deb \
 && curl -fSL "$DAEMONS_URL" -o robotino-daemons.deb \
 && curl -fSL "$DEV_URL" -o robotino-dev.deb \
 && dpkg -i *.deb || apt-get -f install -y \
 && ldconfig \
 && rm -rf /var/lib/apt/lists/* /tmp/robotino_debs

# Workspace + driver sources (robocup-logistics/ros2-robotino)
RUN mkdir -p /opt/robotino_ws/src
WORKDIR /opt/robotino_ws/src
RUN git clone https://github.com/robocup-logistics/ros2-robotino.git

# Build the driver
WORKDIR /opt/robotino_ws
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# Entrypoint: auto-source ROS + workspace
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
