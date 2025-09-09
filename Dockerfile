FROM --platform=linux/amd64 ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive

# Base tools (no colcon yet)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release locales tzdata ca-certificates \
    build-essential git python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US.UTF-8 || true
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Add ROS 2 apt source (properly dearmored key)
RUN mkdir -p /usr/share/keyrings
RUN curl -sSL https://raw.githubusercontent.com/ros-infrastructure/ros-apt-source/refs/heads/master/ros.key \
 | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg
RUN sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 Jazzy + colcon from the ROS repo
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Workspace + driver sources
RUN mkdir -p /opt/robotino_ws/src
WORKDIR /opt/robotino_ws/src
RUN git clone https://github.com/robocup-logistics/ros2-robotino.git

# Runtime deps for API2 (Qt5/Boost) on amd64
WORKDIR /opt/robotino_ws
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5t64 libqt5network5t64 libqt5xml5t64 libqt5dbus5t64 \
    libqt5gui5t64  libqt5widgets5t64 libqt5concurrent5t64 \
    libqt5x11extras5 libqt5serialport5 \
    libboost-system1.83.0 libboost-thread1.83.0 \
 && rm -rf /var/lib/apt/lists/*

# Install API2 client .debs (amd64) shipped in the repo
RUN dpkg -i /opt/robotino_ws/src/ros2-robotino/rec-rpc_*_amd64.deb \
          /opt/robotino_ws/src/ros2-robotino/robotino-api2_*_amd64.deb \
          /opt/robotino_ws/src/ros2-robotino/robotino-dev_*_amd64.deb \
 || apt-get -f install -y

# Build the driver
SHELL ["/bin/bash","-lc"]
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# Entrypoint to auto-source
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
