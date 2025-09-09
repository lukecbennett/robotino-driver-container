# No --platform here; we’ll tell buildx to build amd64 in the workflow
FROM osrf/ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash","-lc"]

# Basics
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget git locales tzdata \
    build-essential python3-pip python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Locale (optional)
RUN sed -i 's/^# *en_US.UTF-8/en_US.UTF-8/' /etc/locale.gen && locale-gen
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Qt/Boost runtime for API2 (Jammy)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5a libqt5network5 libqt5xml5 libqt5dbus5 \
    libqt5gui5 libqt5widgets5 libqt5concurrent5 \
    libqt5x11extras5 libqt5serialport5 \
    libboost-system1.74.0 libboost-thread1.74.0 \
 && rm -rf /var/lib/apt/lists/*

# Robotino API2 (amd64) client libs
WORKDIR /tmp/robotino_debs
ADD https://packages2.openrobotino.org/pool/buster/main/r/rec-rpc/rec-rpc_1.6.1_amd64.deb ./rec-rpc_1.6.1_amd64.deb
ADD https://packages2.openrobotino.org/pool/buster/main/r/robotino-api2/robotino-api2_1.1.14_amd64.deb ./robotino-api2_1.1.14_amd64.deb
ADD https://packages2.openrobotino.org/pool/buster/main/r/robotino-dev/robotino-dev_1.0.3_amd64.deb ./robotino-dev_1.0.3_amd64.deb
RUN dpkg -i *.deb || apt-get -f install -y && rm -rf /var/lib/apt/lists/*

# ros2-robotino driver
RUN mkdir -p /opt/robotino_ws/src
WORKDIR /opt/robotino_ws/src
RUN git clone https://github.com/robocup-logistics/ros2-robotino.git
WORKDIR /opt/robotino_ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Entrypoint
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /opt/robotino_ws/install/setup.bash\nexec "$@"\n' > /entrypoint.sh && \
    chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
