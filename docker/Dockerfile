FROM murilomarinho/sas:jazzy
LABEL authors="rawanelsayed,murilomarinho"
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros-msgs \
    ros-jazzy-realtime-tools \
    ros-jazzy-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && sudo apt-get upgrade -y
RUN python3 -m pip install --upgrade dqrobotics quadprog --break-system-packages

# Workspace location
WORKDIR /root/ros2_ws
RUN mkdir -p /root/ros2_ws/src

# Copy sources into image
COPY . /root/ros2_ws/src/sas_robot_driver_brov2

# Copy build script into the SAME workspace
COPY docker/setup_environment.sh /root/ros2_ws/setup_environment.sh
RUN chmod +x /root/ros2_ws/setup_environment.sh

# Build at image build time
RUN /root/ros2_ws/setup_environment.sh
