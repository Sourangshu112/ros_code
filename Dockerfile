FROM ros:jazzy-ros-base

# Install Zenoh RMW and build tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-zenoh-cpp \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-pip && pip3 install flask flask-socketio simple-websocket --break-system-packages

RUN apt-get install -y python3-colcon-common-extensions

# Set default middleware to Zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

COPY ./ros2_ws /app/ros2_ws
WORKDIR /app/ros2_ws

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"