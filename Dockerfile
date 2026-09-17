FROM ros:lyrical-ros-base

# Install Zenoh RMW and build tools
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-lyrical-rmw-zenoh-cpp \
    iproute2 \
    iputils-ping \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


RUN pip3 install --break-system-packages \
    flask \
    flask-socketio \
    flask-cors \
    simple-websocket \
    eclipse-zenoh

# Set default middleware to Zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

COPY ./ros2_ws /app/ros2_ws
WORKDIR /app/ros2_ws

RUN /bin/bash -c "source /opt/ros/lyrical/setup.bash && colcon build"