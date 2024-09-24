FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-humble-turtlesim\
    && rm -rf /var/lib/apt/lists/*

# Workspace for the container
WORKDIR /turtle_chaser_ws

COPY src ./src

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source and run the turtle_runner node
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /turtle_chaser_ws/install/setup.bash && ros2 launch turtle_command turtle_command_launch.py"]
