# Steps to build:
# 1. "sudo docker build -f docker/Dockerfile -t ros2_workspace ."
# 2. "sudo docker run -it --rm -v $(pwd):/workspace ros2_workspace"
# MAYBE THIS INSTEAD: "sudo docker run -it --rm   -v $(pwd)/src:/workspace/src   --device=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_03536373332351C06150-if00   ros2_workspace"
# 3. "source install/setup.bash"

# Steps to open image in adjacent terminal:
# 1. "sudo docker ps" and copy name under NAMES
# 2. sudo docker exec -it <name> bash

# Steps to drive robot
# 1. "ros2 launch my_robot_description diffdrive_physical.launch.py"
# 2. !!!!!!!!!!! READ THIS !!!!!!!!!!! REDUCE SPEED TO ZERO BEFORE DRIVING !!!!!!!!!!! READ THIS !!!!!!!!!!!
# 3. In a seperate terminal running the docker image: "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
# 4. drive (:


# Use the official ROS 2 Humble base image
FROM ros:humble

# Set environment variables
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (skip init if already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

# Create workspace directory
WORKDIR /workspace

# Ensure the src directory exists
RUN mkdir -p /workspace/src

# Copy everything into the container
COPY . /workspace/

# Install dependencies
#RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y && rm -rf /var/lib/apt/lists/*

# Build the workspace
# RUN colcon build --symlink-install
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source the workspace setup
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Default command to run a shell
CMD ["bash"]
