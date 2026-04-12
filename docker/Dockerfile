# Steps to build:
# 1. "sudo docker build -f docker/Dockerfile -t ros2_workspace ."
# 2. "sudo docker run -it --rm --runtime nvidia --device=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_03536373332351C06150-if00 --device=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_55936343833351518161-if00 ros2_workspace"
# 3. "source install/setup.bash"

# Steps to open image in adjacent terminal:
# 1. "sudo docker ps" and copy name under NAMES
# 2. sudo docker exec -it <name> bash

# Note on caching:
# Docker caches each build layer. The slow ZED SDK install is cached after the first build —
# subsequent builds skip it automatically. Just don't use --no-cache and don't run docker rmi
# (deleting the image clears the cache, forcing a full rebuild including ZED SDK).

# Steps to drive robot
# 1. "ros2 launch my_robot_description diffdrive_physical.launch.py"
# 2. !!!!!!!!!!! READ THIS !!!!!!!!!!! REDUCE SPEED TO ZERO BEFORE DRIVING !!!!!!!!!!! READ THIS !!!!!!!!!!!
# 3. In a seperate terminal running the docker image: "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
# 4. drive (:

# Why humble-ros-base (not humble-desktop)?
# Stereolabs' own Dockerfile.l4t-humble uses ros-base. Both variants ship a custom
# CUDA-compiled OpenCV (opencv-dev 4.8.x-dirty) that conflicts with Ubuntu apt opencv
# packages. ros-base is smaller and has fewer pre-installed packages to conflict with.
#
# Why r36.3.0 (not r36.4.0)?
# humble-ros-base only goes up to r36.3.0 on Docker Hub. This is fine: the host is r36.5.0
# regardless, and --runtime nvidia injects the host GPU driver at runtime. The L4T version
# in the container only determines which ZED SDK binary to download.
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# ZED SDK version — l4t36.3 matches the dustynv base above.
ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=2
ARG ZED_SDK_PATCH=3
ARG L4T_MAJOR=36
ARG L4T_MINOR=3

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive

# Refresh the ROS GPG key — the key baked into the dustynv base image expires over time.
# Stereolabs does the same in their own Dockerfile.l4t-humble (line 48).
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# TODO: Test if explicitly addiing ros-humble-ros2-control fixes most recent build error
# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    lsb-release \
    ca-certificates \
    gnupg \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    libserial-dev \
    zstd \
    libusb-1.0-0-dev \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    ros-humble-rclcpp-lifecycle \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-ros2cli-common-extensions \
    ros-humble-tf2 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (skip if already done in base image)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

# Tell the ZED SDK installer this is a Tegra system matching the dustynv base L4T version.
# Stereolabs does the same thing in their own Dockerfile.l4t-humble (line 59).
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release

# Download and install ZED SDK.
# silent mode auto-accepts any prompts (including minor L4T version warnings).
# skip_tools skips Python/OpenCV extras not needed at build time.
RUN wget -q --no-check-certificate \
    -O ZED_SDK_Linux_JP.run \
    "https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons" && \
    chmod +x ZED_SDK_Linux_JP.run && \
    ./ZED_SDK_Linux_JP.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* && \
    rm ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Copy everything into the container (submodule files are present on host after git submodule add)
COPY . /workspace/

# Install ROS dependencies.
# All dustynv images ship a custom CUDA OpenCV (opencv-dev 4.8.x-dirty) that owns
# /usr/include/opencv4/ headers. Ubuntu apt opencv packages need those same paths.
# --force-overwrite is the standard container fix: Ubuntu headers install on top,
# dustynv's CUDA runtime .so files remain untouched.
RUN echo 'Dpkg::Options { "--force-overwrite"; };' > /etc/apt/apt.conf.d/99force-overwrite && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -f /etc/apt/apt.conf.d/99force-overwrite && \
    rm -rf /var/lib/apt/lists/*

# TODO TEST 0.1
RUN ls /opt/ros/humble

# Build the workspace.
# The cmake flags are required by zed-ros2-wrapper (from Stereolabs' own Dockerfile.l4t-humble).
#RUN bash -c " \
#    set -e && \
#    source /opt/ros/humble/setup.bash && \
#    printenv | grep CMAKE_PREFIX_PATH && \
#    colcon build --symlink-install \
#      --cmake-args \
#      -DCMAKE_BUILD_TYPE=Release \
#      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
#      -DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined \
#"

# TODO TEST 0.2
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble

RUN bash -c "\
    set -e && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --parallel-workers 1 \
    --cmake-args \
    '-DCMAKE_BUILD_TYPE=Release' \
    '-DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    '-DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined' \
"

# Source the workspace setup
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
