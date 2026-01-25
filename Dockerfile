# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# --- 1. Essential Setup (Tools & Keys) ---
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    git \
    build-essential \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8


RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# --- 2. Install ROS 1 Noetic ---
# Setup sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install Noetic Desktop Full + Industrial/MoveIt packages
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-moveit \
    ros-noetic-industrial-core \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-joy \
    ros-noetic-moveit-visual-tools \
    ros-noetic-graph-msgs \
    ros-noetic-rosparam-shortcuts \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-code-coverage \
    ros-noetic-franka-description \
    ros-noetic-spacenav-node \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (Standard for ROS1)
RUN rosdep init && rosdep update

# --- 3. Install ROS 2 Foxy ---
# Setup sources
# 1. Install software-properties-common to get add-apt-repository
# 2. Enable Universe repository
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. Add ROS 2 Source list
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# Install Foxy Base + Bridge + Python tools
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-ros1-bridge \
    ros-foxy-rmw-cyclonedds-cpp \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# --- 4. Configure .bashrc ---
RUN echo "" >> ~/.bashrc && \
    echo "# Custom ROS Paths" >> ~/.bashrc && \
    echo "export ROS1_INSTALL_PATH=/opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "export ROS2_INSTALL_PATH=/opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Default entrypoint
CMD ["bash"]