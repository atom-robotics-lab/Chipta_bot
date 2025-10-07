# Use official Ubuntu 22.04 base image (multi-arch)
FROM ubuntu:22.04

# Set noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update & install required dependencies
RUN apt-get update && apt-get install -y \  
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    git \
    wget \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 Humble desktop
RUN apt-get update && apt-get install -y ros-humble-desktop && rm -rf /var/lib/apt/lists/*

# Source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set default working directory
WORKDIR /workspace

# Default command when container starts
CMD ["/bin/bash"]
