###############################################################################
# Dockerfile: Ubuntu 22.04 + ROS 2 Humble + CMU Exploration Environment (ROS 2)
###############################################################################

# Base image
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools & set locale
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      curl \
      gnupg2 \
      lsb-release \
      build-essential \
      ca-certificates \
      git \
      wget \
      unzip \
      nano \
      locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository universe

# Add the ROS 2 Humble GPG key into /usr/share/keyrings
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 Humble repository, referencing the keyring
RUN sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libusb-dev \
      ros-humble-desktop-full \
      ros-humble-joy \
      ros-dev-tools \ 
      ros-humble-gazebo-msgs \ 
      ros-humble-gazebo-plugins \
      ros-humble-gazebo-ros \
      ros-humble-gazebo-ros2-control \
      ros-humble-gazebo-ros-pkgs \
      python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*
  
# Initialize rosdep
RUN rosdep init && rosdep update

# Create a ROS 2 workspace folder
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Clone the CMU Autonomous Exploration repository
WORKDIR /ros2_ws/src
RUN git clone https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git cmu_exploration
RUN cd cmu_exploration && \
    git checkout humble || echo "No 'humble' branch—proceeding with default branch"

# Download the Gazebo environments
WORKDIR /ros2_ws/src/cmu_exploration/src/vehicle_simulator/mesh
RUN chmod +x download_environments.sh && \
    ./download_environments.sh

# Source

WORKDIR /ros2_ws/install/
# Install any missing dependencies via rosdep
WORKDIR /ros2_ws
RUN rosdep install --from-paths src/cmu_exploration/src --ignore-src -r -y || echo "rosdep failure is OK if no ROS 2 packages"

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --parallel-workers 4"

# Copy entrypoint script and make it executable
WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# 13) Default to bash if no command is given
CMD ["bash"]
