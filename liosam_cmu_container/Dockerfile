FROM ros:humble-ros-core

# avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# install the tool that provides add-apt-repository
RUN apt-get update && \
    apt-get install -y --no-install-recommends wget software-properties-common ca-certificates && \
    rm -rf /var/lib/apt/lists/*

#add the GTSAM PPA
RUN add-apt-repository ppa:borglab/gtsam-release-4.1

# Add microsoft GPG key and repo for vscode
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
    | gpg --dearmor > /usr/share/keyrings/microsoft.gpg && \
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] \
    https://packages.microsoft.com/repos/code stable main" \
    > /etc/apt/sources.list.d/vscode.list

# Install system tools, ROS packages & colcon
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      python3-colcon-common-extensions \
      curl \
      gnupg2 \
      lsb-release \
      ca-certificates \
      git \
      libgtsam-dev \
      libgtsam-unstable-dev \
      unzip \
      nano \
      code \
      locales \
      software-properties-common \
      iputils-ping \
      iproute2 \
      net-tools \
      libusb-dev \
      ros-humble-desktop-full \
      ros-humble-joy \
      ros-dev-tools \
      ros-humble-gazebo-msgs \
      ros-humble-gazebo-plugins \
      ros-humble-gazebo-ros \
      ros-humble-gazebo-ros2-control \
      ros-humble-gazebo-ros-pkgs && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create and configure workspace
WORKDIR /colcon_ws/src

# Clone CMU autonomous exploration environment
RUN git clone https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git cmu_exploration && \
    cd cmu_exploration && \
    git checkout humble || true

# Download the Gazebo environments
WORKDIR /colcon_ws/src/cmu_exploration/src/vehicle_simulator/mesh
RUN chmod +x download_environments.sh && ./download_environments.sh

# Clone LIO-SAM
WORKDIR /colcon_ws/src
RUN git clone https://github.com/TixiaoShan/LIO-SAM.git lio-sam && \
    cd lio-sam && \
    git checkout ros2 || true

# Install any missing ROS dependencies
WORKDIR /colcon_ws
RUN rosdep install --from-paths src/cmu_exploration/src --ignore-src -r -y || echo "Ignoring rosdep failures"

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --merge-install --parallel-workers 4

# Copy and enable the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
