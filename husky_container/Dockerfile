FROM ros:humble-ros-core

# avoid interactive prompts, set UTF-8
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    HUSKY_WS=/husky_ws

# install common tooling & add PPAs
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      wget \
      software-properties-common \
      ca-certificates \
      gnupg2 \
      lsb-release && \
    # GTSAM PPA
    add-apt-repository ppa:borglab/gtsam-release-4.1 && \
    # VSCode repo
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
      | gpg --dearmor > /usr/share/keyrings/microsoft.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] \
      https://packages.microsoft.com/repos/code stable main" \
      > /etc/apt/sources.list.d/vscode.list && \
    rm -rf /var/lib/apt/lists/*

# install ROS Desktop + extras + colcon  
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-pip \
      curl \
      unzip \
      nano \
      code \
      iputils-ping \
      iproute2 \
      net-tools \
      libusb-dev \
      git \
      locales \
      libgtsam-dev \
      libgtsam-unstable-dev \
      ros-${ROS_DISTRO}-desktop-full \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-gazebo-msgs \
      ros-${ROS_DISTRO}-gazebo-plugins \
      ros-${ROS_DISTRO}-gazebo-ros \
      ros-${ROS_DISTRO}-gazebo-ros2-control \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# initialize rosdep
RUN rosdep init && rosdep update

# create workspace
WORKDIR ${HUSKY_WS}

# clone the required ROS 2 packages
WORKDIR ${HUSKY_WS}/src
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git && \
    git clone -b ros2 https://github.com/TixiaoShan/LIO-SAM.git lio-sam && \
    git clone -b humble https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git cmu_exploration

# copy your override YAMLs into each package as needed
COPY local_planner_indoor.launch.py \
  ${HUSKY_WS}/src/autonomous_exploration_development_environment/src/local_planner/launch/local_planner_indoor.launch.py

COPY local_planner.launch.py \
  ${HUSKY_WS}/src/autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch.py

COPY params.yaml \
  ${HUSKY_WS}/src/lio-sam/config/params.yaml

# install ROS dependencies
WORKDIR ${HUSKY_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# build everything
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --merge-install --parallel-workers 4

# expose workspace for bind-mounting or later inspection
VOLUME ["/husky_ws"]

# entrypoint to source both ROS 2 and workspace
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
