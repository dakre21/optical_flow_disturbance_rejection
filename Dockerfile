FROM ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 

RUN apt update \
    && apt install -y git \
    && apt install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt install -y software-properties-common \
    && apt install -y net-tools \
    && apt install -y iputils-ping \
    && apt install -y python3 \
    && apt install -y curl \
    && apt install -y libserial-dev \
    && apt install -y minicom 

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt upgrade -y \
    && apt install -y ros-jazzy-desktop \
    && apt install -y ros-dev-tools \
    && apt install -y ros-jazzy-mavros \
    && apt install -y ros-jazzy-mavros-extras \
    && apt install -y python3-vcstool \
    && apt install -y python3-colcon-common-extensions \
    && apt install -y python3-rosdep \
    && apt install -y python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/optical_flow/src/ros_ws

COPY src/ros_ws .

RUN . /opt/ros/jazzy/setup.sh \
    && vcs import src < dependency.repos \
    && colcon build --symlink-install --packages-skip mocap4r2_optitrack_driver 

RUN . /opt/ros/jazzy/setup.sh \
    && ros2 run mavros install_geographiclib_datasets.sh
