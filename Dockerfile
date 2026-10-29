
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    # General
    git \
    wget \
    sudo \
    gnupg2 \
    build-essential \
    cmake \
    python3-pip \
    python-is-python3 \
    python3-dev \

    # GUI / graphics
    x11-apps \
    mesa-utils \
    vainfo \
    mesa-va-drivers \

    # Python stack
    python3-numpy \
    python3-scipy \
    python3-pandas \
    python3-matplotlib \
    python3-lxml \
    python3-yaml \
    python3-openpyxl \
    python3-tk \
    python3-pyqt5 \

    # C++ / robotics
    libeigen3-dev \
    libboost-all-dev \
    libtinyxml2-dev \
    libconsole-bridge-dev \
    pybind11-dev \

    # ROS tooling
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-eigenpy \
    ros-${ROS_DISTRO}-hpp-fcl \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    && rm -rf /var/lib/apt/lists/*

# Build limits (safe defaults)
ENV MAKEFLAGS="-j1"
ENV CMAKE_BUILD_PARALLEL_LEVEL=3
ENV COLCON_MAX_WORKERS=2


# Create robot user 
RUN useradd -m -s /bin/bash robot \
    && echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER robot

# X11 / GUI env
ENV DISPLAY=host.docker.internal:0.0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth
ENV XDG_RUNTIME_DIR=/home/robot/xdg_dir

RUN mkdir -p /home/robot/xdg_dir \
    && chmod 0700 /home/robot/xdg_dir

RUN mkdir -p /home/robot/ws

WORKDIR /home/robot/ws

# Interactive shell setup (user-specific is OK here)
RUN echo "export TERM=xterm-256color" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "alias build='colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]