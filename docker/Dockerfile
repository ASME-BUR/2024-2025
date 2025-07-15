FROM ubuntu:22.04

USER root

# Install ROS dependencies
RUN apt update \
    && apt install locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

RUN apt install -y software-properties-common \
    && add-apt-repository universe

# Add ROS repository keys
RUN apt update \
    && apt install curl -y \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb

# Install ROS
RUN apt-get update \
    && apt upgrade -y \
    && apt-get install -y ros-humble-desktop \
                          python3-colcon-common-extensions \
                          python3-rosdep \
                          python3-pip \
                          unzip

# Install BUR dependencies
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-behaviortree-cpp \
                       ros-${ROS_DISTRO}-control-toolbox \
                       ros-${ROS_DISTRO}-image-common \
                       ros-${ROS_DISTRO}-mavros \
                       ros-${ROS_DISTRO}-navigation2 \
                       ros-${ROS_DISTRO}-robot-localization \
                    #    ros-${ROS_DISTRO}-rviz2 \
                    #    ros-${ROS_DISTRO}-rviz-common \
                    #    ros-${ROS_DISTRO}-rviz-default-plugins \
                       libeigen3-dev

RUN rosdep update

WORKDIR /root

COPY . /root/2024-2025
RUN cd 2024-2025 && \
    git submodule init && git submodule update && \
    rosdep install --from-paths ros2_ws/src -y --ignore-src --rosdistro humble

# Install pigpio
WORKDIR /root/2024-2025/ros2_ws/src
RUN wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && \
    cd pigpio-master && \
    make && \
    make install && \
    cd .. && \
    rm -rf master.zip pigpio-master

WORKDIR /root
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/2024-2025/ros2_ws/install/setup.bash" >> ~/.bashrc

