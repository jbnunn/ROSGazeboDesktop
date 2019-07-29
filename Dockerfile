FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt
LABEL maintainer "jbnunn@gmail.com"

# Setup your sources list and keys
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Melodic
RUN apt update
RUN apt install -y ros-melodic-desktop-full
RUN rosdep init && rosdep update

# Install some essentials
RUN apt-get install -y git wget curl nano mercurial
RUN apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Setup the shell
RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc"
RUN /bin/bash -c "echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc && source /root/.bashrc"

# Install Catkin
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-melodic-catkin python-catkin-tools

# Download OSRF robot models for Gazebo.
RUN mkdir -p ~/.gazebo
RUN hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models

