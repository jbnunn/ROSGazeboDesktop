FROM dorowu/ubuntu-desktop-lxde-vnc:xenial
LABEL maintainer "jbnunn@gmail.com"

ENV DEBIAN_FRONTEND noninteractive

# Setup your sources list and keys
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Kinetic
RUN apt update
RUN apt-get install -y ros-kinetic-desktop-full
RUN rosdep init && rosdep update

# Install some essentials
RUN apt-get install -y git wget curl nano mercurial python-pip
RUN apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Install Turtlebot Packages
RUN apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard \
    ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino \
    ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs \
    ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
    ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation \
    ros-kinetic-interactive-markers

# Setup the shell
RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc"
RUN /bin/bash -c "echo 'source /opt/ros/kinetic/setup.bash' >> /root/.bashrc"
RUN cp /root/.bashrc /home/ubuntu/.bashrc
RUN /bin/bash -c "source /home/ubuntu/.bashrc"

# Install VS Code and Python extensions
RUN curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
RUN install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
RUN sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
RUN apt-get install -y apt-transport-https
RUN apt-get update
RUN apt-get install -y code

# Install Catkin
RUN apt-get install -y ros-kinetic-catkin python-catkin-tools

# Copy some starter models
RUN mkdir -p /home/ubuntu/.gazebo/
COPY models /home/ubuntu/.gazebo/models


