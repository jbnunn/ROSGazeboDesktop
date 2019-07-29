FROM dorowu/ubuntu-desktop-lxde-vnc:xenial
LABEL maintainer "jbnunn@gmail.com"

ENV DEBIAN_FRONTEND noninteractive

# Setup your sources list and keys
RUN apt-get update && apt-get install -q -y \
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

# Setup the shell
RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc"
RUN /bin/bash -c "echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc"
RUN cp /root/.bashrc /home/ubuntu/.bashrc
RUN /bin/bash -c "source /home/ubuntu/.bashrc"

# Install VS Code and Python extensions
RUN curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
RUN install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
RUN sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
RUN apt-get install -y apt-transport-https
RUN apt-get update
RUN apt-get install -y code
RUN pip install pylint

# Install Catkin
RUN apt-get install -y ros-melodic-catkin python-catkin-tools

# Copy some starter models
RUN mkdir -p /home/ubuntu/.gazebo/
COPY models /home/ubuntu/.gazebo/models


