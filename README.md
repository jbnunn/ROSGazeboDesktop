# Running ROS and Gazebo on an Ubuntu Desktop via Docker

This Dockerfile will install ROS Melodic with Gazebo 9 on Ubuntu 18.04, and give you a VNC interface to work within that environment. Tested and working on Windows 10 and Mac OS X. To install:

    git clone https://github.com/jbnunn/ROSGazeboDesktop
    cd ROSGazeboDesktop
    docker build -t ros-gazebo-desktop .
    
## Test your installation

Start the image and expose port 5900 so you can connect with a VNC client, and/or port 6080 so you can connect via your browser using NoVNC. We'll also expose port 11311 for the ROS master node which we'll need later if we want to communicate to the ROS core and control simualted robots from outside the Docker container.

    docker run -it --rm --name=ros_gazebo_desktop -p 6080:80 -p 5900:5900 -p 11311:1131

Connect to the container using a VNC client or via http://locahost:6080/. From the Ubuntu desktop, open a terminal, and try:

    gazebo worlds/pioneer2dx.world

This should open Gazebo with a simple virtual world and robot:

![Gazebo](./gazebo.png)

Experiment with more worlds by taking a look at the available ones via

    ls /usr/share/gazebo-9/worlds

You can also launch a world with

    roslaunch gazebo_ros empty_world.launch

## Launch a Robot into Gazebo

Let's spawn a robot into our world. If Gazebo isn't running, launch it with `roslaunch gazebo_ros empty_world.launch`. Now, let's spawn a robot into this world. I'll use the ["Create" robot from iRobot](https://www.irobot.com/about-irobot/stem/create-2), which is based off the Roomba platform:

    rosrun gazebo_ros spawn_model -file ~/.gazebo/models/create/model-1_4.sdf -sdf -model Create

You should see `Spawn status: SpawnModel: Successfully spawned entity`. Switch views back to Gazebo and you'll find the Create robot in your virtual world.  

Take a look at the syntax at [http://gazebosim.org/tutorials?tut=ros_roslaunch](http://gazebosim.org/tutorials?tut=ros_roslaunch) for more details.

## Persisting Data

To effectively work with the container and save your data, we'll create a workspace volume on your host that is effectively shared with the container.

    mkdir ros_ws

Now when launching the container, we'll use the `-v` flag to mount `ros_ws` inside the container at `/root/ros_ws`.

* Windows:

        docker run -it --rm --name=ros_gazebo_desktop -p 6080:80 -p 5900:5900 -p 11311:11311 -v %cd%/ros_ws:/root/ros_ws ros-gazebo-desktop

* OS X / Linux:

        docker run -it --rm --name=ros_gazebo_desktop -p 6080:80 -p 5900:5900 -p 11311:11311 -v $PWD/ros_ws:/root/ros_ws ros-gazebo-desktop    

## Other Robot Models and Considerations

* A complete list of the OSRF robots downloaded to your Docker container can be found at [https://bitbucket.org/osrf/gazebo_models/src/default/](https://bitbucket.org/osrf/gazebo_models/src/default/). 
* If you continue your experiments with the Create model, look at [https://gist.github.com/eddiem3/4f257b769d53c492b7ea0dc482cd7caa](https://gist.github.com/eddiem3/4f257b769d53c492b7ea0dc482cd7caa) or [http://guitarpenguin.is-programmer.com/posts/58100.html](http://guitarpenguin.is-programmer.com/posts/58100.html) for info on adding a differential-drive plugin.

## Credits

Based on the following work:

* Dockerfile: [https://github.com/bpinaya/robond-docker](https://github.com/bpinaya/robond-docker)
* Docker base image: [https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/](https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/)