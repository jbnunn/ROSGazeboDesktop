# Running ROS and Gazebo on an Ubuntu Desktop via Docker

This Dockerfile will install ROS Melodic with Gazebo 9 on Ubuntu 18.04, and give you a VNC interface to work within that environment. Tested and working on Windows 10 and Mac OS X. To install:

    git clone https://github.com/jbnunn/ROSGazeboDesktop
    cd ROSGazeboDesktop
    docker build -t ros-gazebo-desktop .
    
## Test your installation

Start the image and expose port 5900 so you can connect with a VNC client, and/or port 6080 so you can connect via your browser using NoVNC. We'll also expose port 11311 for the ROS master node and 11345 for the Gazebo server, which we'll need later if we want to communicate to the ROS core and control simualted robots from outside the Docker container.

    docker run -it 
        -p 6080:80 \ 
        -p 5900:5900 \ 
        -p 11311:11311 \  
        -p 11345:11345 \ 
    ros-gazebo-desktop

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

## Make the Robot Move

Ideally, you won't do work inside the container you've launched. Instead, you'll use the container to show virtual worlds and robots within Gazebo, and control them from outside of the Docker container. Let's do that here.

1. Start the container if it's not running:

        docker run -it 
            -p 6080:80 \ 
            -p 5900:5900 \ 
            -p 11311:11311 \  
            -p 11345:11345 \ 
        ros-gazebo-desktop

2. Connect to the container using a VNC client or via http://locahost:6080/

3. Open a terminal within the container, and from the command line start Gazebo with a virtual world (note we've turned on `verbose` to debug an errors)

        roslaunch gazebo_ros empty_world.launch verbose:=true
        
4. From a terminal on your local computer (i.e., from outside of the Docker container)

### Other Robot Models and Considerations

* A complete list of the OSRF robots downloaded to your Docker container can be found at [https://bitbucket.org/osrf/gazebo_models/src/default/](https://bitbucket.org/osrf/gazebo_models/src/default/). 
* If you continue your experiments with the Create model, look at [https://gist.github.com/eddiem3/4f257b769d53c492b7ea0dc482cd7caa](https://gist.github.com/eddiem3/4f257b769d53c492b7ea0dc482cd7caa) or [http://guitarpenguin.is-programmer.com/posts/58100.html](http://guitarpenguin.is-programmer.com/posts/58100.html) for info on adding a differential-drive plugin.

## Credits

Based on the following work:

* Dockerfile: [https://github.com/bpinaya/robond-docker](https://github.com/bpinaya/robond-docker)
* Docker base image: [https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/](https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/)