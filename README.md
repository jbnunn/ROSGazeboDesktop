# Running ROS and Gazebo on Docker

This Dockerfile will install ROS Melodic with Gazebo 9 on Ubuntu 18.04, and give you a VNC interface to work within that environment. To install:

    docker build -t ROSGazebo .
    
## Test your installation

Start the image and expose port 5900 so you can connect with a VNC client, 

    docker run -it -rm -p 6080:80 -p 5900:5900 ROSGazebo

Connect to the image using a VNC client or via http://locahost:6080/. From the Ubuntu desktop, open a terminal, and try:

    gazebo worlds/pioneer2dx.world

This should open Gazebo with a simple virtual world and robot:

![Gazebo](./gazebo.png)

Experiment with more worlds if you want, by taking a look at the available ones via

    ls /usr/share/gazebo-9/worlds

You can also launch a world with

    roslaunch gazebo_ros empty_world.launch

## Launch a Robot into Gazebo

Let's spawn a robot into our world. If Gazebo isn't running, launch it with `roslaunch gazebo_ros empty_world.launch`. Now, let's spawn a robot into this world. I'll use the ["Create" robot from iRobot](https://www.irobot.com/about-irobot/stem/create-2). This assumes you allowed the Dockerfile to download robot models from OSRF.

    rosrun gazebo_ros spawn_model -file ~/.gazebo/models/create/model-1_4.sdf -sdf -model Create

You should see `Spawn status: SpawnModel: Successfully spawned entity`. Switch views back to Gazebo and you'll find the Create robot in your virtual world.  

Take a look at the syntax at [http://gazebosim.org/tutorials?tut=ros_roslaunch](http://gazebosim.org/tutorials?tut=ros_roslaunch) for more details.

### Other Robot Models and Considerations

* A complete list of the OSRF robots downloaded to your Docker container can be found at [https://bitbucket.org/osrf/gazebo_models/src/default/](https://bitbucket.org/osrf/gazebo_models/src/default/). 
* If you continue your experiments with the Create model, look at [https://gist.github.com/eddiem3/4f257b769d53c492b7ea0dc482cd7caa](https://gist.github.com/eddiem3/4f257b769d53c492b7ea0dc482cd7caa) or [http://guitarpenguin.is-programmer.com/posts/58100.html](http://guitarpenguin.is-programmer.com/posts/58100.html) for info on adding a differential-drive plugin.



    


### Credits

Based on the following work:

* Dockerfile: [https://github.com/bpinaya/robond-docker](https://github.com/bpinaya/robond-docker)
* Docker base image: [https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/](https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/)