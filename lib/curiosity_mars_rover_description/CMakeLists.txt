################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(curiosity_mars_rover_description)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


################################################################################
# Setup for python modules and scripts
################################################################################

catkin_python_setup()


################################################################################
# Declare ROS messages, services and actions
################################################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  TurnCommand.msg
#)



################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

################################################################################
# Build
################################################################################

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} )

################################################################################
# Install
################################################################################

catkin_install_python(PROGRAMS
  nodes/ackerman_6_wheels.py
  nodes/arm_and_mast.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf models worlds)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
