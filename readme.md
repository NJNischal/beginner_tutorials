## Overview

This is a ROS package that shows the working of a publisher and a subscriber node. The publisher node publishes a topic (Which here is a custom string message) and the subscriber node subscribes to this topic.


## Building the ROS package
```
# Source environment 

source /opt/ros/kinetic/setup.bash

# Create and build catkin workspace:

mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ 
catkin_make
```

## Running the ROS package
```
# To start ROS master, open a new terminal and enter:

roscore

# To start the publisher, open a new terminal and enter:

cd ~/catkin_ws/ 
source devel/setup.bash
rosrun beginner_tutorials talker

# To start the subscriber, open a new terminal and enter:

cd ~/catkin_ws/ 
source devel/setup.bash
rosrun beginner_tutorials listener 
```

## Assumptions/Dependencies

ROS distro used here is Kinetic

