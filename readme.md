## Beginner Tutorials for Publisher and Subscriber Nodes
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This is a ROS package that shows the working of a publisher and a subscriber node. The publisher node publishes a topic (Which here is a custom string message) and the subscriber node subscribes to this topic.

There is a launch file 'week10HW.launch' which can be used to simultaneously launch both talker and listener nodes. A service named 'editString.srv' is used to edit output string by the client.


## Building the ROS package
```
# Source environment
source /opt/ros/kinetic/setup.bash

# Create and build catkin workspace:
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ 
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/NJNischal/beginner_tutorials.git
cd ..
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

## Using roslaunch
```
# To use roslaunch file, type the following in the terminal
roslaunch begineer_tutorials week10HW.launch requency:=1
# (Here we can set the frequency to any other value we like.)
```

## ROS Service
```
# To change the output text string, please type the following in a new terminal after following the steps needed to source the workspace:
rosservice call /editString "This is the newly modified string message"
```

## RQT console, logger
```
# To run the RQT console and logger, type the following codes in two new terminals:
# (For console)
rqt_console


# (For logger)

rosrun rqt_logger_level rqt_logger_level
```

## TF frames

The talker node has been changed to boradcast static tf frames.To visualize the frames, use the following commands in the terminal seperately

```
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf tf_echo /world /talk
rosrun tf view_frames
```

## Unit testing

For testing the program, use the launch file in the test folder. 
Run the following commands:
```
cd ~/catkin_ws
catkin_make
rostest beginner_tutorials talkerTest.launch
```

## Rosbag usage

To run the ros bag, use the following command to start recording for 10 seconds: (Change the value to the desired amount of time in this command)

```
rosbag record --duration=10 -a -O rostopicsRecord.bag
```
This records for 10 seconds and saves it as a rosbag file. To play the rosbag file saved in my repo, run the following commands in new terminals:
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```
In another terminal, type:
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play rostopicsRecord.bag
```


## Assumptions/Dependencies
1) ROS distro used here is: 'Kinetic'. 
2) Minimum version of CMake required is: 2.8.3
3) Ubuntu version used: 16.04 LTS
