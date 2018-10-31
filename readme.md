## ROS beginner tutorial of Publisher Subscriber

Implementation of ROS publisher and subscriber in roscpp


## Pre-requisite

The project requires ROS kinectic and catkin, and is developed on UBUNTU 16.04 LTS.
```
To install ROS please follow the tutorial on: http://wiki.ros.org/kinetic/Installation

To Install catkin: http://wiki.ros.org/catkin

Link for the tutorial: http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem
```

## Build

Before building make sure ROS kinetic and catkin are installed.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/NithishkumarS/beginner_tutorials.git 
cd ..
catkin_make
```

## Running Demo 

To run the demo open a new terminal and type
```
roscore
```
To run talker open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
To run listener open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
To stop the program press ctrl+C in each of the three terminals.


