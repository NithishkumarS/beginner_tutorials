## ROS beginner tutorial of Publisher Subscriber

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

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
git clone -b Week10_HW --single-branch https://github.com/NithishkumarS/beginner_tutorials.git 
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

## Using service to modify the base string

To modify the default text run the demo either using the launch file as mentioned above (or you can run the demo without the launch file as explained above)

After the demo starts open a new terminal and type

```
cd catkin_ws
source devel/setup.bash
rosservice call /modifyText <your string>
```
You will notice that the default text changes to the text you have entered.

an example would be
```
rosservice call /changeString Nithish
```

## Changing the chatter frequency via command line using launch file

To modify the loop frequency open run the demo using launch file using the following command
```
roslaunch beginner_tutorials Week10.launch use_param:=<int value greater than 0>
```
an example would be

roslaunch beginner_tutorials hw10.launch frequency:=15

## Modifying the severity type

After starting the demo run this is a seperate terminal
```
rosservice call /talker/set_logger_level ros.beginner_tutorials <logger_level>
```


