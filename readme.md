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

| Directory | Description 			    |
| --------- | ------------------------------------- |
| `src`	    | Contains implementation of the nodes  |
| `srv`	    | Contains the service description      |
| `test`    | Contains the test and its launch files|
| `outputs` | Contains the results and the bag files|
| `launch`  | Holds the xml launch file 	    |
| `include` | COntains the main directories	    |

## Build

Before building make sure ROS kinetic and catkin are installed.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone -b Week11_HW --single-branch  https://github.com/NithishkumarS/beginner_tutorials.git 
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

## Using TF Frames

The modified talker.cpp node broadcasts the /talk frame which has a non-zero translation and rotation with respect to the /world frame. We can verify the TF frames using tf_echo and rqr_tf_tree.

To use tf_echo run the software demo using launch file as mentioned above. Open a new terminal and type
``
rosrun tf tf_echo /world /talk
``
The translation and rotation transform vectors are displayed in the terminal. press ctrl+C in each terminal to stop the program.


With the demo running type the follwing in a new terminal to visualize the tree, this can help us see the link between different 
```
rosrun rqt_tf_tree rqt_tf_tree
```
view_frames produces a diagram of the broadcasted frame. 

To generate a .pdf output file of the frame tree type the following in a new terminal
```
cd ~/catkin_ws
rosrun tf view_frames
```

the pdf can be viewed by entering the following in the terminal
```
evince frames.pdf
```
press ctrl+C to stop the program in each of the terminals.

## ROSTEST

To run the test for talker node modifyText service type the following in terminal.
```
cd ~/catkin_ws
source devel/setup.bash
rostest beginner_tutorials talkerTest.launch 
```
example test output
```
... logging to /home/nithish/.ros/log/rostest-nithish-HP-Pavilion-15-Notebook-PC-19866.log
[ROSUNIT] Outputting test results to /home/nithish/.ros/test_results/beginner_tutorials/rostest-test_testTalker.xml
[Testcase: testtestTalker] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-testTalker/checkServiceAvailability][passed]
[beginner_tutorials.rosunit-testTalker/checkStringModification][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/nithish/.ros/log/rostest-nithish-HP-Pavilion-15-Notebook-PC-19866.log
```
## ROSBAG

To create a new bag file type

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials Week10.launch rosbagEnable:=true
```
press ctrl+C in each terminal window to exit from the program and stop the recording.

or we could run the following command in the terminal 
```
rosrun rosbag record -O filename.bag topic-names
```

Rosbag file in .bag format is present in the outputs folder. To verify the listener node with bag files first play the bag file using the command
```
cd ~/catkin_ws
source devel/setup.bash
cd src/beginner_tutorials/outputs
rosbag play filename.bag
```
Then to start the listener in a new terminal

```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

One can find the terminal running listener node printing the recorded /chatter topic.

press ctrl+C in each of the terminal to exit the program.


