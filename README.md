# ROS Beginner - Publisher / Subcriber 
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/nr-parikh/beginner_tutorials/blob/master/LICENSE)

## Project Overview
This is a basic publisher and subscriber in ROS for ENPM808X. It has two nodes viz.
* Talker (`src/talker.cpp`)
* Listener (`src/listener.cpp`)

## Dependencies 
The dependencies of this repository are:
 ```
* Ubuntu 16.04
* ROS Kinetic
```

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

## Building Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ysshah95/beginner_tutorials.git
cd beginner_tutorials
git branch -a
git checkout Week11_HW
cd ~/catkin_ws
catkin_make
```

## Running the Code using rosrun
After following the build instructions, run following commands in seperate terminals. Make sure your current directory in terminal is ~/catkin_ws/.

Set environment variables and run roscore in that terminal: 
```
source devel/setup.bash
roscore
```

Keep the rosmaster running and open another terminal to run talker node by entering following commands.
```
source devel/setup.bash
rosrun beginner_tutorials talker
```

Open another terminal to run the listener node by entering following commands.
```
source devel/setup.bash
rosrun beginner_tutorials listener
```

>Note: Just like the path of the ROS was sourced in .bashrc file (after installing ROS), same needs to be done for this workspace by writing ```source <path to workspace>/devel/setup.bash``` at the end of the .bashrc file. This avoids the need to source every time one needs to use workspace's packages.

## Running the code using roslaunch
Instead of running the code in different command windows using rosrun, you can also run all of them together using roslaunch by typing the following command in the terminal. It will start both talker and listener nodes. 

Make sure your current directory in terminal is ~/catkin_ws/.

>Note: roscore launches automatically (if it is not running) after the following command is executed. 

```
roslaunch beginner_tutorials chatter.launch
```

You can also change the frequency at which talker node publishes the messages. Use any integer instead of "freq_value" in the following command.

```
roslaunch beginner_tutorials chatter.launch freq:="freq_value"
```

## Service
The talker node is a server that responds to the service requests by change_talker_string service. If you would like to change the string that the talker node publishes, type the following command in the new terminal after both talker and listener nodes are running. It will change the publish message to "new string to be published" in the example below. 

```
rosservice call /change_talker_string "new string to be published"
```

## Log Messages

To see the logger messages in the rqt_console GUI, run the comand below after running the roscore, talker and listener nodes from the instructions above.

```
rosrun rqt_console rqt_console
```
You can also refer to the image file in the "rqt_console output" directory. 

## TF Frames Inspection

The talker node broadcasts tf transforms to ```/talk``` relative to ```/world``` frame. Run the following command when roscore and talker node are running to print tf transforms usinf ```tf_echo```.  

```
rosrun tf tf_echo /world /talk
```

The output after executing tf_echo command looks like below. The reference frame is given first and then the frame to be viewed. 

```
At time 1542166478.044
- Translation: [-0.648, 0.761, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.908, 0.419]
            in RPY (radian) [0.000, -0.000, 2.276]
            in RPY (degree) [0.000, -0.000, 130.426]
At time 1542166479.045
- Translation: [-0.650, 0.760, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.908, 0.418]
            in RPY (radian) [0.000, -0.000, 2.279]
            in RPY (degree) [0.000, -0.000, 130.555]
At time 1542166480.044
- Translation: [-0.643, 0.765, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.906, 0.422]
            in RPY (radian) [0.000, -0.000, 2.270]
            in RPY (degree) [0.000, -0.000, 130.049]
```

To get more detailed information about the transforms between frames, use ```rosrun tf view_frames``` or ```rosrun rqt_tf_tree rqt_tf_tree``` commands.

## Running ROSTest

Testing is the most important feature of developing softwares in Robotics to ensure that the newly created or modified modules does not break the running version of the code. For this repository, two tests have been created, one to check the existance of the service, and one to ensure the change that the service executes. 

The tests have been written using gtest and rostest. To run the tests execute the following commands. 

```
cd ~/catkin_ws/
catkin_make run_tests_beginner_tutorials
```

Tou can also use rostest to run the tests using the following command.

```
rostest beginner_tutorials talker_test.launch
```

The output of the execution of tests looks similar to the one below. 

```
... logging to /home/yashshah/.ros/log/rostest-ubuntu-21491.log
[ROSUNIT] Outputting test results to /home/yashshah/.ros/test_results/beginner_tutorials/rostest-test_talker_test.xml
[Testcase: testtalker_test] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talker_test/test_service_existance][passed]
[beginner_tutorials.rosunit-talker_test/test_Service][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/yashshah/.ros/log/rostest-ubuntu-21491.log

```

## Recording bag files with the launch file

You may run the command below to launch the nodes and record all the topics to a bag file.  

```
roslaunch beginner_tutorials chatter.launch record:=true
```

Press ```ctr+C``` when you want to terminate the recording. The bag file will be in the results/ROSbagFiles directory once the recording is complete.

If you want to launch all the nodes without recording bag file, execute the following command. The default record argument is false.  

```
roslaunch beginner_tutorials chatter.launch
```

## Playing back the bag file with the Listener node demonstration

First, navigate to the folder that has .bag file.

```
cd ~/catkin_ws/src/beginner_tutorials/results/ROSbagFiles
```

To inspect the bag file, ensure that the roscore and listener nodes are running. Then in a new terminal, enter the command below.

```
rosbag play listener.bag
```

You will be able to see the listener node output on the screen. It would be playing the same messages that were recorded.


