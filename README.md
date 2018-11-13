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
git checkout Week10_HW
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

