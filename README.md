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
git clone --recursive https://github.com/karanvivekbhargava/beginner_tutorials.git
cd ..
catkin_make
```

## Running the Code
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



