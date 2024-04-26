# dwa_planner

[![CI](https://github.com/amslabtech/dwa_planner/workflows/CI/badge.svg)](https://github.com/amslabtech/dwa_planner/actions)
[![issue_opened](https://img.shields.io/github/issues/amslabtech/dwa_planner.svg)](https://github.com/amslabtech/dwa_planner/issues?q=is%3Aopen+is%3Aissue)
[![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/dwa_planner.svg)](https://github.com/amslabtech/dwa_planner/issues?q=is%3Aissue+is%3Aclosed)

## Overview
ROS implementation of DWA(Dynamic Window Approach) Planner

The API documentation is available at [https://amslabtech.github.io/dwa_planner/](https://amslabtech.github.io/dwa_planner/)

**Note: This simulator is not provided.**

![demo1](docs/images/demo_dwa.gif)

## Environment
- Ubuntu 20.04
- ROS Noetic

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/amslabtech/dwa_planner.git

# build
cd /path/to/your/catkin_ws
rosdep install --from-paths src --ignore-src -y      # Install dependencies
catkin build dwa_planner -DCMAKE_BUILD_TYPE=Release  # Release build is recommended
```

## How to use
```
roslaunch dwa_planner local_planner.launch
```

## Running the demo
### Using dataset
```
roslaunch dwa_planner local_planner.launch
rosbag play /path/to/your/dataset.bag
```

### Using simulatior
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# build
cd /path/to/your/catkin_ws
rosdep install --from-paths src --ignore-src -y # Install dependencies
catkin build -DCMAKE_BUILD_TYPE=Release         # Release build is recommended

# run demo
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch dwa_planner local_planner.launch use_scan_as_input:=True
```
![demo3](docs/images/demo_dwa_3.gif)


## Node I/O

![dwa_planner I/O diagram](docs/images/dwa_planner_io.png)

### Published/Subscribed Topics
Access [here](docs/Topics.md)

### Runtime requirement
- TF (from GLOBAL_FRAME to ROBOT_FRAME) is required

## Parameters
Access [here](docs/Parameters.md)

## References
- [D. Fox,  W. Burgard, and S.Thrun, "The dynamic window approach to collision avoidance", IEEE Robotics Automation Magazine, 1997.](https://ieeexplore.ieee.org/abstract/document/580977)
