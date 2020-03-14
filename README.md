# dwa_planner

[![Build Status](https://travis-ci.org/amslabtech/dwa_planner.svg?branch=master)](https://travis-ci.org/amslabtech/dwa_planner)
![issue_opened](https://img.shields.io/github/issues/amslabtech/dwa_planner.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/dwa_planner.svg)


![demo_dwa](doc/demo_dwa.gif)

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Install and Build

```
cd catkin_workspace/src
git clone https://github.com/amslabtech/dwa_planner.git
cd ..
catkin_make
```

## Node I/O

![dwa_planner I/O diagram](doc/images/dwa_planner_io.png)

#### Parameters
- HZ
  - main loop rate (default: 20[Hz])
- TARGET_VELOCITY
  - max velocity of robot's target velocity (default: 0.8[m/s])
- ROBOT_FRAME
  - robot's coordinate frame (default: base_link)
  
#### Runtime requirement
- TF (from LocalMap_FRAME to ROBOT_FRAME) is required

## How to Use
- for local path planning
```
roslaunch dwa_planner local_planner.launch
```

## References
- D. Fox,  W. Burgard, and S.Thrun, "The dynamic window approach to collision avoidance", IEEE Robotics Automation Magazine, 1997.

(https://ieeexplore.ieee.org/abstract/document/580977)
