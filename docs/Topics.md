# Published Topics
- /cmd_vel (`geometry_msgs/Twist`)
  - velocity command
- ~\<name>/candidate_trajectories (`visualization_msgs/MarkerArray`)
  - candidate trajectories
  - for visualization
- ~\<name>/finish_flag (`std_msgs/Bool`)
  - this flag is true when the robot reaches the goal
- ~\<name>/predict_footprints (`visualization_msgs/MarkerArray`)
  - predicted footprints on selected trajectory
  - for visualization
- ~\<name>/selected_trajectory (`visualization_msgs/Marker`)
  - selected trajectory
  - for visualization

# Subscribed Topics
## Necessary topics
- /local_map (`nav_msgs/OccupancyGrid`)
  - robot-centered costmap
  - the cells with an occupancy probability of 100 are considered as obstacles
- /move_base_simple/goal (`geometry_msgs/PoseStamped`)
  - goal pose
- /odom (`nav_msgs/Odometry`)
  - robot's odometry

## Optional Topics
- /dist_to_goal_th (`std::msgs::Float64`)
  - distance threshold to the goal for judging the goal is reached
  - If you want to change the distance threshold to the goal from the default value, publish the distance to this topic
- /scan (`sensor_msgs/LaserScan`)
  - laser scan data
  - Default input is `/local_map`
  - If laser scan is used, set `USE_SCAN_AS_INPUT` to `true`
- /footprint (`geometry_msgs/PolygonStamped`)
  - robot footprint
  - If robot footprint is used, set `USE_FOOTPRINT` to `true`
  - `footprint_publisher` node in [amsl_navigation_utils](https://github.com/amslabtech/amsl_navigation_utils.git) repository publishes rectangular footprint
- /path (`nav_msgs/Path`)
  - a part of the global path (edge)
  - for path cost
  - Default evaluation does not use path cost
  - If path cost is used, set `USE_PATH_COST` to `true`
    - Give a part of the global path (edge)
- /target_velocity (`geometry_msgs/Twist`)
  - target velocity of the robot