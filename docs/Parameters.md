# Parameters
## Planner Parameters
 (config/dwa_param.yaml)
### Target Parameters
- ~\<name>/<b>TARGET_VELOCITY</b> (double, default: `0.55` [m/s]):<br>
  The max target velocity of robot

### Simulation Parameters
- ~\<name>/<b>PREDICT_TIME</b> (double, default: `3.0` [s]):<br>
  The amount of time to simulate trajectories
- ~\<name>/<b>SIM_TIME_SAMPLES</b> (int, default: `10`):<br>
  The number of samples to use when simulating trajectories
- ~\<name>/<b>SIM_PERIOD</b> (double, default: `0.1` [s]):<br>
  The simulation time related to the dynamic window. The product of this parameter and the acceleration is the amount of movement of the dynamic window.
- ~\<name>/<b>SIM_DIRECTION</b> (double, default: `1.57` [rad]):<br>
  The simulated turning angle when turning on the spot
- ~\<name>/<b>SLOW_VELOCITY_TH</b> (double, default: `0.1` [m/s]):<br>
  The threshold for slow velocity. If the robot's velocity is less than this value, yawrate less than `MIN_YAWRATE` are not sampled.
- ~\<name>/<b>VELOCITY_SAMPLES</b> (int, default: `3`):<br>
  The number of samples to use when searching for the best velocity
- ~\<name>/<b>YAWRATE_SAMPLES</b> (int, default: `20`):<br>
  The number of samples to use when searching for the best yawrate

### Cost Parameters
- ~\<name>/<b>OBSTACLE_COST_GAIN</b> (double, default: `1.0`):<br>
  The weighting for how large the obstacle cost should be. Multiplied by the normalized obstacle cost. When the robot is far from obstacles, the cost is low.
- ~\<name>/<b>TO_GOAL_COST_GAIN</b> (double, default: `0.8`):<br>
  The weighting for how large the goal cost should be. Multiplied by the normalized goal cost. When the robot is close to the goal, the cost is low.
- ~\<name>/<b>SPEED_COST_GAIN</b> (double, default: `0.4`):<br>
  The weighting for how large the speed cost should be. Multiplied by the normalized speed cost. When the robot is fast, the cost is low.
- ~\<name>/<b>PATH_COST_GAIN</b> (double, default: `0.4`):<br>
  The weighting for how large the path cost should be. Multiplied by the normalized path cost. When the robot is close to the path, the cost is low.
- ~\<name>/<b>ANGLE_RESOLUTION</b> (double, default: `0.087` [rad]):<br>
  Search obstacle by this resolution
- ~\<name>/<b>OBS_RANGE</b> (double, default: `2.5` [m]):<br>
  The maximum measurement distance to be considered when calculating obstacle cost

### Goal Tolerance Parameters
- ~\<name>/<b>GOAL_THRESHOLD</b> (double, default: `0.1` [m]):<br>
  The tolerance for the robot in position when achieving its goal
- ~\<name>/<b>TURN_DIRECTION_THRESHOLD</b> (double, default: `0.1` [rad]):<br>
  The tolerance for the robot in yaw/rotation when achieving its goal

### Other Parameters
- ~\<name>/<b>ANGLE_TO_GOAL_TH</b> (double, default: `pi` [rad]):<br>
  The absolute angle to the goal that is considered as the goal direction. If the angle to the goal is less than this value, the robot turn to the goal.


## Robot Parameters
 (config/robot_param.yaml)
### Frame Parameter
- ~\<name>/<b>ROBOT_FRAME</b> (string, default: `base_link`):<br>
  The coordinate frame of robot

### Robot Size Parameters
- ~\<name>/<b>ROBOT_RADIUS</b> (double, default: `0.1` [m]):<br>
  If parameter "USE_FOOTPRINT" is set to true, this parameter is not used.
- ~\<name>/<b>FOOTPRINT_PADDING</b> (double, default: `0.01` [m]):<br>
  If localmap contains padding, you should set this parameter to 0.0.

### Robot Configuration Parameters
- ~\<name>/<b>MAX_VELOCITY</b> (double, default: `1.0` [m/s]):<br>
  The maximum translational velocity of the robot
- ~\<name>/<b>MIN_VELOCITY</b> (double, default: `0.0` [m/s]):<br>
  The minimum translational velocity of the robot
- ~\<name>/<b>MAX_YAWRATE</b> (double, default: `1.0` [rad/s]):<br>
  The maximum yawrate of the robot
- ~\<name>/<b>MIN_YAWRATE</b> (double, default: `0.05` [rad/s]):<br>
  The minimum yawrate of the robot
- ~\<name>/<b>MAX_IN_PLACE_YAWRATE</b> (double, default: `0.6` [rad/s]):<br>
  The maximum yawrate of the robot when turning on the spot
- ~\<name>/<b>MIN_IN_PLACE_YAWRATE</b> (double, default: `0.3` [rad/s]):<br>
  The minimum yawrate of the robot when turning on the spot
- ~\<name>/<b>MAX_ACCELERATION</b> (double, default: `0.5` [m/s^2]):<br>
  The maximum acceleration of the robot
- ~\<name>/<b>MAX_DECELERATION</b> (double, default: `2.0` [m/s^2]):<br>
  The maximum deceleration of the robot
- ~\<name>/<b>MAX_D_YAWRATE</b> (double, default: `3.2` [rad/s^2]):<br>
  The maximum yawrate acceleration of the robot


## Other Parameters
 (launch/local_planner.launch)
### Main Loop Parameter
- ~\<name>/<b>HZ</b> (double, default: `20` [Hz]):<br>
  The rate of main loop

### Frame Parameter
- ~\<name>/<b>GLOBAL_FRAME</b> (string, default: `map`):<br>
  The coordinate frame of map

### Topic Parameter
- ~\<name>/<b>SUBSCRIBE_COUNT_TH</b> (double, default: `3`):<br>
  The allowable number of control loops if topic is not reached. If the number exceeds this value, set velocity and yawrate to 0.0.

### Goal Parameter
- ~\<name>/<b>SLEEP_TIME_AFTER_FINISH</b> (double, default: `0.5` [s]):<br>
  The sleep time after reaching the goal. Prevent bugs that occur on other nodes by not continuously publishing the finish flag when the goal is reached.

### Visualization Parameter
- ~\<name>/<b>V_PATH_WIDTH</b> (double, default: `0.05` [m]):<br>
  The width of the local path visualization. The selected trajectory's width is this value. The candidate trajectories's width is 0.4 times this value. The footprint frame visualization's width is 0.2 times this value.

### Option
- ~\<name>/<b>USE_FOOTPRINT</b> (bool, default: `false`):<br>
  If footprint is used, set to true.
- ~\<name>/<b>USE_PATH_COST</b> (bool, default: `false`):<br>
  If path cost is used, set to true.
- ~\<name>/<b>USE_SCAN_AS_INPUT</b> (bool, default: `false`):<br>
  If scan is used instead of localmap, set to true.
