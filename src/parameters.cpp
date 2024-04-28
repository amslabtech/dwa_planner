// Copyright 2020 amsl

#include <algorithm>
#include <string>

#include "dwa_planner/dwa_planner.h"

void DWAPlanner::load_params(void)
{
  // - A -
  local_nh_.param<double>("ANGLE_RESOLUTION", angle_resolution_, 0.087);
  local_nh_.param<double>("ANGLE_TO_GOAL_TH", angle_to_goal_th_, M_PI);
  // - F -
  local_nh_.param<double>("FOOTPRINT_PADDING", footprint_padding_, 0.01);
  // - G -
  local_nh_.param<std::string>("GLOBAL_FRAME", global_frame_, std::string("map"));
  local_nh_.param<double>("GOAL_THRESHOLD", dist_to_goal_th_, 0.1);
  // - H -
  local_nh_.param<double>("HZ", hz_, 20);
  // - M -
  local_nh_.param<double>("MAX_ACCELERATION", max_acceleration_, 0.5);
  local_nh_.param<double>("MAX_DECELERATION", max_deceleration_, 2.0);
  local_nh_.param<double>("MAX_D_YAWRATE", max_d_yawrate_, 3.2);
  local_nh_.param<double>("MAX_IN_PLACE_YAWRATE", max_in_place_yawrate_, 0.6);
  local_nh_.param<double>("MAX_VELOCITY", max_velocity_, 1.0);
  local_nh_.param<double>("MAX_YAWRATE", max_yawrate_, 1.0);
  local_nh_.param<double>("MIN_IN_PLACE_YAWRATE", min_in_place_yawrate_, 0.3);
  local_nh_.param<double>("MIN_VELOCITY", min_velocity_, 0.0);
  local_nh_.param<double>("MIN_YAWRATE", min_yawrate_, 0.05);
  // - O -
  local_nh_.param<double>("OBSTACLE_COST_GAIN", obs_cost_gain_, 1.0);
  local_nh_.param<double>("OBS_RANGE", obs_range_, 2.5);
  // - P -
  local_nh_.param<double>("PATH_COST_GAIN", path_cost_gain_, 0.4);
  local_nh_.param<double>("PREDICT_TIME", predict_time_, 3.0);
  // - R -
  local_nh_.param<std::string>("ROBOT_FRAME", robot_frame_, std::string("base_link"));
  local_nh_.param<double>("ROBOT_RADIUS", robot_radius_, 0.1);
  // - S -
  local_nh_.param<double>("SIM_DIRECTION", sim_direction_, M_PI / 2.0);
  local_nh_.param<double>("SIM_PERIOD", sim_period_, 0.1);
  local_nh_.param<int>("SIM_TIME_SAMPLES", sim_time_samples_, 10);
  local_nh_.param<double>("SLEEP_TIME_AFTER_FINISH", sleep_time_after_finish_, 0.5);
  local_nh_.param<double>("SLOW_VELOCITY_TH", slow_velocity_th_, 0.1);
  local_nh_.param<double>("SPEED_COST_GAIN", speed_cost_gain_, 0.4);
  local_nh_.param<int>("SUBSCRIBE_COUNT_TH", subscribe_count_th_, 3);
  // - T -
  local_nh_.param<double>("TARGET_VELOCITY", target_velocity_, 0.55);
  local_nh_.param<double>("TO_GOAL_COST_GAIN", to_goal_cost_gain_, 0.8);
  local_nh_.param<double>("TURN_DIRECTION_THRESHOLD", turn_direction_th_, 0.1);
  // - U -
  local_nh_.param<bool>("USE_FOOTPRINT", use_footprint_, false);
  local_nh_.param<bool>("USE_PATH_COST", use_path_cost_, false);
  local_nh_.param<bool>("USE_SCAN_AS_INPUT", use_scan_as_input_, false);
  // - V -
  local_nh_.param<int>("VELOCITY_SAMPLES", velocity_samples_, 3);
  local_nh_.param<double>("V_PATH_WIDTH", v_path_width_, 0.05);
  // - Y -
  local_nh_.param<int>("YAWRATE_SAMPLES", yawrate_samples_, 20);

  target_velocity_ = std::min(target_velocity_, max_velocity_);
}

void DWAPlanner::print_params(void)
{
  // - A -
  ROS_INFO_STREAM("ANGLE_RESOLUTION: " << angle_resolution_);
  ROS_INFO_STREAM("ANGLE_TO_GOAL_TH: " << angle_to_goal_th_);
  // - F -
  ROS_INFO_STREAM("FOOTPRINT_PADDING: " << footprint_padding_);
  // - G -
  ROS_INFO_STREAM("GLOBAL_FRAME: " << global_frame_);
  ROS_INFO_STREAM("GOAL_THRESHOLD: " << dist_to_goal_th_);
  // - H -
  ROS_INFO_STREAM("HZ: " << hz_);
  // - M -
  ROS_INFO_STREAM("MAX_ACCELERATION: " << max_acceleration_);
  ROS_INFO_STREAM("MAX_DECELERATION: " << max_deceleration_);
  ROS_INFO_STREAM("MAX_D_YAWRATE: " << max_d_yawrate_);
  ROS_INFO_STREAM("MAX_IN_PLACE_YAWRATE: " << max_in_place_yawrate_);
  ROS_INFO_STREAM("MAX_VELOCITY: " << max_velocity_);
  ROS_INFO_STREAM("MAX_YAWRATE: " << max_yawrate_);
  ROS_INFO_STREAM("MIN_IN_PLACE_YAWRATE: " << min_in_place_yawrate_);
  ROS_INFO_STREAM("MIN_VELOCITY: " << min_velocity_);
  ROS_INFO_STREAM("MIN_YAWRATE: " << min_yawrate_);
  // - O -
  ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << obs_cost_gain_);
  ROS_INFO_STREAM("OBS_RANGE: " << obs_range_);
  // - P -
  ROS_INFO_STREAM("PATH_COST_GAIN: " << path_cost_gain_);
  ROS_INFO_STREAM("PREDICT_TIME: " << predict_time_);
  // - R -
  ROS_INFO_STREAM("ROBOT_FRAME: " << robot_frame_);
  ROS_INFO_STREAM("ROBOT_RADIUS: " << robot_radius_);
  // - S -
  ROS_INFO_STREAM("SIM_DIRECTION: " << sim_direction_);
  ROS_INFO_STREAM("SIM_PERIOD: " << sim_period_);
  ROS_INFO_STREAM("SIM_TIME_SAMPLES: " << sim_time_samples_);
  ROS_INFO_STREAM("SLEEP_TIME_AFTER_FINISH: " << sleep_time_after_finish_);
  ROS_INFO_STREAM("SLOW_VELOCITY_TH: " << slow_velocity_th_);
  ROS_INFO_STREAM("SPEED_COST_GAIN: " << speed_cost_gain_);
  ROS_INFO_STREAM("SUBSCRIBE_COUNT_TH: " << subscribe_count_th_);
  // - T -
  ROS_INFO_STREAM("TARGET_VELOCITY: " << target_velocity_);
  ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << to_goal_cost_gain_);
  ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << turn_direction_th_);
  // - U -
  ROS_INFO_STREAM("USE_FOOTPRINT: " << use_footprint_);
  ROS_INFO_STREAM("USE_PATH_COST: " << use_path_cost_);
  ROS_INFO_STREAM("USE_SCAN_AS_INPUT: " << use_scan_as_input_);
  // - V -
  ROS_INFO_STREAM("VELOCITY_SAMPLES: " << velocity_samples_);
  ROS_INFO_STREAM("V_PATH_WIDTH: " << v_path_width_);
  // - Y -
  ROS_INFO_STREAM("YAWRATE_SAMPLES: " << yawrate_samples_);
}
