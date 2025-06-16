// Copyright 2020 amsl

#include <algorithm>
#include <string>
#include <cmath> // For M_PI

#include "dwa_planner/dwa_planner.h" // Assuming this header includes rclcpp/rclcpp.hpp

void DWAPlanner::load_params(void)
{
  // In ROS 2, you declare parameters and then get their values.
  // The 'this->' prefix is used to call methods of the rclcpp::Node base class.
  // Declare and get parameters for DWAPlanner.
  // Each parameter needs to be declared, then retrieved.
  // - A -
  this->declare_parameter<double>("ANGLE_RESOLUTION", 0.087);
  this->get_parameter("ANGLE_RESOLUTION", angle_resolution_);
  this->declare_parameter<double>("ANGLE_TO_GOAL_TH", M_PI);
  this->get_parameter("ANGLE_TO_GOAL_TH", angle_to_goal_th_);
  // - F -
  this->declare_parameter<double>("FOOTPRINT_PADDING", 0.01);
  this->get_parameter("FOOTPRINT_PADDING", footprint_padding_);
  // - G -
  this->declare_parameter<std::string>("GLOBAL_FRAME", "map");
  this->get_parameter("GLOBAL_FRAME", global_frame_);
  this->declare_parameter<double>("GOAL_THRESHOLD", 0.1);
  this->get_parameter("GOAL_THRESHOLD", dist_to_goal_th_);
  // - H -
  this->declare_parameter<double>("HZ", 20.0);
  this->get_parameter("HZ", hz_);
  // - M -
  this->declare_parameter<double>("MAX_ACCELERATION", 0.5);
  this->get_parameter("MAX_ACCELERATION", max_acceleration_);
  this->declare_parameter<double>("MAX_DECELERATION", 2.0);
  this->get_parameter("MAX_DECELERATION", max_deceleration_);
  this->declare_parameter<double>("MAX_D_YAWRATE", 3.2);
  this->get_parameter("MAX_D_YAWRATE", max_d_yawrate_);
  this->declare_parameter<double>("MAX_IN_PLACE_YAWRATE", 0.6);
  this->get_parameter("MAX_IN_PLACE_YAWRATE", max_in_place_yawrate_);
  this->declare_parameter<double>("MAX_VELOCITY", 1.0);
  this->get_parameter("MAX_VELOCITY", max_velocity_);
  this->declare_parameter<double>("MAX_YAWRATE", 1.0);
  this->get_parameter("MAX_YAWRATE", max_yawrate_);
  this->declare_parameter<double>("MIN_IN_PLACE_YAWRATE", 0.3);
  this->get_parameter("MIN_IN_PLACE_YAWRATE", min_in_place_yawrate_);
  this->declare_parameter<double>("MIN_VELOCITY", 0.0);
  this->get_parameter("MIN_VELOCITY", min_velocity_);
  this->declare_parameter<double>("MIN_YAWRATE", 0.05);
  this->get_parameter("MIN_YAWRATE", min_yawrate_);
  // - O -
  this->declare_parameter<double>("OBSTACLE_COST_GAIN", 1.0);
  this->get_parameter("OBSTACLE_COST_GAIN", obs_cost_gain_);
  this->declare_parameter<double>("OBS_RANGE", 2.5);
  this->get_parameter("OBS_RANGE", obs_range_);
  // - P -
  this->declare_parameter<double>("PATH_COST_GAIN", 0.4);
  this->get_parameter("PATH_COST_GAIN", path_cost_gain_);
  this->declare_parameter<double>("PREDICT_TIME", 3.0);
  this->get_parameter("PREDICT_TIME", predict_time_);
  // - R -
  this->declare_parameter<std::string>("ROBOT_FRAME", "base_link");
  this->get_parameter("ROBOT_FRAME", robot_frame_);
  this->declare_parameter<double>("ROBOT_RADIUS", 0.1);
  this->get_parameter("ROBOT_RADIUS", robot_radius_);
  // - S -
  this->declare_parameter<double>("SIM_DIRECTION", M_PI / 2.0);
  this->get_parameter("SIM_DIRECTION", sim_direction_);
  this->declare_parameter<double>("SIM_PERIOD", 0.1);
  this->get_parameter("SIM_PERIOD", sim_period_);
  this->declare_parameter<int>("SIM_TIME_SAMPLES", 10);
  this->get_parameter("SIM_TIME_SAMPLES", sim_time_samples_);
  this->declare_parameter<double>("SLEEP_TIME_AFTER_FINISH", 0.5);
  this->get_parameter("SLEEP_TIME_AFTER_FINISH", sleep_time_after_finish_);
  this->declare_parameter<double>("SLOW_VELOCITY_TH", 0.1);
  this->get_parameter("SLOW_VELOCITY_TH", slow_velocity_th_);
  this->declare_parameter<double>("SPEED_COST_GAIN", 0.4);
  this->get_parameter("SPEED_COST_GAIN", speed_cost_gain_);
  this->declare_parameter<int>("SUBSCRIBE_COUNT_TH", 3);
  this->get_parameter("SUBSCRIBE_COUNT_TH", subscribe_count_th_);
  // - T -
  this->declare_parameter<double>("TARGET_VELOCITY", 0.55);
  this->get_parameter("TARGET_VELOCITY", target_velocity_);
  this->declare_parameter<double>("TO_GOAL_COST_GAIN", 0.8);
  this->get_parameter("TO_GOAL_COST_GAIN", to_goal_cost_gain_);
  this->declare_parameter<double>("TURN_DIRECTION_THRESHOLD", 0.1);
  this->get_parameter("TURN_DIRECTION_THRESHOLD", turn_direction_th_);
  // - U -
  this->declare_parameter<bool>("USE_FOOTPRINT", false);
  this->get_parameter("USE_FOOTPRINT", use_footprint_);
  this->declare_parameter<bool>("USE_PATH_COST", false);
  this->get_parameter("USE_PATH_COST", use_path_cost_);
  this->declare_parameter<bool>("USE_SCAN_AS_INPUT", false);
  this->get_parameter("USE_SCAN_AS_INPUT", use_scan_as_input_);
  // - V -
  this->declare_parameter<int>("VELOCITY_SAMPLES", 3);
  this->get_parameter("VELOCITY_SAMPLES", velocity_samples_);
  this->declare_parameter<double>("V_PATH_WIDTH", 0.05);
  this->get_parameter("V_PATH_WIDTH", v_path_width_);
  // - Y -
  this->declare_parameter<int>("YAWRATE_SAMPLES", 20);
  this->get_parameter("YAWRATE_SAMPLES", yawrate_samples_);

  // Ensure target_velocity does not exceed max_velocity
  target_velocity_ = std::min(target_velocity_, max_velocity_);
}

void DWAPlanner::print_params(void)
{
  // In ROS 2, `ROS_INFO_STREAM` is replaced by `RCLCPP_INFO_STREAM`.
  // You need to pass the logger of the node instance.
  // - A -
  RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE_RESOLUTION: " << angle_resolution_);
  RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE_TO_GOAL_TH: " << angle_to_goal_th_);
  // - F -
  RCLCPP_INFO_STREAM(this->get_logger(), "FOOTPRINT_PADDING: " << footprint_padding_);
  // - G -
  RCLCPP_INFO_STREAM(this->get_logger(), "GLOBAL_FRAME: " << global_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "GOAL_THRESHOLD: " << dist_to_goal_th_);
  // - H -
  RCLCPP_INFO_STREAM(this->get_logger(), "HZ: " << hz_);
  // - M -
  RCLCPP_INFO_STREAM(this->get_logger(), "MAX_ACCELERATION: " << max_acceleration_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MAX_DECELERATION: " << max_deceleration_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MAX_D_YAWRATE: " << max_d_yawrate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MAX_IN_PLACE_YAWRATE: " << max_in_place_yawrate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MAX_VELOCITY: " << max_velocity_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MAX_YAWRATE: " << max_yawrate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MIN_IN_PLACE_YAWRATE: " << min_in_place_yawrate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MIN_VELOCITY: " << min_velocity_);
  RCLCPP_INFO_STREAM(this->get_logger(), "MIN_YAWRATE: " << min_yawrate_);
  // - O -
  RCLCPP_INFO_STREAM(this->get_logger(), "OBSTACLE_COST_GAIN: " << obs_cost_gain_);
  RCLCPP_INFO_STREAM(this->get_logger(), "OBS_RANGE: " << obs_range_);
  // - P -
  RCLCPP_INFO_STREAM(this->get_logger(), "PATH_COST_GAIN: " << path_cost_gain_);
  RCLCPP_INFO_STREAM(this->get_logger(), "PREDICT_TIME: " << predict_time_);
  // - R -
  RCLCPP_INFO_STREAM(this->get_logger(), "ROBOT_FRAME: " << robot_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "ROBOT_RADIUS: " << robot_radius_);
  // - S -
  RCLCPP_INFO_STREAM(this->get_logger(), "SIM_DIRECTION: " << sim_direction_);
  RCLCPP_INFO_STREAM(this->get_logger(), "SIM_PERIOD: " << sim_period_);
  RCLCPP_INFO_STREAM(this->get_logger(), "SIM_TIME_SAMPLES: " << sim_time_samples_);
  RCLCPP_INFO_STREAM(this->get_logger(), "SLEEP_TIME_AFTER_FINISH: " << sleep_time_after_finish_);
  RCLCPP_INFO_STREAM(this->get_logger(), "SLOW_VELOCITY_TH: " << slow_velocity_th_);
  RCLCPP_INFO_STREAM(this->get_logger(), "SPEED_COST_GAIN: " << speed_cost_gain_);
  RCLCPP_INFO_STREAM(this->get_logger(), "SUBSCRIBE_COUNT_TH: " << subscribe_count_th_);
  // - T -
  RCLCPP_INFO_STREAM(this->get_logger(), "TARGET_VELOCITY: " << target_velocity_);
  RCLCPP_INFO_STREAM(this->get_logger(), "TO_GOAL_COST_GAIN: " << to_goal_cost_gain_);
  RCLCPP_INFO_STREAM(this->get_logger(), "TURN_DIRECTION_THRESHOLD: " << turn_direction_th_);
  // - U -
  RCLCPP_INFO_STREAM(this->get_logger(), "USE_FOOTPRINT: " << use_footprint_);
  RCLCPP_INFO_STREAM(this->get_logger(), "USE_PATH_COST: " << use_path_cost_);
  RCLCPP_INFO_STREAM(this->get_logger(), "USE_SCAN_AS_INPUT: " << use_scan_as_input_);
  // - V -
  RCLCPP_INFO_STREAM(this->get_logger(), "VELOCITY_SAMPLES: " << velocity_samples_);
  RCLCPP_INFO_STREAM(this->get_logger(), "V_PATH_WIDTH: " << v_path_width_);
  // - Y -
  RCLCPP_INFO_STREAM(this->get_logger(), "YAWRATE_SAMPLES: " << yawrate_samples_);
}
