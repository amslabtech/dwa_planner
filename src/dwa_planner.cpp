// Copyright 2020 amsl

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    : local_nh_("~"), odom_updated_(false), local_map_updated_(false), scan_updated_(false), has_reached_(false),
      use_speed_cost_(false), odom_not_subscribe_count_(0), local_map_not_subscribe_count_(0),
      scan_not_subscribe_count_(0)
{
  load_params();

  ROS_INFO("=== DWA Planner ===");
  print_params();

  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  candidate_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_trajectory_pub_ = local_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  predict_footprints_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("predict_footprints", 1);
  finish_flag_pub_ = local_nh_.advertise<std_msgs::Bool>("finish_flag", 1);

  dist_to_goal_th_sub_ = nh_.subscribe("/dist_to_goal_th", 1, &DWAPlanner::dist_to_goal_th_callback, this);
  edge_on_global_path_sub_ = nh_.subscribe("/path", 1, &DWAPlanner::edge_on_global_path_callback, this);
  footprint_sub_ = nh_.subscribe("/footprint", 1, &DWAPlanner::footprint_callback, this);
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DWAPlanner::goal_callback, this);
  local_map_sub_ = nh_.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
  target_velocity_sub_ = nh_.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);

  if (!use_footprint_)
    footprint_ = geometry_msgs::PolygonStamped();
  if (!use_path_cost_)
    edge_points_on_path_ = nav_msgs::Path();
  if (!use_scan_as_input_)
    scan_updated_ = true;
  else
    local_map_updated_ = true;
}

DWAPlanner::State::State(void) : x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

DWAPlanner::State::State(const double x, const double y, const double yaw, const double velocity, const double yawrate)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate)
{
}

DWAPlanner::Window::Window(void) : min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0) {}

void DWAPlanner::Window::show(void)
{
  ROS_INFO_STREAM("Window:");
  ROS_INFO_STREAM("\tVelocity:");
  ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
  ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
  ROS_INFO_STREAM("\tYawrate:");
  ROS_INFO_STREAM("\t\tmax: " << max_yawrate_);
  ROS_INFO_STREAM("\t\tmin: " << min_yawrate_);
}

DWAPlanner::Cost::Cost(void) : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0), total_cost_(0.0)
{
}

DWAPlanner::Cost::Cost(
    const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
    const float total_cost)
    : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
      total_cost_(total_cost)
{
}

void DWAPlanner::Cost::show(void)
{
  ROS_INFO_STREAM("Cost: " << total_cost_);
  ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
  ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
  ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
  ROS_INFO_STREAM("\tPath cost: " << path_cost_);
}

void DWAPlanner::Cost::calc_total_cost(void) { total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_; }

void DWAPlanner::goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  goal_msg_ = *msg;
  if (goal_msg_.value().header.frame_id != global_frame_)
  {
    try
    {
      listener_.transformPose(
          global_frame_, ros::Time(0), goal_msg_.value(), goal_msg_.value().header.frame_id, goal_msg_.value());
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void DWAPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  if (use_scan_as_input_)
    create_obs_list(*msg);
  scan_not_subscribe_count_ = 0;
  scan_updated_ = true;
}

void DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  if (!use_scan_as_input_)
    create_obs_list(*msg);
  local_map_not_subscribe_count_ = 0;
  local_map_updated_ = true;
}

void DWAPlanner::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  current_cmd_vel_ = msg->twist.twist;
  odom_not_subscribe_count_ = 0;
  odom_updated_ = true;
}

void DWAPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  target_velocity_ = std::min(msg->linear.x, max_velocity_);
  ROS_INFO_STREAM_THROTTLE(1.0, "target velocity was updated to " << target_velocity_ << " [m/s]");
}

void DWAPlanner::footprint_callback(const geometry_msgs::PolygonStampedPtr &msg)
{
  footprint_ = *msg;
  for (auto &point : footprint_.value().polygon.points)
  {
    point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
    point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
  }
}

void DWAPlanner::dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg)
{
  dist_to_goal_th_ = msg->data;
  ROS_INFO_STREAM_THROTTLE(1.0, "distance to goal threshold was updated to " << dist_to_goal_th_ << " [m]");
}

void DWAPlanner::edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg)
{
  if (!use_path_cost_)
    return;
  edge_points_on_path_ = *msg;
  try
  {
    for (auto &pose : edge_points_on_path_.value().poses)
      listener_.transformPose(robot_frame_, ros::Time(0), pose, msg->header.frame_id, pose);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

std::vector<DWAPlanner::State>
DWAPlanner::dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories)
{
  Cost min_cost(0.0, 0.0, 0.0, 0.0, 1e6);
  const Window dynamic_window = calc_dynamic_window();
  std::vector<State> best_traj;
  best_traj.resize(sim_time_samples_);
  std::vector<Cost> costs;
  const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1);
  costs.reserve(costs_size);

  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double yawrate_resolution =
      std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);

  int available_traj_count = 0;
  for (int i = 0; i < velocity_samples_; i++)
  {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++)
    {
      std::pair<std::vector<State>, bool> traj;
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;
      if (v < slow_velocity_th_)
        y = y > 0 ? std::max(y, min_yawrate_) : std::min(y, -min_yawrate_);
      traj.first = generate_trajectory(v, y);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6)
      {
        traj.second = false;
      }
      else
      {
        traj.second = true;
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_)
    {
      std::pair<std::vector<State>, bool> traj;
      traj.first = generate_trajectory(v, 0.0);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6)
      {
        traj.second = false;
      }
      else
      {
        traj.second = true;
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }
  }

  if (available_traj_count == 0)
  {
    ROS_ERROR_THROTTLE(1.0, "No available trajectory");
    best_traj = generate_trajectory(0.0, 0.0);
  }
  else
  {
    normalize_costs(costs);
    for (int i = 0; i < costs.size(); i++)
    {
      if (costs[i].obs_cost_ != 1e6)
      {
        costs[i].to_goal_cost_ *= to_goal_cost_gain_;
        costs[i].obs_cost_ *= obs_cost_gain_;
        costs[i].speed_cost_ *= speed_cost_gain_;
        costs[i].path_cost_ *= path_cost_gain_;
        costs[i].calc_total_cost();
        if (costs[i].total_cost_ < min_cost.total_cost_)
        {
          min_cost = costs[i];
          best_traj = trajectories[i].first;
        }
      }
    }
  }

  ROS_INFO("===");
  ROS_INFO_STREAM("(v, y) = (" << best_traj.front().velocity_ << ", " << best_traj.front().yawrate_ << ")");
  min_cost.show();
  ROS_INFO_STREAM("num of trajectories available: " << available_traj_count << " of " << trajectories.size());
  ROS_INFO(" ");

  return best_traj;
}

void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> &costs)
{
  Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

  for (const auto &cost : costs)
  {
    if (cost.obs_cost_ != 1e6)
    {
      min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
      max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
      min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
      max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
      if (use_speed_cost_)
      {
        min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
        max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
      }
      if (use_path_cost_)
      {
        min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
        max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
      }
    }
  }

  for (auto &cost : costs)
  {
    if (cost.obs_cost_ != 1e6)
    {
      cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_ + DBL_EPSILON);
      cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
                           (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + DBL_EPSILON);
      if (use_speed_cost_)
        cost.speed_cost_ =
            (cost.speed_cost_ - min_cost.speed_cost_) / (max_cost.speed_cost_ - min_cost.speed_cost_ + DBL_EPSILON);
      if (use_path_cost_)
        cost.path_cost_ =
            (cost.path_cost_ - min_cost.path_cost_) / (max_cost.path_cost_ - min_cost.path_cost_ + DBL_EPSILON);
    }
  }
}

void DWAPlanner::process(void)
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;
    if (can_move())
      cmd_vel = calc_cmd_vel();
    velocity_pub_.publish(cmd_vel);
    finish_flag_pub_.publish(has_finished_);
    if (has_finished_.data)
      ros::Duration(sleep_time_after_finish_).sleep();

    if (use_scan_as_input_)
      scan_updated_ = false;
    else
      local_map_updated_ = false;
    odom_updated_ = false;
    has_finished_.data = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool DWAPlanner::can_move(void)
{
  if (!footprint_.has_value())
    ROS_WARN_THROTTLE(1.0, "Robot Footprint has not been updated");
  if (!goal_msg_.has_value())
    ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
  if (!edge_points_on_path_.has_value())
    ROS_WARN_THROTTLE(1.0, "Edge on global path has not been updated");
  if (subscribe_count_th_ < odom_not_subscribe_count_)
    ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
  if (subscribe_count_th_ < local_map_not_subscribe_count_)
    ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
  if (subscribe_count_th_ < scan_not_subscribe_count_)
    ROS_WARN_THROTTLE(1.0, "Scan has not been updated");

  if (!odom_updated_)
    odom_not_subscribe_count_++;
  if (!local_map_updated_)
    local_map_not_subscribe_count_++;
  if (!scan_updated_)
    scan_not_subscribe_count_++;

  if (footprint_.has_value() && goal_msg_.has_value() && edge_points_on_path_.has_value() &&
      odom_not_subscribe_count_ <= subscribe_count_th_ && local_map_not_subscribe_count_ <= subscribe_count_th_ &&
      scan_not_subscribe_count_ <= subscribe_count_th_)
    return true;
  else
    return false;
}

geometry_msgs::Twist DWAPlanner::calc_cmd_vel(void)
{
  geometry_msgs::Twist cmd_vel;
  std::pair<std::vector<State>, bool> best_traj;
  std::vector<std::pair<std::vector<State>, bool>> trajectories;
  const size_t trajectories_size = velocity_samples_ * (yawrate_samples_ + 1);
  trajectories.reserve(trajectories_size);

  geometry_msgs::PoseStamped goal_;
  try
  {
    listener_.transformPose(robot_frame_, ros::Time(0), goal_msg_.value(), goal_msg_.value().header.frame_id, goal_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation));

  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (M_PI / 4.0 < fabs(angle_to_goal))
    use_speed_cost_ = true;

  if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_)
  {
    if (can_adjust_robot_direction(goal))
    {
      cmd_vel.angular.z = angle_to_goal > 0 ? std::min(angle_to_goal, max_in_place_yawrate_)
                                            : std::max(angle_to_goal, -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z > 0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_)
                                                : std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
      best_traj.first = generate_trajectory(cmd_vel.angular.z, goal);
      trajectories.push_back(best_traj);
    }
    else
    {
      best_traj.first = dwa_planning(goal, trajectories);
      cmd_vel.linear.x = best_traj.first.front().velocity_;
      cmd_vel.angular.z = best_traj.first.front().yawrate_;
    }
  }
  else
  {
    has_reached_ = true;
    if (turn_direction_th_ < fabs(goal[2]))
    {
      cmd_vel.angular.z =
          goal[2] > 0 ? std::min(goal[2], max_in_place_yawrate_) : std::max(goal[2], -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z > 0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_)
                                                : std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
    }
    else
    {
      has_finished_.data = true;
      has_reached_ = false;
    }
    best_traj.first = generate_trajectory(cmd_vel.linear.x, cmd_vel.angular.z);
    trajectories.push_back(best_traj);
  }

  for (int i = 0; i < trajectories_size; i++)
    trajectories.push_back(trajectories.front());

  visualize_trajectory(best_traj.first, selected_trajectory_pub_);
  visualize_trajectories(trajectories, candidate_trajectories_pub_);
  visualize_footprints(best_traj.first, predict_footprints_pub_);

  use_speed_cost_ = false;

  return cmd_vel;
}

bool DWAPlanner::can_adjust_robot_direction(const Eigen::Vector3d &goal)
{
  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (fabs(angle_to_goal) < angle_to_goal_th_)
    return false;

  const double yawrate = std::min(std::max(angle_to_goal, -max_in_place_yawrate_), max_in_place_yawrate_);
  std::vector<State> traj = generate_trajectory(yawrate, goal);

  if (!check_collision(traj))
    return true;
  else
    return false;
}

bool DWAPlanner::check_collision(const std::vector<State> &traj)
{
  if (!use_footprint_)
    return false;

  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list_.poses)
    {
      const geometry_msgs::PolygonStamped footprint = move_footprint(state);
      if (is_inside_of_robot(obs.position, footprint, state))
        return true;
    }
  }

  return false;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
  Window window;
  window.min_velocity_ = std::max((current_cmd_vel_.linear.x - max_deceleration_ * sim_period_), min_velocity_);
  window.max_velocity_ = std::min((current_cmd_vel_.linear.x + max_acceleration_ * sim_period_), target_velocity_);
  window.min_yawrate_ = std::max((current_cmd_vel_.angular.z - max_d_yawrate_ * sim_period_), -max_yawrate_);
  window.max_yawrate_ = std::min((current_cmd_vel_.angular.z + max_d_yawrate_ * sim_period_), max_yawrate_);
  return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_obs_cost(const std::vector<State> &traj)
{
  float min_dist = obs_range_;
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list_.poses)
    {
      float dist;
      if (use_footprint_)
        dist = calc_dist_from_robot(obs.position, state);
      else
        dist = hypot((state.x_ - obs.position.x), (state.y_ - obs.position.y)) - robot_radius_ - footprint_padding_;

      if (dist < DBL_EPSILON)
        return 1e6;
      min_dist = std::min(min_dist, dist);
    }
  }
  return obs_range_ - min_dist;
}

float DWAPlanner::calc_speed_cost(const std::vector<State> &traj)
{
  if (!use_speed_cost_)
    return 0.0;
  const Window dynamic_window = calc_dynamic_window();
  return dynamic_window.max_velocity_ - traj.front().velocity_;
}

float DWAPlanner::calc_path_cost(const std::vector<State> &traj)
{
  if (!use_path_cost_)
    return 0.0;
  else
    return calc_dist_to_path(traj.back());
}

float DWAPlanner::calc_dist_to_path(const State state)
{
  geometry_msgs::Point edge_point1 = edge_points_on_path_.value().poses.front().pose.position;
  geometry_msgs::Point edge_point2 = edge_points_on_path_.value().poses.back().pose.position;
  const float a = edge_point2.y - edge_point1.y;
  const float b = -(edge_point2.x - edge_point1.x);
  const float c = -a * edge_point1.x - b * edge_point1.y;

  return fabs(a * state.x_ + b * state.y_ + c) / (hypot(a, b) + DBL_EPSILON);
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double velocity, const double yawrate)
{
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++)
  {
    motion(state, velocity, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double yawrate, const Eigen::Vector3d &goal)
{
  const double target_direction = atan2(goal.y(), goal.x()) > 0 ? sim_direction_ : -sim_direction_;
  const double predict_time = target_direction / (yawrate + DBL_EPSILON);
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++)
  {
    motion(state, 0.0, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

DWAPlanner::Cost DWAPlanner::evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal)
{
  Cost cost;
  cost.to_goal_cost_ = calc_to_goal_cost(trajectory, goal);
  cost.obs_cost_ = calc_obs_cost(trajectory);
  cost.speed_cost_ = calc_speed_cost(trajectory);
  cost.path_cost_ = calc_path_cost(trajectory);
  cost.calc_total_cost();
  return cost;
}

geometry_msgs::Point DWAPlanner::calc_intersection(
    const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint)
{
  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    const Eigen::Vector3d vector_A(obstacle.x, obstacle.y, 0.0);
    const Eigen::Vector3d vector_B(state.x_, state.y_, 0.0);
    const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y, 0.0);
    Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
    if (i != footprint.polygon.points.size() - 1)
      vector_D << footprint.polygon.points[i + 1].x, footprint.polygon.points[i + 1].y, 0.0;
    else
      vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;

    const double deno = (vector_B - vector_A).cross(vector_D - vector_C).z();
    const double s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
    const double t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

    geometry_msgs::Point point;
    point.x = vector_A.x() + s * (vector_B - vector_A).x();
    point.y = vector_A.y() + s * (vector_B - vector_A).y();

    // cross
    if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t))
      return point;
  }

  geometry_msgs::Point point;
  point.x = 1e6;
  point.y = 1e6;
  return point;
}

float DWAPlanner::calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state)
{
  const geometry_msgs::PolygonStamped footprint = move_footprint(state);
  if (is_inside_of_robot(obstacle, footprint, state))
  {
    return 0.0;
  }
  else
  {
    geometry_msgs::Point intersection = calc_intersection(obstacle, state, footprint);
    return hypot((obstacle.x - intersection.x), (obstacle.y - intersection.y));
  }
}

geometry_msgs::PolygonStamped DWAPlanner::move_footprint(const State &target_pose)
{
  geometry_msgs::PolygonStamped footprint;
  if (use_footprint_)
  {
    footprint = footprint_.value();
  }
  else
  {
    const int plot_num = 20;
    for (int i = 0; i < plot_num; i++)
    {
      geometry_msgs::Point32 point;
      point.x = (robot_radius_ + footprint_padding_) * cos(2 * M_PI * i / plot_num);
      point.y = robot_radius_ * sin(2 * M_PI * i / plot_num);
      footprint.polygon.points.push_back(point);
    }
  }

  footprint.header.stamp = ros::Time::now();

  for (auto &point : footprint.polygon.points)
  {
    Eigen::VectorXf point_in(2);
    point_in << point.x, point.y;
    Eigen::Matrix2f rot;
    rot = Eigen::Rotation2Df(target_pose.yaw_);
    const Eigen::VectorXf point_out = rot * point_in;

    point.x = point_out.x() + target_pose.x_;
    point.y = point_out.y() + target_pose.y_;
  }

  return footprint;
}

bool DWAPlanner::is_inside_of_robot(
    const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint, const State &state)
{
  geometry_msgs::Point32 state_point;
  state_point.x = state.x_;
  state_point.y = state.y_;

  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    geometry_msgs::Polygon triangle;
    triangle.points.push_back(state_point);
    triangle.points.push_back(footprint.polygon.points[i]);

    if (i != footprint.polygon.points.size() - 1)
      triangle.points.push_back(footprint.polygon.points[i + 1]);
    else
      triangle.points.push_back(footprint.polygon.points[0]);

    if (is_inside_of_triangle(obstacle, triangle))
      return true;
  }

  return false;
}

bool DWAPlanner::is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle)
{
  if (triangle.points.size() != 3)
  {
    ROS_ERROR("Not triangle");
    exit(1);
  }

  const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
  const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
  const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
  const Eigen::Vector3d vector_P(target_point.x, target_point.y, 0.0);

  const Eigen::Vector3d vector_AB = vector_B - vector_A;
  const Eigen::Vector3d vector_BP = vector_P - vector_B;
  const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

  const Eigen::Vector3d vector_BC = vector_C - vector_B;
  const Eigen::Vector3d vector_CP = vector_P - vector_C;
  const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

  const Eigen::Vector3d vector_CA = vector_A - vector_C;
  const Eigen::Vector3d vector_AP = vector_P - vector_A;
  const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

  if ((0 < cross1.z() && 0 < cross2.z() && 0 < cross3.z()) || (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0))
    return true;
  else
    return false;
}

void DWAPlanner::motion(State &state, const double velocity, const double yawrate)
{
  const double sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_);
  state.yaw_ += yawrate * sim_time_step;
  state.x_ += velocity * std::cos(state.yaw_) * sim_time_step;
  state.y_ += velocity * std::sin(state.yaw_) * sim_time_step;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
}

void DWAPlanner::create_obs_list(const sensor_msgs::LaserScan &scan)
{
  obs_list_.poses.clear();
  float angle = scan.angle_min;
  const int angle_index_step = static_cast<int>(angle_resolution_ / scan.angle_increment);
  for (int i = 0; i < scan.ranges.size(); i++)
  {
    const float r = scan.ranges[i];
    if (r < scan.range_min || scan.range_max < r || i % angle_index_step != 0)
    {
      angle += scan.angle_increment;
      continue;
    }
    geometry_msgs::Pose pose;
    pose.position.x = r * cos(angle);
    pose.position.y = r * sin(angle);
    obs_list_.poses.push_back(pose);
    angle += scan.angle_increment;
  }
}

void DWAPlanner::create_obs_list(const nav_msgs::OccupancyGrid &map)
{
  obs_list_.poses.clear();
  const double max_search_dist = hypot(map.info.origin.position.x, map.info.origin.position.y);
  for (float angle = -M_PI; angle <= M_PI; angle += angle_resolution_)
  {
    for (float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution)
    {
      geometry_msgs::Pose pose;
      pose.position.x = dist * cos(angle);
      pose.position.y = dist * sin(angle);
      const int index_x = floor((pose.position.x - map.info.origin.position.x) / map.info.resolution);
      const int index_y = floor((pose.position.y - map.info.origin.position.y) / map.info.resolution);

      if ((0 <= index_x && index_x < map.info.width) && (0 <= index_y && index_y < map.info.height))
      {
        if (map.data[index_x + index_y * map.info.width] == 100)
        {
          obs_list_.poses.push_back(pose);
          break;
        }
      }
    }
  }
}

visualization_msgs::Marker DWAPlanner::create_marker_msg(
    const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
    const geometry_msgs::PolygonStamped &footprint)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = robot_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = scale;
  marker.color = color;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration(1 / hz_);

  geometry_msgs::Point p;
  if (footprint.polygon.points.empty())
  {
    for (const auto &point : trajectory)
    {
      p.x = point.x_;
      p.y = point.y_;
      marker.points.push_back(p);
    }
  }
  else
  {
    for (const auto &point : footprint.polygon.points)
    {
      p.x = point.x;
      p.y = point.y;
      marker.points.push_back(p);
    }
    p.x = footprint.polygon.points.front().x;
    p.y = footprint.polygon.points.front().y;
    marker.points.push_back(p);
  }

  return marker;
}

void DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  visualization_msgs::Marker v_trajectory = create_marker_msg(0, v_path_width_, color, trajectory);
  pub.publish(v_trajectory);
}

void DWAPlanner::visualize_trajectories(
    const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  for (int i = 0; i < trajectories.size(); i++)
  {
    std_msgs::ColorRGBA color;
    if (trajectories[i].second)
    {
      color.g = 1.0;
    }
    else
    {
      color.r = 0.5;
      color.b = 0.5;
    }
    visualization_msgs::Marker v_trajectory = create_marker_msg(i, v_path_width_ * 0.4, color, trajectories[i].first);
    v_trajectories.markers.push_back(v_trajectory);
  }
  pub.publish(v_trajectories);
}

void DWAPlanner::visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub)
{
  std_msgs::ColorRGBA color;
  color.b = 1.0;
  visualization_msgs::MarkerArray v_footprints;
  for (int i = 0; i < trajectory.size(); i++)
  {
    const geometry_msgs::PolygonStamped footprint = move_footprint(trajectory[i]);
    visualization_msgs::Marker v_footprint = create_marker_msg(i, v_path_width_ * 0.2, color, trajectory, footprint);
    v_footprints.markers.push_back(v_footprint);
  }
  pub.publish(v_footprints);
}
