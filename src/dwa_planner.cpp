// Copyright 2020 amsl

#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <limits>


#include "dwa_planner/dwa_planner.hpp"

// --- State Class Implementation ---
DWAPlanner::State::State(void)
: x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

DWAPlanner::State::State(
  const double x, const double y, const double yaw, const double velocity,
  const double yawrate)
: x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate)
{
}

// --- Window Class Implementation ---
DWAPlanner::Window::Window(void)
: min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0), logger_(rclcpp::get_logger(
      "dwa_planner_window")) {}

void DWAPlanner::Window::show(void)
{
  RCLCPP_INFO(logger_, "Window:");
  RCLCPP_INFO(logger_, "\tVelocity:");
  RCLCPP_INFO(logger_, "\t\tmax: %f", max_velocity_);
  RCLCPP_INFO(logger_, "\t\tmin: %f", min_velocity_);
  RCLCPP_INFO(logger_, "\tYawrate:");
  RCLCPP_INFO(logger_, "\t\tmax: %f", max_yawrate_);
  RCLCPP_INFO(logger_, "\t\tmin: %f", min_yawrate_);
}

// --- Cost Class Implementation ---
DWAPlanner::Cost::Cost(void)
: obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0), total_cost_(0.0), logger_(rclcpp::get_logger(
      "dwa_planner_window"))
{
}

DWAPlanner::Cost::Cost(
  const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
  const float total_cost)
: obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
  total_cost_(total_cost), logger_(rclcpp::get_logger("dwa_planner_window"))
{
}

void DWAPlanner::Cost::show(void)
{
  RCLCPP_INFO(logger_, "Cost: %f", total_cost_);
  RCLCPP_INFO(logger_, "\tObs cost: %f", obs_cost_);
  RCLCPP_INFO(logger_, "\tGoal cost: %f", to_goal_cost_);
  RCLCPP_INFO(logger_, "\tSpeed cost: %f", speed_cost_);
  RCLCPP_INFO(logger_, "\tPath cost: %f", path_cost_);
}

void DWAPlanner::Cost::calc_total_cost(void)
{
  total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_;
}

// --- DWAPlanner Class Implementation ---

DWAPlanner::DWAPlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("dwa_planner_node", options),
  odom_updated_(false), local_map_updated_(false), scan_updated_(false), has_reached_(false),
  use_speed_cost_(false), odom_not_subscribe_count_(0), local_map_not_subscribe_count_(0),
  scan_not_subscribe_count_(0)
{

  RCLCPP_INFO(this->get_logger(), "=== DWA Planner ===");


  load_params();

  print_params();


  velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  candidate_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "candidate_trajectories", 1);
  selected_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "selected_trajectory", 1);
  predict_footprints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "predict_footprints", 1);
  finish_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("finish_flag", 1);


  dist_to_goal_th_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/dist_to_goal_th", 1,
    std::bind(&DWAPlanner::dist_to_goal_th_callback, this, std::placeholders::_1));
  edge_on_global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 1, std::bind(&DWAPlanner::edge_on_global_path_callback, this, std::placeholders::_1));
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/footprint", 1, std::bind(&DWAPlanner::footprint_callback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/move_base_simple/goal", 1,
    std::bind(&DWAPlanner::goal_callback, this, std::placeholders::_1));
  local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_map", 1, std::bind(&DWAPlanner::local_map_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1, std::bind(&DWAPlanner::odom_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 1, std::bind(&DWAPlanner::scan_callback, this, std::placeholders::_1));
  target_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/target_velocity", 1,
    std::bind(&DWAPlanner::target_velocity_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!use_footprint_) {
    footprint_ = geometry_msgs::msg::PolygonStamped();
  }
  if (!use_path_cost_) {
    edge_points_on_path_ = nav_msgs::msg::Path();
  }
  if (!use_scan_as_input_) {
    scan_updated_ = true;
  } else {
    local_map_updated_ = true;
  }
}

// --- Callback Functions ---
void DWAPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  goal_msg_ = *msg;
  if (goal_msg_->header.frame_id != global_frame_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        global_frame_, goal_msg_->header.frame_id,
        tf2::TimePointZero);


      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(*goal_msg_, transformed_pose, transform_stamped);
      goal_msg_ = transformed_pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform goal: %s", ex.what());
      goal_msg_.reset();
    }
  }
}

void DWAPlanner::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (use_scan_as_input_) {
    create_obs_list(*msg);
  }
  scan_not_subscribe_count_ = 0;
  scan_updated_ = true;
}

void DWAPlanner::local_map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (!use_scan_as_input_) {
    create_obs_list(*msg);
  }
  local_map_not_subscribe_count_ = 0;
  local_map_updated_ = true;
}

void DWAPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_cmd_vel_ = msg->twist.twist;
  odom_not_subscribe_count_ = 0;
  odom_updated_ = true;
}

void DWAPlanner::target_velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  target_velocity_ = std::min(msg->linear.x, max_velocity_);
  RCLCPP_INFO_STREAM_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "target velocity was updated to " << target_velocity_ << " [m/s]");
}

void DWAPlanner::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  footprint_ = *msg;
  for (auto & point : footprint_->polygon.points) {
    point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
    point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
  }
}

void DWAPlanner::dist_to_goal_th_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  dist_to_goal_th_ = msg->data;
  RCLCPP_INFO_STREAM_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "distance to goal threshold was updated to " << dist_to_goal_th_ << " [m]");
}

void DWAPlanner::edge_on_global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (!use_path_cost_) {
    return;
  }
  edge_points_on_path_ = *msg;

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      robot_frame_, msg->header.frame_id,
      tf2::TimePointZero);

    for (auto & pose : edge_points_on_path_->poses) {
      geometry_msgs::msg::PoseStamped original_pose_stamped;
      original_pose_stamped.header = msg->header;
      original_pose_stamped.pose = pose.pose;

      geometry_msgs::msg::PoseStamped transformed_pose_stamped;
      tf2::doTransform(original_pose_stamped, transformed_pose_stamped, transform_stamped);
      pose.pose = transformed_pose_stamped.pose;
    }

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform path edge points: %s", ex.what());
    edge_points_on_path_.reset();
  }
}
// --- DWA Planning Core Functions ---
std::vector<DWAPlanner::State>
DWAPlanner::dwa_planning(
  const Eigen::Vector3d & goal, std::vector<std::pair<std::vector<State>,
  bool>> & trajectories)
{
  Cost min_cost(0.0, 0.0, 0.0, 0.0, 1e6);
  const Window dynamic_window = calc_dynamic_window();
  std::vector<State> best_traj;
  best_traj.resize(sim_time_samples_);
  std::vector<Cost> costs;
  const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1);
  costs.reserve(costs_size);

  const double velocity_resolution =
    std::max(
    (dynamic_window.max_velocity_ - dynamic_window.min_velocity_) /
    (static_cast<double>(velocity_samples_ - 1) + std::numeric_limits<double>::epsilon()),
    DBL_EPSILON);
  const double yawrate_resolution =
    std::max(
    (dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) /
    (static_cast<double>(yawrate_samples_ - 1) + std::numeric_limits<double>::epsilon()),
    DBL_EPSILON);

  int available_traj_count = 0;
  for (int i = 0; i < velocity_samples_; i++) {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++) {
      std::pair<std::vector<State>, bool> traj;
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;
      if (v < slow_velocity_th_) {
        y = y > 0 ? std::max(y, min_yawrate_) : std::min(y, -min_yawrate_);
      }
      traj.first = generate_trajectory(v, y);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6) {
        traj.second = false;
      } else {
        traj.second = true;
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_) {
      std::pair<std::vector<State>, bool> traj;
      traj.first = generate_trajectory(v, 0.0);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6) {
        traj.second = false;
      } else {
        traj.second = true;
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }
  }

  if (available_traj_count == 0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No available trajectory");
    best_traj = generate_trajectory(0.0, 0.0);
  } else {
    normalize_costs(costs);
    for (int i = 0; i < costs.size(); i++) {
      if (costs[i].obs_cost_ != 1e6) {
        costs[i].to_goal_cost_ *= to_goal_cost_gain_;
        costs[i].obs_cost_ *= obs_cost_gain_;
        costs[i].speed_cost_ *= speed_cost_gain_;
        costs[i].path_cost_ *= path_cost_gain_;
        costs[i].calc_total_cost();
        if (costs[i].total_cost_ < min_cost.total_cost_) {
          min_cost = costs[i];
          best_traj = trajectories[i].first;
        }
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "===");
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "(v, y) = (" << best_traj.front().velocity_ << ", " << best_traj.front().yawrate_ << ")");
  min_cost.show();
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "num of trajectories available: " << available_traj_count << " of " << trajectories.size());
  RCLCPP_INFO(this->get_logger(), " ");

  return best_traj;
}

void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> & costs)
{
  Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

  for (const auto & cost : costs) {
    if (cost.obs_cost_ != 1e6) {
      min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
      max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
      min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
      max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
      if (use_speed_cost_) {
        min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
        max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
      }
      if (use_path_cost_) {
        min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
        max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
      }
    }
  }

  for (auto & cost : costs) {
    if (cost.obs_cost_ != 1e6) {
      cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) /
        (max_cost.obs_cost_ - min_cost.obs_cost_ + std::numeric_limits<double>::epsilon());
      cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
        (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + std::numeric_limits<double>::epsilon());
      if (use_speed_cost_) {
        cost.speed_cost_ =
          (cost.speed_cost_ - min_cost.speed_cost_) /
          (max_cost.speed_cost_ - min_cost.speed_cost_ + std::numeric_limits<double>::epsilon());
      }
      if (use_path_cost_) {
        cost.path_cost_ =
          (cost.path_cost_ - min_cost.path_cost_) /
          (max_cost.path_cost_ - min_cost.path_cost_ + std::numeric_limits<double>::epsilon());
      }
    }
  }
}

void DWAPlanner::process(void)
{
  rclcpp::Rate loop_rate(hz_);
  while (rclcpp::ok()) {
    geometry_msgs::msg::Twist cmd_vel;
    if (can_move()) {
      cmd_vel = calc_cmd_vel();
    }
    velocity_pub_->publish(cmd_vel);
    finish_flag_pub_->publish(has_finished_);
    if (has_finished_.data) {
      loop_rate.sleep();
    }
    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(sleep_time_after_finish_)));


    if (use_scan_as_input_) {
      scan_updated_ = false;
    } else {
      local_map_updated_ = false;
    }
    odom_updated_ = false;
    has_finished_.data = false;

    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

bool DWAPlanner::can_move(void)
{

  if (!footprint_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Robot Footprint has not been updated");
  }
  if (!goal_msg_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Local goal has not been updated");
  }
  if (!edge_points_on_path_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Edge on global path has not been updated");
  }
  if (subscribe_count_th_ < odom_not_subscribe_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Odom has not been updated");
  }
  if (subscribe_count_th_ < local_map_not_subscribe_count_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Local map has not been updated");
  }
  if (subscribe_count_th_ < scan_not_subscribe_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Scan has not been updated");
  }

  if (!odom_updated_) {
    odom_not_subscribe_count_++;
  }
  if (!local_map_updated_) {
    local_map_not_subscribe_count_++;
  }
  if (!scan_updated_) {
    scan_not_subscribe_count_++;
  }


  if (footprint_.has_value() && goal_msg_.has_value() && edge_points_on_path_.has_value() &&
    odom_not_subscribe_count_ <= subscribe_count_th_ &&
    local_map_not_subscribe_count_ <= subscribe_count_th_ &&
    scan_not_subscribe_count_ <= subscribe_count_th_)
  {
    return true;
  } else {
    return false;
  }
}

geometry_msgs::msg::Twist DWAPlanner::calc_cmd_vel(void)
{
  geometry_msgs::msg::Twist cmd_vel;
  std::pair<std::vector<State>, bool> best_traj;
  std::vector<std::pair<std::vector<State>, bool>> trajectories;
  const size_t trajectories_size = velocity_samples_ * (yawrate_samples_ + 1);
  trajectories.reserve(trajectories_size);

  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {

    transform_stamped = tf_buffer_->lookupTransform(
      robot_frame_, goal_msg_->header.frame_id,
      tf2::TimePointZero);

    tf2::doTransform(*goal_msg_, goal_, transform_stamped);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform goal for calc_cmd_vel: %s", ex.what());
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  tf2::Quaternion tf2_quat;
  tf2::fromMsg(goal_.pose.orientation, tf2_quat);

  tf2::Matrix3x3 m(tf2_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, yaw);


  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (M_PI / 4.0 < fabs(angle_to_goal)) {
    use_speed_cost_ = true;
  }

  if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_) {
    if (can_adjust_robot_direction(goal)) {
      cmd_vel.angular.z = angle_to_goal > 0 ? std::min(angle_to_goal, max_in_place_yawrate_) :
        std::max(angle_to_goal, -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z >
        0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_) :
        std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
      best_traj.first = generate_trajectory(cmd_vel.angular.z, goal);
      trajectories.push_back(best_traj);
    } else {
      best_traj.first = dwa_planning(goal, trajectories);
      cmd_vel.linear.x = best_traj.first.front().velocity_;
      cmd_vel.angular.z = best_traj.first.front().yawrate_;
    }
  } else {
    has_reached_ = true;
    if (turn_direction_th_ < fabs(goal[2])) {
      cmd_vel.angular.z =
        goal[2] > 0 ? std::min(goal[2], max_in_place_yawrate_) : std::max(
        goal[2],
        -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z >
        0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_) :
        std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
    } else {
      has_finished_.data = true;
      has_reached_ = false;
    }
    best_traj.first = generate_trajectory(cmd_vel.linear.x, cmd_vel.angular.z);
    trajectories.push_back(best_traj);


  }


  for (int i = 0; i < trajectories_size; i++) {
    trajectories.push_back(trajectories.front());
  }


  visualize_trajectory(best_traj.first, selected_trajectory_pub_);
  visualize_trajectories(trajectories, candidate_trajectories_pub_);
  visualize_footprints(best_traj.first, predict_footprints_pub_);

  use_speed_cost_ = false;

  return cmd_vel;
}

bool DWAPlanner::can_adjust_robot_direction(const Eigen::Vector3d & goal)
{
  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (fabs(angle_to_goal) < angle_to_goal_th_) {
    return false;
  }

  const double yawrate = std::min(
    std::max(
      angle_to_goal,
      -max_in_place_yawrate_), max_in_place_yawrate_);
  std::vector<State> traj = generate_trajectory(0.0, yawrate);

  if (!check_collision(traj)) {
    return true;
  } else {
    return false;
  }
}

bool DWAPlanner::check_collision(const std::vector<State> & traj)
{
  if (!use_footprint_) {
    return false;
  }

  if (!obs_list_.poses.empty()) {
    for (const auto & state : traj) {
      for (const auto & obs : obs_list_.poses) {
        const geometry_msgs::msg::PolygonStamped footprint = move_footprint(state);
        if (is_inside_of_robot(obs.position, footprint, state)) {
          return true;
        }
      }
    }
  }

  return false;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
  Window window;
  window.min_velocity_ = std::max(
    (current_cmd_vel_.linear.x - max_deceleration_ * sim_period_),
    min_velocity_);
  window.max_velocity_ = std::min(
    (current_cmd_vel_.linear.x + max_acceleration_ * sim_period_),
    target_velocity_);
  window.min_yawrate_ = std::max(
    (current_cmd_vel_.angular.z - max_d_yawrate_ * sim_period_),
    -max_yawrate_);
  window.max_yawrate_ = std::min(
    (current_cmd_vel_.angular.z + max_d_yawrate_ * sim_period_),
    max_yawrate_);
  return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State> & traj, const Eigen::Vector3d & goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_obs_cost(const std::vector<State> & traj)
{
  float min_dist = obs_range_;
  for (const auto & state : traj) {
    for (const auto & obs : obs_list_.poses) {
      float dist;
      if (use_footprint_) {
        dist = calc_dist_from_robot(obs.position, state);
      } else {
        dist =
          hypot(
          (state.x_ - obs.position.x),
          (state.y_ - obs.position.y)) - robot_radius_ - footprint_padding_;
      }

      if (dist < std::numeric_limits<double>::epsilon()) {
        return 1e6;
      }
      min_dist = std::min(min_dist, dist);
    }
  }
  return obs_range_ - min_dist;
}

float DWAPlanner::calc_speed_cost(const std::vector<State> & traj)
{
  if (!use_speed_cost_) {
    return 0.0;
  }
  const Window dynamic_window = calc_dynamic_window();
  return dynamic_window.max_velocity_ - traj.front().velocity_;
}

float DWAPlanner::calc_path_cost(const std::vector<State> & traj)
{
  if (!use_path_cost_) {
    return 0.0;
  } else if (edge_points_on_path_.has_value()) {
    return calc_dist_to_path(traj.back());
  } else {
    return 0.0;
  }
}

float DWAPlanner::calc_dist_to_path(const State state)
{
  if (!edge_points_on_path_.has_value() || edge_points_on_path_->poses.empty()) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Path for cost calculation is not available.");
    return 0.0;
  }
  geometry_msgs::msg::Point edge_point1 = edge_points_on_path_->poses.front().pose.position;
  geometry_msgs::msg::Point edge_point2 = edge_points_on_path_->poses.back().pose.position;
  const float a = edge_point2.y - edge_point1.y;
  const float b = -(edge_point2.x - edge_point1.x);
  const float c = -a * edge_point1.x - b * edge_point1.y;

  return fabs(a * state.x_ + b * state.y_ + c) /
         (hypot(a, b) + std::numeric_limits<double>::epsilon());
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(
  const double velocity,
  const double yawrate)
{
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++) {
    motion(state, velocity, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(
  const double yawrate,
  const Eigen::Vector3d & goal)
{
  const double target_direction = atan2(goal.y(), goal.x()) > 0 ? sim_direction_ : -sim_direction_;

  const double predict_time_for_turn = target_direction /
    (yawrate + std::numeric_limits<double>::epsilon());

  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++) {
    motion(state, 0.0, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

DWAPlanner::Cost DWAPlanner::evaluate_trajectory(
  const std::vector<State> & trajectory,
  const Eigen::Vector3d & goal)
{
  Cost cost;
  cost.to_goal_cost_ = calc_to_goal_cost(trajectory, goal);
  cost.obs_cost_ = calc_obs_cost(trajectory);
  cost.speed_cost_ = calc_speed_cost(trajectory);
  cost.path_cost_ = calc_path_cost(trajectory);
  cost.calc_total_cost();
  return cost;
}

geometry_msgs::msg::Point DWAPlanner::calc_intersection(
  const geometry_msgs::msg::Point & obstacle, const State & state,
  geometry_msgs::msg::PolygonStamped footprint)
{
  for (size_t i = 0; i < footprint.polygon.points.size(); ++i) {
    const Eigen::Vector3d vector_A(obstacle.x, obstacle.y, 0.0);
    const Eigen::Vector3d vector_B(state.x_, state.y_, 0.0);
    const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y,
      0.0);
    Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
    if (i != footprint.polygon.points.size() - 1) {
      vector_D << footprint.polygon.points[i + 1].x, footprint.polygon.points[i + 1].y, 0.0;
    } else {
      vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;
    }

    const double deno = (vector_B - vector_A).cross(vector_D - vector_C).z();

    if (std::abs(deno) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    const double s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
    const double t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

    geometry_msgs::msg::Point point;
    point.x = vector_A.x() + s * (vector_B - vector_A).x();
    point.y = vector_A.y() + s * (vector_B - vector_A).y();


    if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t)) {
      return point;
    }
  }

  geometry_msgs::msg::Point point;
  point.x = 1e6;
  point.y = 1e6;
  return point;
}

float DWAPlanner::calc_dist_from_robot(
  const geometry_msgs::msg::Point & obstacle,
  const State & state)
{
  const geometry_msgs::msg::PolygonStamped footprint = move_footprint(state);
  if (is_inside_of_robot(obstacle, footprint, state)) {
    return 0.0;
  } else {
    geometry_msgs::msg::Point intersection = calc_intersection(obstacle, state, footprint);
    return hypot((obstacle.x - intersection.x), (obstacle.y - intersection.y));
  }
}

geometry_msgs::msg::PolygonStamped DWAPlanner::move_footprint(const State & target_pose)
{
  geometry_msgs::msg::PolygonStamped footprint_moved;
  if (use_footprint_ && footprint_.has_value()) {
    footprint_moved = footprint_.value();
  } else {
    const int plot_num = 20;
    for (int i = 0; i < plot_num; i++) {
      geometry_msgs::msg::Point32 point;
      point.x = (robot_radius_ + footprint_padding_) * cos(2 * M_PI * i / plot_num);
      point.y = robot_radius_ * sin(2 * M_PI * i / plot_num);
      footprint_moved.polygon.points.push_back(point);
    }
  }

  footprint_moved.header.stamp = this->get_clock()->now();

  for (auto & point : footprint_moved.polygon.points) {
    Eigen::Vector2f point_in(point.x, point.y);
    Eigen::Rotation2Df rot(target_pose.yaw_);
    const Eigen::Vector2f point_out = rot * point_in;

    point.x = point_out.x() + target_pose.x_;
    point.y = point_out.y() + target_pose.y_;
  }

  return footprint_moved;
}

bool DWAPlanner::is_inside_of_robot(
  const geometry_msgs::msg::Point & obstacle, const geometry_msgs::msg::PolygonStamped & footprint,
  const State & state)
{
  geometry_msgs::msg::Point32 state_point;
  state_point.x = state.x_;
  state_point.y = state.y_;

  for (size_t i = 0; i < footprint.polygon.points.size(); ++i) {
    geometry_msgs::msg::Polygon triangle;
    triangle.points.push_back(state_point);
    triangle.points.push_back(footprint.polygon.points[i]);

    if (i != footprint.polygon.points.size() - 1) {
      triangle.points.push_back(footprint.polygon.points[i + 1]);
    } else {
      triangle.points.push_back(footprint.polygon.points[0]);
    }

    if (is_inside_of_triangle(obstacle, triangle)) {
      return true;
    }
  }

  return false;
}

bool DWAPlanner::is_inside_of_triangle(
  const geometry_msgs::msg::Point & target_point,
  const geometry_msgs::msg::Polygon & triangle)
{
  if (triangle.points.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "Not triangle, points size: %zu", triangle.points.size());

    return false;
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


  if ((cross1.z() >= 0 && cross2.z() >= 0 && cross3.z() >= 0) ||
    (cross1.z() <= 0 && cross2.z() <= 0 && cross3.z() <= 0))
  {
    return true;
  } else {
    return false;
  }
}

void DWAPlanner::motion(State & state, const double velocity, const double yawrate)
{
  const double sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_);
  state.yaw_ += yawrate * sim_time_step;
  state.x_ += velocity * std::cos(state.yaw_) * sim_time_step;
  state.y_ += velocity * std::sin(state.yaw_) * sim_time_step;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
}

void DWAPlanner::create_obs_list(const sensor_msgs::msg::LaserScan & scan)
{
  obs_list_.poses.clear();
  float angle = scan.angle_min;

  const int angle_index_step =
    static_cast<int>(std::round(angle_resolution_ / static_cast<double>(scan.angle_increment)));
  if (angle_index_step == 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "angle_index_step is zero, possibly due to angle_resolution_ being too small or scan.angle_increment being too large.");
    return;
  }
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    if (r < scan.range_min || scan.range_max < r || (i % angle_index_step != 0)) {
      angle += scan.angle_increment;
      continue;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = r * cos(angle);
    pose.position.y = r * sin(angle);
    obs_list_.poses.push_back(pose);
    angle += scan.angle_increment;
  }
}

void DWAPlanner::create_obs_list(const nav_msgs::msg::OccupancyGrid & map)
{
  obs_list_.poses.clear();
  const double max_search_dist = hypot(map.info.origin.position.x, map.info.origin.position.y);
  for (float angle = -M_PI; angle <= M_PI; angle += angle_resolution_) {
    for (float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = dist * cos(angle);
      pose.position.y = dist * sin(angle);
      const int index_x =
        static_cast<int>(floor(
          (pose.position.x - map.info.origin.position.x) /
          static_cast<double>(map.info.resolution)));
      const int index_y =
        static_cast<int>(floor(
          (pose.position.y - map.info.origin.position.y) /
          static_cast<double>(map.info.resolution)));

      if ((0 <= index_x && index_x < static_cast<int>(map.info.width)) &&
        (0 <= index_y && index_y < static_cast<int>(map.info.height)))
      {
        if (map.data[index_x + index_y * map.info.width] == 100) {
          obs_list_.poses.push_back(pose);
          break;
        }
      }
    }
  }
}

visualization_msgs::msg::Marker DWAPlanner::create_marker_msg(
  const int id, const double scale, const std_msgs::msg::ColorRGBA color,
  const std::vector<State> & trajectory,
  const geometry_msgs::msg::PolygonStamped & footprint)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = robot_frame_;
  marker.header.stamp = this->get_clock()->now();
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = scale;
  marker.color = color;
  marker.color.a = 0.8;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0 / hz_);

  geometry_msgs::msg::Point p;
  if (footprint.polygon.points.empty()) {
    for (const auto & point : trajectory) {
      p.x = point.x_;
      p.y = point.y_;
      marker.points.push_back(p);
    }
  } else {
    for (const auto & point : footprint.polygon.points) {
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

void DWAPlanner::visualize_trajectory(
  const std::vector<State> & trajectory,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0;
  visualization_msgs::msg::Marker v_trajectory = create_marker_msg(
    0, v_path_width_, color,
    trajectory);
  pub->publish(v_trajectory);
}

void DWAPlanner::visualize_trajectories(
  const std::vector<std::pair<std::vector<State>, bool>> & trajectories,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub)
{
  visualization_msgs::msg::MarkerArray v_trajectories;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    std_msgs::msg::ColorRGBA color;
    if (trajectories[i].second) {
      color.g = 1.0;
    } else {
      color.r = 0.5;
      color.b = 0.5;
    }
    visualization_msgs::msg::Marker v_trajectory = create_marker_msg(
      i, v_path_width_ * 0.4, color,
      trajectories[i].first);
    v_trajectories.markers.push_back(v_trajectory);
  }
  pub->publish(v_trajectories);
}

void DWAPlanner::visualize_footprints(
  const std::vector<State> & trajectory,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub)
{
  std_msgs::msg::ColorRGBA color;
  color.b = 1.0;
  visualization_msgs::msg::MarkerArray v_footprints;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const geometry_msgs::msg::PolygonStamped footprint = move_footprint(trajectory[i]);
    visualization_msgs::msg::Marker v_footprint = create_marker_msg(
      i, v_path_width_ * 0.2, color,
      trajectory, footprint);
    v_footprints.markers.push_back(v_footprint);
  }
  pub->publish(v_footprints);
}
