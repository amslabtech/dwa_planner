// Copyright 2023 amsl

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    : local_nh_("~"), footprint_subscribed_(false), goal_subscribed_(false), odom_updated_(false),
      edge_on_global_path_subscribed_(false), local_map_updated_(false), scan_updated_(false), has_reached_(false),
      odom_not_subscribe_count_(0), local_map_not_subscribe_count_(0), scan_not_subscribe_count_(0)
{
    local_nh_.param<std::string>("ROBOT_FRAME", robot_frame_, {"base_link"});
    local_nh_.param<double>("HZ", hz_, {20});
    local_nh_.param<double>("TARGET_VELOCITY", target_velocity_, {0.55});
    local_nh_.param<double>("MAX_VELOCITY", max_velocity_, {1.0});
    local_nh_.param<double>("MIN_VELOCITY", min_velocity_, {0.0});
    local_nh_.param<double>("MAX_YAWRATE", max_yawrate_, {1.0});
    local_nh_.param<double>("MAX_ACCELERATION", max_acceleration_, {1.0});
    local_nh_.param<double>("MAX_D_YAWRATE", max_d_yawrate_, {3.2});
    local_nh_.param<double>("ANGLE_RESOLUTION", angle_resolution_, {0.087});
    local_nh_.param<double>("PREDICT_TIME", predict_time_, {3.0});
    local_nh_.param<double>("DT", dt_, {0.1});
    local_nh_.param<double>("OBSTACLE_COST_GAIN", obs_cost_gain_, {1.0});
    local_nh_.param<double>("TO_GOAL_COST_GAIN", to_goal_cost_gain_, {1.0});
    local_nh_.param<double>("PATH_COST_GAIN", path_cost_gain_, {0.0});
    local_nh_.param<double>("GOAL_THRESHOLD", dist_to_goal_th_, {0.3});
    local_nh_.param<double>("TURN_DIRECTION_THRESHOLD", turn_direction_th_, {1.0});
    local_nh_.param<double>("ANGLE_TO_GOAL_TH", angle_to_goal_th_, {M_PI});
    local_nh_.param<double>("OBS_RANGE", obs_range_, {2.5});
    local_nh_.param<bool>("USE_SCAN_AS_INPUT", use_scan_as_input_, {false});
    local_nh_.param<bool>("USE_FOOTPRINT", use_footprint_, {false});
    local_nh_.param<bool>("USE_PATH_COST", use_path_cost_, {false});
    local_nh_.param<int>("SUBSCRIBE_COUNT_TH", subscribe_count_th_, {3});
    local_nh_.param<int>("VELOCITY_SAMPLES", velocity_samples_, {3});
    local_nh_.param<int>("YAWRATE_SAMPLES", yawrate_samples_, {20});

    ROS_INFO("=== DWA Planner ===");
    ROS_INFO_STREAM("ROBOT_FRAME: " << robot_frame_);
    ROS_INFO_STREAM("HZ: " << hz_);
    ROS_INFO_STREAM("TARGET_VELOCITY: " << target_velocity_);
    ROS_INFO_STREAM("MAX_VELOCITY: " << max_velocity_);
    ROS_INFO_STREAM("MIN_VELOCITY: " << min_velocity_);
    ROS_INFO_STREAM("MAX_YAWRATE: " << max_yawrate_);
    ROS_INFO_STREAM("MAX_ACCELERATION: " << max_acceleration_);
    ROS_INFO_STREAM("MAX_D_YAWRATE: " << max_d_yawrate_);
    ROS_INFO_STREAM("ANGLE_RESOLUTION: " << angle_resolution_);
    ROS_INFO_STREAM("PREDICT_TIME: " << predict_time_);
    ROS_INFO_STREAM("DT: " << dt_);
    ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << obs_cost_gain_);
    ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << to_goal_cost_gain_);
    ROS_INFO_STREAM("PATH_COST_GAIN: " << path_cost_gain_);
    ROS_INFO_STREAM("GOAL_THRESHOLD: " << dist_to_goal_th_);
    ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << turn_direction_th_);
    ROS_INFO_STREAM("ANGLE_TO_GOAL_TH: " << angle_to_goal_th_);
    ROS_INFO_STREAM("OBS_RANGE: " << obs_range_);
    ROS_INFO_STREAM("USE_SCAN_AS_INPUT: " << use_scan_as_input_);
    ROS_INFO_STREAM("USE_FOOTPRINT: " << use_footprint_);
    ROS_INFO_STREAM("USE_PATH_COST: " << use_path_cost_);
    ROS_INFO_STREAM("SUBSCRIBE_COUNT_TH: " << subscribe_count_th_);
    ROS_INFO_STREAM("VELOCITY_SAMPLES: " << velocity_samples_);
    ROS_INFO_STREAM("YAWRATE_SAMPLES: " << yawrate_samples_);

    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub_ = local_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
    predict_footprint_pub_ = local_nh_.advertise<geometry_msgs::PolygonStamped>("predict_footprint", 1);
    finish_flag_pub_ = local_nh_.advertise<std_msgs::Bool>("finish_flag", 1);

    dist_to_goal_th_sub_ = nh_.subscribe("/dist_to_goal_th", 1, &DWAPlanner::dist_to_goal_th_callback, this);
    edge_on_global_path_sub_ = nh_.subscribe("/path", 1, &DWAPlanner::edge_on_global_path_callback, this);
    footprint_sub_ = nh_.subscribe("/footprint", 1, &DWAPlanner::footprint_callback, this);
    goal_sub_ = nh_.subscribe("/local_goal", 1, &DWAPlanner::goal_callback, this);
    local_map_sub_ = nh_.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
    target_velocity_sub_ = nh_.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);

    if (!use_footprint_)
        footprint_subscribed_ = true;
    if (!use_path_cost_)
        edge_on_global_path_subscribed_ = true;
    if (!use_scan_as_input_)
        scan_updated_ = true;
    else
        local_map_updated_ = true;

    edge_point_on_path_.resize(2);
}

DWAPlanner::State::State(void) : x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

DWAPlanner::State::State(const double x, const double y, const double yaw, const double velocity, const double yawrate)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate)
{
}

DWAPlanner::Window::Window(void) : min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0) {}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    : min_velocity_(min_v), max_velocity_(max_v), min_yawrate_(min_y), max_yawrate_(max_y)
{
}

DWAPlanner::Cost::Cost(void) : to_goal_cost_(0.0), obs_cost_(0.0), path_cost_(0.0), total_cost_(0.0) {}

DWAPlanner::Cost::Cost(const float to_goal_cost, const float obs_cost, const float path_cost, const float total_cost)
    : to_goal_cost_(to_goal_cost), obs_cost_(obs_cost), path_cost_(path_cost), total_cost_(total_cost)
{
}

void DWAPlanner::Cost::show(void)
{
    ROS_INFO_STREAM("Cost: " << total_cost_);
    ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
    ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
    ROS_INFO_STREAM("\tPath cost: " << path_cost_);
}

void DWAPlanner::Cost::calc_total_cost(void) { total_cost_ = to_goal_cost_ + obs_cost_ + path_cost_; }

void DWAPlanner::goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    goal_ = *msg;
    try
    {
        listener_.transformPose(robot_frame_, ros::Time(0), goal_, goal_.header.frame_id, goal_);
        goal_subscribed_ = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void DWAPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (use_scan_as_input_)
        scan_to_obs(*msg);
    scan_not_subscribe_count_ = 0;
    scan_updated_ = true;
}

void DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    if (!use_scan_as_input_)
        raycast(*msg);
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
    ROS_INFO_THROTTLE(1.0, "target velocity was updated to %f [m/s]", target_velocity_);
}

void DWAPlanner::footprint_callback(const geometry_msgs::PolygonStampedPtr &msg)
{
    footprint_ = *msg;
    footprint_subscribed_ = true;
}

void DWAPlanner::dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg)
{
    dist_to_goal_th_ = msg->data;
    ROS_INFO_THROTTLE(1.0, "distance to goal threshold was updated to %f [m]", dist_to_goal_th_);
}

void DWAPlanner::edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg)
{
    edge_point_on_path_.front() = msg->poses.front();
    edge_point_on_path_.back() = msg->poses.back();
    try
    {
        for (auto &pose : edge_point_on_path_)
        {
            listener_.transformPose(robot_frame_, ros::Time(0), pose, msg->header.frame_id, pose);
            edge_on_global_path_subscribed_ = true;
        }
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

std::vector<DWAPlanner::State>
DWAPlanner::dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories)
{
    Cost min_cost(0.0, 0.0, 0.0, 1e6);
    Window dynamic_window = calc_dynamic_window();
    const size_t trajectory_size = predict_time_ / dt_;
    std::vector<State> best_traj;
    best_traj.resize(trajectory_size);
    std::vector<Cost> costs;
    const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1);
    costs.reserve(costs_size);

    const double velocity_resolution =
        std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / velocity_samples_, DBL_EPSILON);
    const double yawrate_resolution =
        std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / yawrate_samples_, DBL_EPSILON);

    int available_traj_count = 0;
    for (int i = 0; i < velocity_samples_; i++)
    {
        const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
        for (int j = 0; j < yawrate_samples_; j++)
        {
            std::pair<std::vector<State>, bool> traj;
            const double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;
            traj.first = generate_trajectory(v, y);
            Cost cost = evaluate_trajectory(traj.first, goal);
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
            Cost cost = evaluate_trajectory(traj.first, goal);
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
    min_cost.show();
    ROS_INFO_STREAM("num of trajectories available: " << available_traj_count << " of " << trajectories.size());
    ROS_INFO(" ");

    return best_traj;
}

void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> &costs)
{
    Cost min_cost(1e6, 1e6, 1e6, 1e6), max_cost;

    for (const auto &cost : costs)
    {
        if (cost.obs_cost_ != 1e6)
        {
            min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
            max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
            min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
            max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
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
            cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_);
            cost.to_goal_cost_ =
                (cost.to_goal_cost_ - min_cost.to_goal_cost_) / (max_cost.to_goal_cost_ - min_cost.to_goal_cost_);
            if (use_path_cost_)
                cost.path_cost_ = (cost.path_cost_ - min_cost.path_cost_) / (max_cost.path_cost_ - min_cost.path_cost_);
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
    if (!footprint_subscribed_)
        ROS_WARN_THROTTLE(1.0, "Robot Footprint has not been updated");
    if (!goal_subscribed_)
        ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
    if (!edge_on_global_path_subscribed_)
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

    if (footprint_subscribed_ && goal_subscribed_ && edge_on_global_path_subscribed_ &&
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

    const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation));

    if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_)
    {
        if (can_adjust_robot_direction(goal))
        {
            const double angle_to_goal = atan2(goal.y(), goal.x());
            cmd_vel.angular.z = std::min(std::max(angle_to_goal, -max_yawrate_), max_yawrate_);
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
            cmd_vel.angular.z = std::min(std::max(goal[2], -max_yawrate_), max_yawrate_);
        }
        else
        {
            has_finished_.data = true;
            has_reached_ = false;
        }
        best_traj.first = generate_trajectory(cmd_vel.linear.x, cmd_vel.angular.z);
        trajectories.push_back(best_traj);
    }

    visualize_trajectory(best_traj.first, 1, 0, 0, selected_trajectory_pub_);
    visualize_trajectories(trajectories, 0, 1, 0, 1000, candidate_trajectories_pub_);
    if (use_footprint_)
        predict_footprint_pub_.publish(transform_footprint(best_traj.first.back()));

    return cmd_vel;
}

bool DWAPlanner::can_adjust_robot_direction(const Eigen::Vector3d &goal)
{
    const double angle_to_goal = atan2(goal.y(), goal.x());
    if (fabs(angle_to_goal) < angle_to_goal_th_)
        return false;

    const double yawrate = std::min(std::max(angle_to_goal, -max_yawrate_), max_yawrate_);
    std::vector<State> traj = generate_trajectory(yawrate, goal);

    if (!check_collision(traj))
        return true;
    else
        return false;
}

bool DWAPlanner::check_collision(const std::vector<State> &traj)
{
    for (const auto &state : traj)
    {
        for (const auto &obs : obs_list_.poses)
        {
            const geometry_msgs::PolygonStamped footprint = transform_footprint(state);
            if (is_inside_of_robot(obs.position, footprint, state))
                return true;
        }
    }

    return false;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
    Window window(min_velocity_, max_velocity_, -max_yawrate_, max_yawrate_);
    window.min_velocity_ = std::max((current_cmd_vel_.linear.x - max_acceleration_ * dt_), min_velocity_);
    window.max_velocity_ = std::min((current_cmd_vel_.linear.x + max_acceleration_ * dt_), target_velocity_);
    window.min_yawrate_ = std::max((current_cmd_vel_.angular.z - max_d_yawrate_ * dt_), -max_yawrate_);
    window.max_yawrate_ = std::min((current_cmd_vel_.angular.z + max_d_yawrate_ * dt_), max_yawrate_);
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
                dist = hypot((state.x_ - obs.position.x), (state.y_ - obs.position.y));

            if (dist < DBL_EPSILON)
                return 1e6;
            min_dist = std::min(min_dist, dist);
        }
    }
    return obs_range_ - min_dist;
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
    geometry_msgs::Point edge_point1 = edge_point_on_path_.front().pose.position;
    geometry_msgs::Point edge_point2 = edge_point_on_path_.back().pose.position;
    const float a = edge_point2.y - edge_point1.y;
    const float b = -(edge_point2.x - edge_point1.x);
    const float c = -a * edge_point1.x - b * edge_point1.y;

    return fabs(a * state.x_ + b * state.y_ + c) / hypot(a, b);
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double velocity, const double yawrate)
{
    const size_t trajectory_size = predict_time_ / dt_;
    std::vector<State> trajectory;
    trajectory.resize(trajectory_size);
    State state;
    for (int i = 0; i < trajectory_size; i++)
    {
        motion(state, velocity, yawrate);
        trajectory[i] = state;
    }
    return trajectory;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double yawrate, const Eigen::Vector3d &goal)
{
    const double angle_to_goal = atan2(goal.y(), goal.x());
    const double predict_time = angle_to_goal / yawrate;
    const size_t trajectory_size = predict_time / dt_;
    std::vector<State> trajectory;
    trajectory.resize(trajectory_size);
    State state;
    for (int i = 0; i < trajectory_size; i++)
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
    const geometry_msgs::PolygonStamped footprint = transform_footprint(state);
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

geometry_msgs::PolygonStamped DWAPlanner::transform_footprint(const State &target_pose)
{
    geometry_msgs::PolygonStamped footprint = footprint_;
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
    state.yaw_ += yawrate * dt_;
    state.x_ += velocity * std::cos(state.yaw_) * dt_;
    state.y_ += velocity * std::sin(state.yaw_) * dt_;
    state.velocity_ = velocity;
    state.yawrate_ = yawrate;
}

void DWAPlanner::scan_to_obs(const sensor_msgs::LaserScan &scan)
{
    obs_list_.poses.clear();
    float angle = scan.angle_min;
    for (auto r : scan.ranges)
    {
        geometry_msgs::Pose pose;
        pose.position.x = r * cos(angle);
        pose.position.y = r * sin(angle);
        obs_list_.poses.push_back(pose);
        angle += scan.angle_increment;
    }
}

void DWAPlanner::raycast(const nav_msgs::OccupancyGrid &map)
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

void DWAPlanner::visualize_trajectories(
    const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const double r, const double g,
    const double b, const int trajectories_size, const ros::Publisher &pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for (; count < size; count++)
    {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = robot_frame_;
        v_trajectory.header.stamp = ros::Time::now();
        if (trajectories[count].second)
        {
            v_trajectory.color.r = r;
            v_trajectory.color.g = g;
            v_trajectory.color.b = b;
        }
        else
        {
            v_trajectory.color.r = 0.5;
            v_trajectory.color.g = 0.0;
            v_trajectory.color.b = 0.5;
        }
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        v_trajectory.pose = pose;
        geometry_msgs::Point p;
        for (const auto &pose : trajectories[count].first)
        {
            p.x = pose.x_;
            p.y = pose.y_;
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for (; count < trajectories_size; count++)
    {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = robot_frame_;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
    }
    pub.publish(v_trajectories);
}

void DWAPlanner::visualize_trajectory(
    const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = robot_frame_;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    for (const auto &pose : trajectory)
    {
        p.x = pose.x_;
        p.y = pose.y_;
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}
