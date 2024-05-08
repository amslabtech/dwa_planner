// Copyright 2020 amsl

/**
 * @file dwa_plannr.h
 * @brief C++ implementation for dwa planner
 * @author AMSL
 */

#ifndef DWA_PLANNER_DWA_PLANNER_H
#define DWA_PLANNER_DWA_PLANNER_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

/**
 * @class DWAPlanner
 * @brief A class implementing a local planner using the Dynamic Window Approach
 */
class DWAPlanner
{
public:
  /**
   * @brief Constructor for the DWAPlanner
   */
  DWAPlanner(void);

  /**
   * @class State
   * @brief A data class for state of robot
   */
  class State
  {
  public:
    /**
     * @brief Constructor
     */
    State(void);

    /**
     * @brief Constractor
     * @param x The x position of robot
     * @param y The y position of robot
     * @param yaw The orientation of robot
     * @param velocity The linear velocity of robot
     * @param yawrate The angular velocity of robot
     */
    State(const double x, const double y, const double yaw, const double velocity, const double yawrate);

    double x_;
    double y_;
    double yaw_;
    double velocity_;
    double yawrate_;

  private:
  };

  /**
   * @class Window
   * @brief A data class for dynamic window
   */
  class Window
  {
  public:
    /**
     * @brief Constructor
     */
    Window(void);

    /**
     * @brief Show the dynamic window information
     */
    void show(void);

    double min_velocity_;
    double max_velocity_;
    double min_yawrate_;
    double max_yawrate_;

  private:
  };

  /**
   * @class Cost
   * @brief A data class for cost
   */
  class Cost
  {
  public:
    /**
     * @brief Constructor
     */
    Cost(void);

    /**
     * @brief Constructor
     * @param obs_cost The cost of obstacle
     * @param to_goal_cost The cost of distance to goal
     * @param speed_cost The cost of speed
     * @param path_cost The cost of path
     * @param total_cost The total cost
     */
    Cost(
        const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
        const float total_cost);

    /**
     * @brief Show the cost
     */
    void show(void);

    /**
     * @brief Calculate the total cost
     */
    void calc_total_cost(void);

    float obs_cost_;
    float to_goal_cost_;
    float speed_cost_;
    float path_cost_;
    float total_cost_;

  private:
  };

  /**
   * @brief Execute local path planning
   */
  void process(void);

  /**
   * @brief Load parameters
   */
  void load_params(void);

  /**
   * @brief Print parameters
   */
  void print_params(void);

  /**
   * @brief A callback to hanldle buffering local goal messages
   */
  void goal_callback(const geometry_msgs::PoseStampedConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering scan messages
   */
  void scan_callback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering local map messages
   */
  void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering odometry messages
   */
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering target velocity messages
   */
  void target_velocity_callback(const geometry_msgs::TwistConstPtr &msg);

  /**
   * @brief A calllback to handle buffering footprint messages
   */
  void footprint_callback(const geometry_msgs::PolygonStampedPtr &msg);

  /**
   * @brief A callback to handle buffering distance to goal threshold messages
   */
  void dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg);

  /**
   * @brief A callback to handle buffering edge on global path messages
   */
  void edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg);

  /**
   * @brief Calculate dynamic window
   * @return The dynamic window
   */
  Window calc_dynamic_window(void);

  /**
   * @brief Calculate obstacle cost
   * @param traj The estimated trajectory
   * @return The obstacle cost
   */
  float calc_obs_cost(const std::vector<State> &traj);

  /**
   * @brief Calculate the distance of current pose to goal pose
   * @param traj The estimated trajectory
   * @param goal The pose of goal
   * @return The distance of current pose to goal pose
   */
  float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);

  /**
   * @brief Calculate the speed cost
   * @param traj The estimated trajectory
   * @return The speed cost
   */
  float calc_speed_cost(const std::vector<State> &traj);

  /**
   * @brief Calculate the path cost
   * @param traj The estimated trajectory
   * @return The path cost
   */
  float calc_path_cost(const std::vector<State> &traj);

  /**
   * @brief Calculate the distance of current pose to global path
   * @param state The robot state
   * @return The distance of current pose to global path
   */
  float calc_dist_to_path(const State state);

  /**
   * @brief Simulate the robot motion
   * @param state The start state of robot
   * @param velocity The velocity of robot
   * @param yawrate The angular velocity of robot
   */
  void motion(State &state, const double velocity, const double yawrate);

  /**
   * @brief Get obstacle list from local map
   * @param map The local map
   */
  void create_obs_list(const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Get obstacle list from laser scan
   * @param scan The laser scan
   */
  void create_obs_list(const sensor_msgs::LaserScan &scan);

  /**
   * @brief Calculate the distance from robot footprint to the nearest obstacle
   * @param obstacle The position of obstacle
   * @param state The robot state
   * @return The distance from robot footprint to the nearest obstacle
   */
  float calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state);

  /**
   * @brief Move the robot footprint to the target pose
   * @param target_pose The target pose
   * @return The moved footprint
   */
  geometry_msgs::PolygonStamped move_footprint(const State &target_pose);

  /**
   * @brief Check if the obstacle is inside of robot footprint
   * @param obstacle The position of obstacle
   * @param footprint The robot footprint
   * @param state The robot state
   * @return True if the obstacle is inside of robot footprint
   */
  bool is_inside_of_robot(
      const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint, const State &state);

  /**
   * @brief Check if the target point is inside of triangle
   * @param target_point The target point
   * @param triangle The triangle
   * @return True if the target point is inside of triangle
   */
  bool is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle);

  /**
   * @brief Calculate the intersection point of the line and the circle
   * @param obstacle The position of obstacle
   * @param state The robot state
   * @param footprint The robot footprint
   * @return The intersection point of the line and the circle
   */
  geometry_msgs::Point
  calc_intersection(const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint);

  /**
   * @brief Generate trajectory
   * @param velocity The velocity of robot
   * @param yawrate The angular velocity of robot
   * @return The generated trajectory
   */
  std::vector<State> generate_trajectory(const double velocity, const double yawrate);

  /**
   * @brief Generate trajectory
   * @param yawrate The angular velocity of robot
   * @param goal The pose of goal
   * @return The generated trajectory
   */
  std::vector<State> generate_trajectory(const double yawrate, const Eigen::Vector3d &goal);

  /**
   * @brief Evaluate trajectory
   * @param trajectory The estimated trajectory
   * @param goal The pose of goal
   * @return The cost of trajectory
   */
  Cost evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal);

  /**
   * @brief Check if the robot can move
   * @return True if the robot can move
   */
  bool can_move(void);

  /**
   * @brief Calculate the command velocity
   * @return The command velocity
   */
  geometry_msgs::Twist calc_cmd_vel(void);

  /**
   * @brief Check if the robot can adjust the direction
   * @param goal The pose of goal
   * @return True if the robot can adjust the direction
   */
  bool can_adjust_robot_direction(const Eigen::Vector3d &goal);

  /**
   * @brief Check if the robot has collided
   * @param traj The estimated trajectory
   * @return True if the robot has collided
   */
  bool check_collision(const std::vector<State> &traj);

  /**
   * @brief Normalize the costs
   * @param costs array of costs
   */
  void normalize_costs(std::vector<Cost> &costs);

  /**
   * @brief Create a marker message
   * @param id The id of marker
   * @param scale The scale of marker
   * @param color The color of marker
   * @param trajectory The estimated trajectory
   * @param footprint The robot footprint
   */
  visualization_msgs::Marker create_marker_msg(
      const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
      const geometry_msgs::PolygonStamped &footprint = geometry_msgs::PolygonStamped());

  /**
   * @brief Publish selected trajectory
   * @param trajectory Selected trajectry
   * @param pub Publisher of selected trajectory
   */
  void visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub);

  /**
   * @brief Publish candidate trajectories
   * @param trajectories Candidated trajectories
   * @param pub Publisher of candidate trajectories
   */
  void visualize_trajectories(
      const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub);

  /**
   * @brief Publish predicted footprints
   * @param trajectory Selected trajectry
   * @param pub Publisher of predicted footprints
   */
  void visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub);

  /**
   * @brief Execute dwa planning
   * @param window Dynamic window
   * @param goal Goal pose
   * @param obs_list Obstacle's position
   */
  std::vector<State>
  dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories);

protected:
  std::string global_frame_;
  std::string robot_frame_;
  double hz_;
  double target_velocity_;
  double max_velocity_;
  double min_velocity_;
  double max_yawrate_;
  double min_yawrate_;
  double max_in_place_yawrate_;
  double min_in_place_yawrate_;
  double max_acceleration_;
  double max_deceleration_;
  double max_d_yawrate_;
  double sim_period_;
  double angle_resolution_;
  double predict_time_;
  double sleep_time_after_finish_;
  double obs_cost_gain_;
  double to_goal_cost_gain_;
  double speed_cost_gain_;
  double path_cost_gain_;
  double dist_to_goal_th_;
  double turn_direction_th_;
  double angle_to_goal_th_;
  double sim_direction_;
  double slow_velocity_th_;
  double obs_range_;
  double robot_radius_;
  double footprint_padding_;
  double v_path_width_;
  bool use_footprint_;
  bool use_scan_as_input_;
  bool use_path_cost_;
  bool use_speed_cost_;
  bool odom_updated_;
  bool local_map_updated_;
  bool scan_updated_;
  bool has_reached_;
  int velocity_samples_;
  int yawrate_samples_;
  int sim_time_samples_;
  int subscribe_count_th_;
  int odom_not_subscribe_count_;
  int local_map_not_subscribe_count_;
  int scan_not_subscribe_count_;

  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Publisher velocity_pub_;
  ros::Publisher candidate_trajectories_pub_;
  ros::Publisher selected_trajectory_pub_;
  ros::Publisher predict_footprints_pub_;
  ros::Publisher finish_flag_pub_;
  ros::Subscriber dist_to_goal_th_sub_;
  ros::Subscriber edge_on_global_path_sub_;
  ros::Subscriber footprint_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber local_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber target_velocity_sub_;

  geometry_msgs::Twist current_cmd_vel_;
  std::optional<geometry_msgs::PoseStamped> goal_msg_;
  geometry_msgs::PoseArray obs_list_;
  std::optional<geometry_msgs::PolygonStamped> footprint_;
  std::optional<nav_msgs::Path> edge_points_on_path_;

  std_msgs::Bool has_finished_;

  tf::TransformListener listener_;
};

#endif  // DWA_PLANNER_DWA_PLANNER_H
