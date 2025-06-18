// Copyright 2020 amsl

#ifndef DWA_PLANNER_DWA_PLANNER_H
#define DWA_PLANNER_DWA_PLANNER_H


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


#include <string>
#include <utility>
#include <vector>
#include <optional>
#include <cmath>
#include <limits>

// Eigen
#include <Eigen/Dense>

/**
 * @class DWAPlanner
 * @brief A class implementing a local planner using the Dynamic Window Approach
 */
class DWAPlanner: public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the DWAPlanner

   */
  explicit DWAPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
    State(
      const double x, const double y, const double yaw, const double velocity,
      const double yawrate);

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

  void process(void);

  void load_params(void);

  void print_params(void);

  void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  void local_map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void target_velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  void dist_to_goal_th_callback(const std_msgs::msg::Float64::ConstSharedPtr msg);

  void edge_on_global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);

  Window calc_dynamic_window(void);

  float calc_obs_cost(const std::vector < State > & traj);

  float calc_to_goal_cost(const std::vector < State > & traj, const Eigen::Vector3d & goal);

  float calc_speed_cost(const std::vector < State > & traj);

  float calc_path_cost(const std::vector < State > & traj);

  float calc_dist_to_path(const State state);

  void motion(State & state, const double velocity, const double yawrate);

  void create_obs_list(const nav_msgs::msg::OccupancyGrid & map);

  void create_obs_list(const sensor_msgs::msg::LaserScan & scan);

  float calc_dist_from_robot(const geometry_msgs::msg::Point & obstacle, const State & state);

  geometry_msgs::msg::PolygonStamped move_footprint(const State & target_pose);

  bool is_inside_of_robot(
    const geometry_msgs::msg::Point & obstacle,
    const geometry_msgs::msg::PolygonStamped & footprint, const State & state);

  bool is_inside_of_triangle(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Polygon & triangle);

  geometry_msgs::msg::Point
  calc_intersection(
    const geometry_msgs::msg::Point & obstacle, const State & state,
    geometry_msgs::msg::PolygonStamped footprint);

  std::vector < State > generate_trajectory(const double velocity, const double yawrate);

  std::vector < State > generate_trajectory(const double yawrate, const Eigen::Vector3d & goal);

  Cost evaluate_trajectory(const std::vector < State > & trajectory, const Eigen::Vector3d & goal);

  bool can_move(void);

  geometry_msgs::msg::Twist calc_cmd_vel(void);

  bool can_adjust_robot_direction(const Eigen::Vector3d & goal);

  bool check_collision(const std::vector < State > & traj);

  void normalize_costs(std::vector < Cost > & costs);

  visualization_msgs::msg::Marker create_marker_msg(
    const int id, const double scale, const std_msgs::msg::ColorRGBA color,
    const std::vector < State > & trajectory,
    const geometry_msgs::msg::PolygonStamped & footprint = geometry_msgs::msg::PolygonStamped());

  void visualize_trajectory(
    const std::vector < State > & trajectory,
    const rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr & pub);

  void visualize_trajectories(
    const std::vector < std::pair < std::vector < State >, bool >> & trajectories,
    const rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr & pub);

  void visualize_footprints(
    const std::vector < State > & trajectory,
    const rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr & pub);

  std::vector < State >
  dwa_planning(
    const Eigen::Vector3d & goal, std::vector < std::pair < std::vector < State >,
    bool >> &trajectories);

protected:
  // パラメータ
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

  // ROS 2 パブリッシャー
  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr velocity_pub_;
  rclcpp::Publisher < visualization_msgs::msg::MarkerArray >
  ::SharedPtr candidate_trajectories_pub_;
  rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr selected_trajectory_pub_;
  rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr predict_footprints_pub_;
  rclcpp::Publisher < std_msgs::msg::Bool > ::SharedPtr finish_flag_pub_;

  // ROS 2 サブスクライバー
  rclcpp::Subscription < std_msgs::msg::Float64 > ::SharedPtr dist_to_goal_th_sub_;
  rclcpp::Subscription < nav_msgs::msg::Path > ::SharedPtr edge_on_global_path_sub_;
  rclcpp::Subscription < geometry_msgs::msg::PolygonStamped > ::SharedPtr footprint_sub_;
  rclcpp::Subscription < geometry_msgs::msg::PoseStamped > ::SharedPtr goal_sub_;
  rclcpp::Subscription < nav_msgs::msg::OccupancyGrid > ::SharedPtr local_map_sub_;
  rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr scan_sub_;
  rclcpp::Subscription < geometry_msgs::msg::Twist > ::SharedPtr target_velocity_sub_;

  // データメンバー (ROS 2 メッセージ型)
  geometry_msgs::msg::Twist current_cmd_vel_;
  std::optional < geometry_msgs::msg::PoseStamped > goal_msg_;
  geometry_msgs::msg::PoseArray obs_list_;
  std::optional < geometry_msgs::msg::PolygonStamped > footprint_;
  std::optional < nav_msgs::msg::Path > edge_points_on_path_;

  std_msgs::msg::Bool has_finished_;

  // TF2
  std::unique_ptr < tf2_ros::Buffer > tf_buffer_;
  std::shared_ptr < tf2_ros::TransformListener > tf_listener_;
};

#endif  // DWA_PLANNER_DWA_PLANNER_H
