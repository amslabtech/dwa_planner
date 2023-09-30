// Copyright 2023 amsl

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
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

/**
 * @brief Class for dwa planner
 */
class DWAPlanner
{
public:
    /**
     * @brief Constructor for the DWAPlanner
     */
    DWAPlanner(void);

    /**
     * @brief Class for set pose
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
     * @brief Class for calculating dynamic window
     */
    class Window
    {
    public:
        /**
         * @brief Constructor
         */
        Window(void);
        /**
         * @brief Constructor
         * @param min_velocity The minimum velocity of robot
         * @param max_velocity The maximum velocity of robot
         * @param min_yawrate The minimum angular velocity of robot
         * @param max_yawrate The maximum angular velocity of robot
         */
        Window(
                const double min_velocity,
                const double max_velocity,
                const double min_yawrate,
                const double max_yawrate);

        double min_velocity_;
        double max_velocity_;
        double min_yawrate_;
        double max_yawrate_;

    private:
    };

    /**
     * @brief Calculate local path plan
     */
    void process(void);
    /**
     * @brief A callback to hanldle buffering locacl goal messages
     */
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr &msg);
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
     * @brief
     */
    void footprint_callback(const geometry_msgs::PolygonStampedPtr &msg);
    /**
     * @brief
     */
    void dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg);
    /**
     * @brief Calculate dynamic window
     * @return The dynamic window
     */
    Window calc_dynamic_window(void);
    /**
     * @brief Calculate the distance of current pose to goal pose
     * @param traj The estimated trajectory
     * @param goal The pose of goal
     * @return The distance of current pose to goal pose
     */
    float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
    /**
     * @brief Calculate difference of target velocity from estimated velocity
     * @return The difference of velocity
     */
    float calc_speed_cost(const std::vector<State> &traj);
    /**
     * @brief Caluclate distance from obstacle
     * @param traj Theestimated trajectory
     * @param obs_list The gird map informations which there is obstacle or not
     * @return The inverde of distance from obstacle
     */
    float calc_obs_cost(const std::vector<State> &traj);
    /**
     * @brief Calculate the pose of robot
     * @param velocity The velocity of robot
     * @param yawrate The angular velocity of robot
     * @param state The constractor setting pose information
     */
    void motion(State &state, const double velocity, const double yawrate);
    /**
     * @brief Change map coordinates to robot coordinates
     * @return The position of obstacle
     */
    void raycast(const nav_msgs::OccupancyGrid &map);
    /**
     * @brief  Calculate the position of obstacle
     * @return The position of obstacle
     */
    void scan_to_obs(const sensor_msgs::LaserScan &scan);
    /**
     * @brief
     */
    float calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state);
    /**
     * @brief
     */
    geometry_msgs::PolygonStamped transform_footprint(const State &target_pose);
    /**
     * @brief
     */
    bool is_inside_of_robot(
            const geometry_msgs::Point &obstacle,
            const geometry_msgs::PolygonStamped &footprint,
            const State &state);
    /**
     * @brief
     */
    bool is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle);
    /**
     * @brief
     */
    geometry_msgs::Point calc_intersection(
            const geometry_msgs::Point &obstacle,
            const State &state,
            geometry_msgs::PolygonStamped footprint);
    /**
     * @brief
     */
    void generate_trajectory(std::vector<State> &trajectory, const double velocity, const double yawrate);
    /**
     * @brief
     */
    void evaluate_trajectory(
            const std::vector<State> &trajectory,
            float &to_goal_cost,
            float &speed_cost,
            float &obs_cost,
            float &total_cost,
            const Eigen::Vector3d &goal);
    /**
     * @brief
     */
    bool can_move(void);
    /**
     * @brief
     */
    geometry_msgs::Twist calc_cmd_vel(void);
    /**
     * @brief
     */
    bool can_adjust_robot_direction(const Eigen::Vector3d &goal);
    /**
     * @brief
     */
    bool check_collision(const std::vector<State> &traj);
    /**
     * @brief Publish candidate trajectories
     * @param trajectories Candidated trajectory
     * @param r Rgb color chart number of red
     * @param g Rgb color chart number of green
     * @param b Rgb color chart number of blue
     * @param trajectories_size Size of candidate trajectories
     * @param pub Publisher of candidate trajectories
     */
    void visualize_trajectories(
            const std::vector<std::vector<State>> &trajectories,
            const double r,
            const double g,
            const double b,
            const int trajectories_size,
            const ros::Publisher &pub);
    /**
     * @brief Publish candidate trajectory
     * @param trajectory Selected trajectry
     * @param r Rgb color chart number of red
     * @param g Rgb color chart number of green
     * @param b Rgb color chart number of blue
     * @param pub Publisher of candidate trajectory
     */
    void visualize_trajectory(
        const std::vector<State> &trajectory,
        const double r,
        const double g,
        const double b,
        const ros::Publisher &pub);
    /**
     * @brief Execut dwa planner
     * @param window Dynamic window
     * @param goal Goal pose
     * @param obs_list Obstacle's position
     */
    std::vector<State> dwa_planning(Eigen::Vector3d goal);

protected:
    std::string robot_frame_;
    double hz_;
    double target_velocity_;
    double max_velocity_;
    double min_velocity_;
    double max_yawrate_;
    double max_acceleration_;
    double max_d_yawrate_;
    double angle_resolution_;
    double predict_time_;
    double dt_;
    double to_goal_cost_gain_;
    double speed_cost_gain_;
    double obs_cost_gain_;
    double dist_to_goal_th_;
    double turn_direction_th_;
    double angle_to_goal_th_;
    bool use_footprint_;
    bool use_scan_as_input_;
    bool footprint_subscribed_;
    bool local_goal_subscribed_;
    bool odom_updated_;
    bool local_map_updated_;
    bool scan_updated_;
    bool has_reached_;
    int velocity_samples_;
    int yawrate_samples_;
    int subscribe_count_th_;
    int odom_not_subscribe_count_;
    int local_map_not_subscribe_count_;
    int scan_not_subscribe_count_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher velocity_pub_;
    ros::Publisher candidate_trajectories_pub_;
    ros::Publisher selected_trajectory_pub_;
    ros::Publisher predict_footprint_pub_;
    ros::Publisher finish_flag_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber local_map_sub_;
    ros::Subscriber local_goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber target_velocity_sub_;
    ros::Subscriber footprint_sub_;
    ros::Subscriber dist_to_goal_th_sub_;

    geometry_msgs::Twist current_cmd_vel_;
    geometry_msgs::PoseStamped local_goal_;
    geometry_msgs::PoseArray obs_list_;
    geometry_msgs::PolygonStamped footprint_;

    std_msgs::Bool has_finished_;

    tf::TransformListener listener_;
};

#endif  // DWA_PLANNER_DWA_PLANNER_H
