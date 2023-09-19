/**
* @file dwa_plannr.h
* @brief C++ implementation for dwa planner
* @author AMSL
*/
#ifndef __DWA_PLANNER_H
#define __DWA_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

/**
 * @brief Class for dwa planning
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
         * @brief Constractor
         * @param x The x position of robot
         * @param y The y position of robot
         * @param yaw The orientation of robot
         * @param velocity The linear velocity of robot
         * @param yawrate The angular velocity of robot
        */
        State(double, double, double, double, double);

        double x;
        double y;
        double yaw;
        double velocity;
        double yawrate;
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
        Window(const double, const double, const double, const double);

        double min_velocity;
        double max_velocity;
        double min_yawrate;
        double max_yawrate;
    private:
    };

    /**
     * @brief calculating local path plan
    */
    void process(void);
    /**
     * @brief A callback to hanldle buffering locacl goal messages
    */
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    /**
     * @brief A callback to hanldle buffering scan messages
    */
    void scan_callback(const sensor_msgs::LaserScanConstPtr&);
    /**
     * @brief A callback to hanldle buffering local map messages
    */
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    /**
     * @brief A callback to hanldle buffering odometry messages
    */
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    /**
     * @brief A callback to hanldle buffering target velocity messages
    */
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    /**
     * @brief calculating dynamic window
     * @return class The dynamic window
    */
    Window calc_dynamic_window(const geometry_msgs::Twist&);
    /**
     * @brief calclat the distance of current pose to goal pose
     * @param traj The estimated trajectory
     * @param goal The pose of goal
     * @return The distance of current pose to goal pose
    */
    float calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    /**
     * @brief calculat difference of target velocity from estimated velocity
     * @return The difference of velocity
    */
    float calc_speed_cost(const std::vector<State>& traj, const float target_velocity);
    /**
     * @brief caluclat distance from obstacle
     * @param traj Theestimated trajectory
     * @param obs_list The gird map informations which there is obstacle or not
     * @return The inverde of distance from obstacle
    */
    float calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float>>&);
    /**
     * @brief calculat pose 
     * @param velocity The velocity of robot
     * @param yawrate The angular velocity of robot
     * @param state The constractor setting pose information
    */
    void motion(State& state, const double velocity, const double yawrate);
    /**
     * @brief change map coordinates to robot coordinates
     * @return The position of obstacle
    */
    std::vector<std::vector<float>> raycast();
    /**
     * @brief calclat the position of obstacle
     * @return The position of obstacle
    */
    std::vector<std::vector<float>> scan_to_obs();
    /**
     * @brief publish candidate trajectories
     * @param trajectories candidated trajectory
     * @param r number of red in rgb color chart
     * @param g number of green in rgb color chart
     * @param b number of blue in rgb color chart
     * @param trajectories_size number of size about 
     * @param pub publisher of candidate trajectories
    */
    void visualize_trajectories(const std::vector<std::vector<State>>&, const double, const double, const double, const int, const ros::Publisher&);
    /**
     * @brief publish candidate trajectory
     * @param trajectory selected trajectry
     * @param r number of red in rgb color chart
     * @param g number of green in rgb color chart
     * @param b number of blue in rgb color chart
     * @param pub publisher of candidate trajectory
    */
    void visualize_trajectory(const std::vector<State>&, const double, const double, const double, const ros::Publisher&);
    /**
     * @brief execut dwa planner
     * @param window class of dynamic window
     * @param goal vector3d of goal pose
     * @param obs_list vector of obstacle's position  
    */
    std::vector<State> dwa_planning(Window, Eigen::Vector3d, std::vector<std::vector<float>>);

protected:
    double HZ;
    std::string ROBOT_FRAME;
    double TARGET_VELOCITY;
    double MAX_VELOCITY;
    double MIN_VELOCITY;
    double MAX_YAWRATE;
    double MAX_ACCELERATION;
    double MAX_D_YAWRATE;
    double MAX_DIST;
    double VELOCITY_RESOLUTION;
    double YAWRATE_RESOLUTION;
    double ANGLE_RESOLUTION;
    double PREDICT_TIME;
    double TO_GOAL_COST_GAIN;
    double SPEED_COST_GAIN;
    double OBSTACLE_COST_GAIN;
    double DT;
    bool USE_SCAN_AS_INPUT;
    double GOAL_THRESHOLD;
    double TURN_DIRECTION_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_velocity_sub;
    tf::TransformListener listener;

    geometry_msgs::PoseStamped local_goal;
    sensor_msgs::LaserScan scan;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_velocity;
    bool local_goal_subscribed;
    bool scan_updated;
    bool local_map_updated;
    bool odom_updated;
};

#endif //__DWA_PLANNER_H
