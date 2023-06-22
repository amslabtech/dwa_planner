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
     * @brief Constructor
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
         * @param[in] _x Number of robot position x
         * @param[in] _y Number of robot position y
         * @param[in] _yaw Number of robot orientation yaw
         * @param[in] _velocity Numbre of robot linear velocity
         * @param[in] _yawrate Number of robot angular velocity
        */
        State(double, double, double, double, double);

        double x;// robot position x
        double y;// robot posiiton y
        double yaw;// robot orientation yaw
        double velocity;// robot linear velocity
        double yawrate;// robot angular velocity
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
         * @param[in] _min_velocity Number of minimum velocity
         * @param[in] _max_velocity Number of maximum velocity
         * @param[in] _min_yawrate Number of minimum angular velocity
         * @param[in] _max_yawrate Number of maximam angular velocity
        */
        Window(const double, const double, const double, const double);

        double min_velocity;// minimum velocity
        double max_velocity;// maximum velocity
        double min_yawrate;// minimun angular velocity
        double max_yawrate;// maximum angular velocity
    private:
    };

    /**
     * @brief calculating local path plan
    */
    void process(void);
    /**
     * @brief set local goal
    */
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    /**
     * @brief set scan information
    */
    void scan_callback(const sensor_msgs::LaserScanConstPtr&);
    /**
     * @brief set lacal map
    */
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    /**
     * @brief set odometry information
    */
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    /**
     * @brief set target velocity information
    */
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    /**
     * @brief calculating dynamic window
     * @return class dynamic window
    */
    Window calc_dynamic_window(const geometry_msgs::Twist&);
    /**
     * @brief calclating the distance of current pose to goal pose
     * @param[in] traj estimated trajectory
     * @param[in] goal pose of goal
     * @return float distance of current pose to goal pose
    */
    float calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    /**
     * @brief calculating difference of target velocity from estimated velocity
     * @return float difference of velocity
    */
    float calc_speed_cost(const std::vector<State>& traj, const float target_velocity);
    /**
     * @brief calculating distance from obstacle
     * @param[in] traj estimated trajectory
     * @param[in] obs_list gird map informations which there is obstacle or not
     * @return float inverde of distance from obstacle
    */
    float calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float>>&);
    /**
     * @berif calculating pose 
     * @param[in] velocity velocity
     * @param[in] yawrate angular velocity
     * @param[out] state constractor setting pose information
    */
    void motion(State& state, const double velocity, const double yawrate);
    /**
     * @brief changing map coordinates to robot coordinates
     * @return vector position of obstacle
    */
    std::vector<std::vector<float>> raycast();
    /**
     * @brief caluculating position of obstacle
     * @return vector position of obstacle
    */
    std::vector<std::vector<float>> scan_to_obs();
    /**
     * @brief publishing candidate trajectories
     * @param[in] trajectories candidated trajectory
     * @param[in] r number of red in rgb color chart
     * @param[in] g number of green in rgb color chart
     * @param[in] b number of blue in rgb color chart
     * @param[in] trajectories_size number of size about 
     * @param[out] pub publisher of candidate trajectories
    */
    void visualize_trajectories(const std::vector<std::vector<State>>&, const double, const double, const double, const int, const ros::Publisher&);
    /**
     * @brief publishing candidate trajectory
     * @param[in] trajectory selected trajectry
     * @param[in] r number of red in rgb color chart
     * @param[in] g number of green in rgb color chart
     * @param[in] b number of blue in rgb color chart
     * @param[out] pub publisher of candidate trajectory
    */
    void visualize_trajectory(const std::vector<State>&, const double, const double, const double, const ros::Publisher&);
    /**
     * @brief executing dwa planner
     * @param[in] window class of dynamic window
     * @param[in] goal vector3d of goal pose
     * @param[in] obs_list vector of obstacle's position  
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
