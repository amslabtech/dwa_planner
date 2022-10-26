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
#include <cmath>
#include <Eigen/Dense>
#include <dwa_planner/Gain.h>
#include <dwa_planner/Edge_Gain.h>
#include <dynamic_reconfigure/server.h>
#include <dwa_planner/GainSlopeConfig.h>
#include <cassert>

class DWAPlanner
{
public:
    DWAPlanner(void);

    class State
    {
    public:
        State(double, double, double, double, double);

        double x;// robot position x
        double y;// robot posiiton y
        double yaw;// robot orientation yaw
        double velocity;// robot linear velocity
        double yawrate;// robot angular velocity
    private:
    };

    class Window
    {
    public:
        Window(void);
        Window(const double, const double, const double, const double);
        double min_velocity;
        double max_velocity;
        double min_yawrate;
        double max_yawrate;
    private:
    };

    //for active gain
    class Cost
    {
    public:
        float to_goal_cost;
        float speed_cost;
        float obstacle_cost;
        float to_edge_cost;
        std::vector<State> traj;
    private:
    };

    class Frame
    {
    public:
        Frame(const float, const float, const float, const float);
        float min_angle;
        float max_angle;
        std::vector<float> p1={0,0};
        std::vector<float> p2={0,0};
    private:
    };
    enum DIRECTION
    {
        RIGHT=0, FRONT=1, LEFT=2, REAR=3,
    };

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void scan_callback(const sensor_msgs::LaserScanConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void gain_slope_callback(const dwa_planner::GainSlopeConfig&);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    Window calc_dynamic_window(const geometry_msgs::Twist&);
    float calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    float calc_to_edge_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    float calc_speed_cost(const std::vector<State>& traj, const float target_velocity);
    float calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float>>&);
    float calc_obs_to_edge(const std::vector<std::vector<float>>&, const Eigen::Vector3d& goal);
    void motion(State& state, const double velocity, const double yawrate);
    std::vector<std::vector<float>> raycast();
    std::vector<std::vector<float>> scan_to_obs();
    void visualize_trajectories(const std::vector<std::vector<State>>&, const double, const double, const double, const int, const ros::Publisher&);
    void visualize_trajectory(const std::vector<State>&, const double, const double, const double, const ros::Publisher&);
    std::vector<State> dwa_planning(Window, Eigen::Vector3d, std::vector<std::vector<float>>);
    std::vector<float> calc_each_gain(const float pile_weight_obstacle_cost);
    float calc_distance_from_robot(const State& state, const std::vector<float>& obs);
    void set_robot_frames(const double front, const double rear, const double left, const double right);
    int judge_nearest_frame(const State& state, const std::vector<float>& obs);
    Frame transform_nearest_frame(const State& state, const Frame& frame);
    float calc_distance_from_frame(const Frame& frame, const std::vector<float>& obs);


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
    double TO_EDGE_COST_GAIN;
    double SPEED_COST_GAIN;
    double OBSTACLE_COST_GAIN;
    double SPEED_GAIN_RATE;
    double GOAL_GAIN_RATE;
    double EDGE_GAIN_RATE;
    double DT;
    bool USE_SCAN_AS_INPUT;
    bool USE_ACTIVE_GAIN;
    double GOAL_THRESHOLD;
    double TURN_DIRECTION_THRESHOLD;
    double THRESHOLD_OBS_EDGE_DIST;
    double GAIN_SLOPE;
    double GAIN_INTERCEPT;
    float normalization_to_goal_cost;
    float normalization_to_edge_cost;
    float normalization_obstacle_cost;
    float normalization_speed_cost;
    std::vector<Frame> ROBOT_FRAMES;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Publisher gain_pub;
    ros::Publisher edge_gain_pub;
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
    dynamic_reconfigure::Server<dwa_planner::GainSlopeConfig> server;
    dynamic_reconfigure::Server<dwa_planner::GainSlopeConfig>::CallbackType f;

    // Eigen::Vector3d old_goal(0.0,0.0,0.0);
    // Eigen::Vector3d goal(0.0,0.0,0.0);
    bool local_goal_subscribed;
    bool scan_updated;
    bool local_map_updated;
    bool odom_updated;
};

#endif //__DWA_PLANNER_H
