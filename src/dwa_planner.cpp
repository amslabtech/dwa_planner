#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    :local_nh("~"), local_goal_subscribed(false), scan_updated(false), local_map_updated(false), odom_updated(false)
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
    local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
    local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    local_nh.param("MAX_DIST", MAX_DIST, {10.0});
    local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.1});
    local_nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, {0.1});
    local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
    local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
    local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {1.0});
    local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
    local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
    local_nh.param("USE_SCAN_AS_INPUT", USE_SCAN_AS_INPUT, {false});
    local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
    DT = 1.0 / HZ;

    ROS_INFO("=== DWA Planner ===");
    ROS_INFO_STREAM("HZ: " << HZ);
    ROS_INFO_STREAM("DT: " << DT);
    ROS_INFO_STREAM("ROBOT_FRAME: " << ROBOT_FRAME);
    ROS_INFO_STREAM("TARGET_VELOCITY: " << TARGET_VELOCITY);
    ROS_INFO_STREAM("MAX_VELOCITY: " << MAX_VELOCITY);
    ROS_INFO_STREAM("MIN_VELOCITY: " << MIN_VELOCITY);
    ROS_INFO_STREAM("MAX_YAWRATE: " << MAX_YAWRATE);
    ROS_INFO_STREAM("MAX_ACCELERATION: " << MAX_ACCELERATION);
    ROS_INFO_STREAM("MAX_D_YAWRATE: " << MAX_D_YAWRATE);
    ROS_INFO_STREAM("MAX_DIST: " << MAX_DIST);
    ROS_INFO_STREAM("VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION);
    ROS_INFO_STREAM("YAWRATE_RESOLUTION: " << YAWRATE_RESOLUTION);
    ROS_INFO_STREAM("ANGLE_RESOLUTION: " << ANGLE_RESOLUTION);
    ROS_INFO_STREAM("PREDICT_TIME: " << PREDICT_TIME);
    ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << TO_GOAL_COST_GAIN);
    ROS_INFO_STREAM("SPEED_COST_GAIN: " << SPEED_COST_GAIN);
    ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << OBSTACLE_COST_GAIN);
    ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);
    ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD);

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
    if(USE_SCAN_AS_INPUT){
        scan_sub = nh.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
    }else{
        local_map_sub = nh.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);
    }
    odom_sub = nh.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);
}

DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _yawrate)
    :x(_x), y(_y), yaw(_yaw), velocity(_velocity), yawrate(_yawrate)
{
}

DWAPlanner::Window::Window(void)
    :min_velocity(0.0), max_velocity(0.0), min_yawrate(0.0), max_yawrate(0.0)
{
}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    :min_velocity(min_v), max_velocity(max_v), min_yawrate(min_y), max_yawrate(max_y)
{
}

void DWAPlanner::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal = *msg;
    try{
        listener.transformPose(ROBOT_FRAME, ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        local_goal_subscribed = true;
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
}

void DWAPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    scan = *msg;
    scan_updated = true;
}

void DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    local_map = *msg;
    local_map_updated = true;
}

void DWAPlanner::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
    odom_updated = true;
}

void DWAPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    TARGET_VELOCITY = msg->linear.x;
    ROS_INFO_STREAM("target velocity was updated to " << TARGET_VELOCITY << "[m/s]");
}

std::vector<DWAPlanner::State> DWAPlanner::dwa_planning(
        Window dynamic_window, 
        Eigen::Vector3d goal,
        std::vector<std::vector<float>> obs_list)
{
    float min_cost = 1e6;
    float min_obs_cost = min_cost;
    float min_goal_cost = min_cost;
    float min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectories;
    std::vector<State> best_traj;

    for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION){
        for(float y=dynamic_window.min_yawrate; y<=dynamic_window.max_yawrate; y+=YAWRATE_RESOLUTION){
            State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
            std::vector<State> traj;
            for(float t=0; t<=PREDICT_TIME; t+=DT){
                motion(state, v, y);
                traj.push_back(state);
            }
            trajectories.push_back(traj);

            float to_goal_cost = calc_to_goal_cost(traj, goal);
            float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
            float obstacle_cost = calc_obstacle_cost(traj, obs_list);
            float final_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost;
            if(min_cost >= final_cost){
                min_goal_cost = TO_GOAL_COST_GAIN*to_goal_cost;
                min_obs_cost = OBSTACLE_COST_GAIN*obstacle_cost;
                min_speed_cost = SPEED_COST_GAIN*speed_cost;
                min_cost = final_cost;
                best_traj = traj;
            }
        }
    }
    ROS_INFO_STREAM("Cost: " << min_cost);
    ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
    ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
    ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
    ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
    visualize_trajectories(trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
    if(min_cost == 1e6){
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
        traj.push_back(state);
        best_traj = traj;
    }
    return best_traj;
}

void DWAPlanner::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        ROS_INFO("==========================================");
        double start = ros::Time::now().toSec();
        bool input_updated = false;
        if(USE_SCAN_AS_INPUT && scan_updated){
            input_updated = true;
        }else if(!USE_SCAN_AS_INPUT && local_map_updated){
            input_updated = true;
        }
        if(input_updated && local_goal_subscribed && odom_updated){
            Window dynamic_window = calc_dynamic_window(current_velocity);
            Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            ROS_INFO_STREAM("local goal: (" << goal[0] << "," << goal[1] << "," << goal[2]/M_PI*180 << ")");

            geometry_msgs::Twist cmd_vel;
            if(goal.segment(0, 2).norm() > GOAL_THRESHOLD){
                std::vector<std::vector<float>> obs_list;
                if(USE_SCAN_AS_INPUT){
                    obs_list = scan_to_obs();
                    scan_updated = false;
                }else{
                    obs_list = raycast();
                    local_map_updated = false;
                }

                std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);

                cmd_vel.linear.x = best_traj[0].velocity;
                cmd_vel.angular.z = best_traj[0].yawrate;
                visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
            }else{
                cmd_vel.linear.x = 0.0;
                if(fabs(goal[2])>TURN_DIRECTION_THRESHOLD){
                    cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                }
                else{
                    cmd_vel.angular.z = 0.0;
                }
            }
            ROS_INFO_STREAM("cmd_vel: (" << cmd_vel.linear.x << "[m/s], " << cmd_vel.angular.z << "[rad/s])");
            velocity_pub.publish(cmd_vel);

            odom_updated = false;
        }else{
            if(!local_goal_subscribed){
                ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
            }
            if(!odom_updated){
                ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
            }
            if(!USE_SCAN_AS_INPUT && !local_map_updated){
                ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
            }
            if(USE_SCAN_AS_INPUT && !scan_updated){
                ROS_WARN_THROTTLE(1.0, "Scan has not been updated");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM("loop time: " << ros::Time::now().toSec() - start << "[s]");
    }
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(const geometry_msgs::Twist& current_velocity)
{
    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
    window.min_velocity = std::max((current_velocity.linear.x - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_velocity = std::min((current_velocity.linear.x + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_yawrate = std::max((current_velocity.angular.z - MAX_D_YAWRATE*DT), -MAX_YAWRATE);
    window.max_yawrate = std::min((current_velocity.angular.z + MAX_D_YAWRATE*DT),  MAX_YAWRATE);
    return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_speed_cost(const std::vector<State>& traj, const float target_velocity)
{
    float cost = fabs(target_velocity - fabs(traj[traj.size()-1].velocity));
    return cost;
}

float DWAPlanner::calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float>>& obs_list)
{
    float cost = 0.0;
    float min_dist = 1e3;
    for(const auto& state : traj){
        for(const auto& obs : obs_list){
            float dist = sqrt((state.x - obs[0])*(state.x - obs[0]) + (state.y - obs[1])*(state.y - obs[1]));
            if(dist <= local_map.info.resolution){
                cost = 1e6;
                return cost;
            }
            min_dist = std::min(min_dist, dist);
        }
    }
    cost = 1.0 / min_dist;
    return cost;
}

void DWAPlanner::motion(State& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate*DT;
    state.x += velocity*std::cos(state.yaw)*DT;
    state.y += velocity*std::sin(state.yaw)*DT;
    state.velocity = velocity;
    state.yawrate = yawrate;
}

std::vector<std::vector<float>> DWAPlanner::scan_to_obs()
{
    std::vector<std::vector<float>> obs_list;
    float angle = scan.angle_min;
    for(auto r : scan.ranges){
        float x = r * cos(angle);
        float y = r * sin(angle);
        std::vector<float> obs_state = {x, y};
        obs_list.push_back(obs_state);
        angle += scan.angle_increment;
    }
    return obs_list;
}

std::vector<std::vector<float>> DWAPlanner::raycast()
{
    std::vector<std::vector<float>> obs_list;
    for(float angle = -M_PI; angle <= M_PI; angle += ANGLE_RESOLUTION){
        for(float dist = 0.0; dist <= MAX_DIST; dist += local_map.info.resolution){
            float x = dist * cos(angle);
            float y = dist * sin(angle);
            int i = floor(x / local_map.info.resolution + 0.5) + local_map.info.width * 0.5;
            int j = floor(y / local_map.info.resolution + 0.5) + local_map.info.height * 0.5;
            if( (i < 0 || i >= local_map.info.width) || (j < 0 || j >= local_map.info.height) ){
                break;
            }
            if(local_map.data[j*local_map.info.width + i] == 100){
                std::vector<float> obs_state = {x, y};
                obs_list.push_back(obs_state);
                break;
            }
        }
    }
    return obs_list;
}

void DWAPlanner::visualize_trajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
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
        for(const auto& pose : trajectories[count]){
            p.x = pose.x;
            p.y = pose.y;
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for(;count<trajectories_size;){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }
    pub.publish(v_trajectories);
}

void DWAPlanner::visualize_trajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
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
    for(const auto& pose : trajectory){
        p.x = pose.x;
        p.y = pose.y;
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}
