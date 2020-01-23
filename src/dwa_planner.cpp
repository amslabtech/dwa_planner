#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    :local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
    local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
    local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.1});
    local_nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, {0.1});
    local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
    local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {0.15});
    local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
    local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
    DT = 1.0 / HZ;

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "DT: " << DT << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME << std::endl;
    std::cout << "TARGET_VELOCITY: " << TARGET_VELOCITY << std::endl;
    std::cout << "MAX_VELOCITY: " << MAX_VELOCITY << std::endl;
    std::cout << "MIN_VELOCITY: " << MIN_VELOCITY << std::endl;
    std::cout << "MAX_YAWRATE: " << MAX_YAWRATE << std::endl;
    std::cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << std::endl;
    std::cout << "MAX_D_YAWRATE: " << MAX_D_YAWRATE << std::endl;
    std::cout << "VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION << std::endl;
    std::cout << "YAWRATE_RESOLUTION: " << YAWRATE_RESOLUTION << std::endl;
    std::cout << "PREDICT_TIME: " << PREDICT_TIME << std::endl;
    std::cout << "TO_GOAL_COST_GAIN: " << TO_GOAL_COST_GAIN << std::endl;
    std::cout << "SPEED_COST_GAIN: " << SPEED_COST_GAIN << std::endl;
    std::cout << "OBSTACLE_COST_GAIN: " << OBSTACLE_COST_GAIN << std::endl;


    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
    local_map_sub = nh.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);

    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
    dynamic_window = window;
    local_goal_subscribed = false;
    local_map_updated = false;
    odom_updated = false;

}

DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _yawrate)
{
    x = _x;
    y = _y;
    yaw = _yaw;
    velocity = _velocity;
    yawrate = _yawrate;
}

DWAPlanner::Window::Window(void)
{
    min_velocity = 0.0;
    max_velocity = 0.0;
    min_yawrate = 0.0;
    max_yawrate = 0.0;
}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
{
    min_velocity = min_v;
    max_velocity = max_v;
    min_yawrate = min_y;
    max_yawrate = max_y;
}

void DWAPlanner::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal = *msg;
    try{
        listener.transformPose(ROBOT_FRAME, ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        local_goal_subscribed = true;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
    }
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
    std::cout << "\033[31mtarget velocity was updated to " << TARGET_VELOCITY << "[m/s]\033[0m" << std::endl;
}
void DWAPlanner::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(local_map_updated && local_goal_subscribed && odom_updated){
            std::cout << "=== dwa planner ===" << std::endl;
            double start = ros::Time::now().toSec();
            std::cout << "local goal: \n" << local_goal << std::endl;
            Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            calc_dynamic_window(dynamic_window, current_velocity);
            float min_cost = 1e6;

            std::vector<std::vector<State>> trajectories;
            std::vector<State> best_traj;
            for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION){
                for(float y=dynamic_window.min_yawrate; y<=dynamic_window.max_yawrate; y+=YAWRATE_RESOLUTION){
                    State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
                    std::vector<State> traj;
                    for(float t=0; t<=PREDICT_TIME; t+=DT){
                        motion(state, v, y);
                        traj.push_back(state);
                        t += DT;
                    }
                    trajectories.push_back(traj);

                    float to_goal_cost = calc_to_goal_cost(traj, goal);
                    float speed_cost = 0.0;//calc_speed_cost(traj, TARGET_VELOCITY);
                    float obstacle_cost = 0.0;//calc_obstacle_cost(traj, local_map);
                    float final_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost;
                    if(min_cost >= final_cost){
                        min_cost = final_cost;
                        best_traj = traj;
                    }
                }
            }
            std::cout << "min cost: " << min_cost << std::endl;
            std::cout << "trajectories size: " << trajectories.size() << std::endl;
            visualize_trajectories(trajectories, 0, 1, 0, 100, candidate_trajectories_pub);

            std::cout << "publish velocity" << std::endl;
            geometry_msgs::Twist cmd_vel;
            if(min_cost!=1e6){
                cmd_vel.linear.x = best_traj[0].velocity;
                cmd_vel.angular.z = best_traj[0].yawrate;
                visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
            }else{
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            std::cout << "cmd_vel: \n" << cmd_vel << std::endl;
            velocity_pub.publish(cmd_vel);

            local_map_updated = false;
            odom_updated = false;

            std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }else{
            if(!local_goal_subscribed){
                std::cout << "waiting for local goal" << std::endl;
            }
            if(!local_map_updated){
                std::cout << "waiting for local map" << std::endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void DWAPlanner::calc_dynamic_window(Window& window, const geometry_msgs::Twist& current_velocity)
{
    window.min_velocity = std::max((current_velocity.linear.x - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_velocity = std::min((current_velocity.linear.x + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_yawrate = std::max((current_velocity.angular.z - MAX_D_YAWRATE*DT), -MAX_YAWRATE);
    window.max_yawrate = std::min((current_velocity.angular.z + MAX_D_YAWRATE*DT),  MAX_YAWRATE);
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_speed_cost(const std::vector<State>& traj, const float target_velocity)
{
    float cost = target_velocity - traj[traj.size()-1].velocity;
    return cost;
}

float DWAPlanner::calc_obstacle_cost(const std::vector<State>& traj, const nav_msgs::OccupancyGrid& map)
{
    float cost = 0.0;
    float min_dis = 1000.0;
    bool collision = false;
    for(int t=0; t<traj.size(); t++){
        for(int i=0; i<map.data.size(); i++){
            if(map.data[i] != 0){
                float x = (i%map.info.width - map.info.width/2.0 + 0.5)*map.info.resolution;
                float y = (i/map.info.width - map.info.height/2.0 + 0.5)*map.info.resolution;
                float dis = sqrt((traj[t].x-x)*(traj[t].x-x)+(traj[t].y-y)*(traj[t].y-y));
                if(dis<1e-3){
                    collision = true;
                    break;
                }
                if(min_dis >= dis){
                    min_dis = dis;
                }
            }
        }
        if(collision){
            break;
        }
    }
    if(collision){
        cost = 1e6;
    }else{
        cost = 1.0/min_dis;
    }
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
    geometry_msgs::Point p;
    for(const auto& pose : trajectory){
        p.x = pose.x;
        p.y = pose.y;
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}
