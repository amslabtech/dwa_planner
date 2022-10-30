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
    local_nh.param("TO_EDGE_COST_GAIN", TO_EDGE_COST_GAIN, {1.0});
    local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
    local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
    local_nh.param("GAIN_SLOPE", GAIN_SLOPE, {0.1});
    local_nh.param("GAIN_INTERCEPT", GAIN_INTERCEPT, {0.01});
    local_nh.param("SPEED_GAIN_RATE", SPEED_GAIN_RATE, {0.2});
    local_nh.param("GOAL_GAIN_RATE", GOAL_GAIN_RATE, {0.5});
    local_nh.param("EDGE_GAIN_RATE", EDGE_GAIN_RATE, {0.3});
    local_nh.param("GAIN_INTERCEPT", GAIN_INTERCEPT, {0.01});
    local_nh.param("USE_SCAN_AS_INPUT", USE_SCAN_AS_INPUT, {false});
    local_nh.param("USE_ACTIVE_GAIN", USE_ACTIVE_GAIN, {false});
    local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
    local_nh.param("THRESHOLD_OBS_EDGE_DIST", THRESHOLD_OBS_EDGE_DIST, {2.0});

    double FRONT_FRAME_DISTANCE, REAR_FRAME_DISTANCE, LEFT_FRAME_DISTANCE, RIGHT_FRAME_DISTANCE;
    local_nh.param("FRONT_FRAME_DISTANCE", FRONT_FRAME_DISTANCE, {0.5});
    local_nh.param("REAR_FRAME_DISTANCE", REAR_FRAME_DISTANCE, -FRONT_FRAME_DISTANCE);
    local_nh.param("LEFT_FRAME_DISTANCE", LEFT_FRAME_DISTANCE, {0.5});
    local_nh.param("RIGHT_FRAME_DISTANCE", RIGHT_FRAME_DISTANCE, -LEFT_FRAME_DISTANCE);
    set_robot_frames(FRONT_FRAME_DISTANCE, REAR_FRAME_DISTANCE, LEFT_FRAME_DISTANCE, RIGHT_FRAME_DISTANCE);
    local_nh.param("OBS_SEARCH_REDUCTION_RATE", OBS_SEARCH_REDUCTION_RATE, {1});
    local_nh.param("VISUALIZE_NEAREST_OBS", VISUALIZE_NEAREST_OBS, {false});
    if(VISUALIZE_NEAREST_OBS)
    {
        robot_frame_pub = local_nh.advertise<visualization_msgs::MarkerArray>("robot_frame", 1);
        nearest_obs_pub = local_nh.advertise<visualization_msgs::Marker>("nearest_obstacle", 1);
        nearest_obs_marker.header.frame_id = ROBOT_FRAME;
        nearest_obs_marker.header.stamp = ros::Time::now();
        nearest_obs_marker.ns = "nearest_obs";
        nearest_obs_marker.id = 0;
        nearest_obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        nearest_obs_marker.action = visualization_msgs::Marker::ADD;
        nearest_obs_marker.pose.orientation.w = 1.0;
        nearest_obs_marker.scale.x = 0.3;
        nearest_obs_marker.scale.y = 0.3;
        nearest_obs_marker.scale.z = 0.3;
        nearest_obs_marker.color.b = 1.0;
        nearest_obs_marker.color.a = 1.0;
        nearest_obs_marker.lifetime = ros::Duration(1.0/HZ);
    }
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
    ROS_INFO_STREAM("GAIN_SLOPE: " << GAIN_SLOPE);
    ROS_INFO_STREAM("GAIN_INTERCEPT: " << GAIN_INTERCEPT);
    ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);
    ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD);
    ROS_INFO_STREAM("SPEED_GAIN_RATE: " << SPEED_GAIN_RATE);
    ROS_INFO_STREAM("GOAL_GAIN_RATE: " << GOAL_GAIN_RATE);
    ROS_INFO_STREAM("EDGE_GAIN_RATE: " << EDGE_GAIN_RATE);
    ROS_INFO_STREAM("THRESHOLD_OBS_EDGE_DIST: " << THRESHOLD_OBS_EDGE_DIST);
    ROS_INFO_STREAM("FRONT_FRAME_DISTANCE: " << FRONT_FRAME_DISTANCE);
    ROS_INFO_STREAM("REAR_FRAME_DISTANCE: " << REAR_FRAME_DISTANCE);
    ROS_INFO_STREAM("LEFT_FRAME_DISTANCE: " << LEFT_FRAME_DISTANCE);
    ROS_INFO_STREAM("RIGHT_FRAME_DISTANCE: " << RIGHT_FRAME_DISTANCE);
    ROS_INFO_STREAM("OBS_SEARCH_REDUCTION_RATE: " << OBS_SEARCH_REDUCTION_RATE);

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    gain_pub = nh.advertise<dwa_planner::Gain>("/gain", 1);
    edge_gain_pub = nh.advertise<dwa_planner::Edge_Gain>("/edge_gain", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
    if(USE_SCAN_AS_INPUT){
        scan_sub = nh.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
    }else{
        local_map_sub = nh.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);
    }
    odom_sub = nh.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);
    current_checkpoint_sub = nh.subscribe("/current_checkpoint", 1, &DWAPlanner::current_checkpoint_callback, this);

    previous_checkpoint = current_checkpoint = -1;
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

void DWAPlanner::set_robot_frames(const double front, const double rear, const double left, const double right){
    ROBOT_FRAMES.clear();
    Frame frame(rear, right, front, right);
    ROBOT_FRAMES.push_back(frame);
    frame = Frame(front, right, front, left);
    ROBOT_FRAMES.push_back(frame);
    frame = Frame(front, left, rear, left);
    ROBOT_FRAMES.push_back(frame);
    frame = Frame(rear, left, rear, right);
    ROBOT_FRAMES.push_back(frame);
}

DWAPlanner::Frame::Frame(const float x1, const float y1, const float x2, const float y2)
{
    float angle1 = atan2(y1, x1);
    float angle2 = atan2(y2, x2);
    p1[0] = x1;
    p1[1] = y1;
    p2[0] = x2;
    p2[1] = y2;
    if(angle1 > angle2)
    {
        this->min_angle = angle1;
        this->max_angle = angle2;
    }
    else
    {
        this->min_angle = angle1;
        this->max_angle = angle2;
    }
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

void DWAPlanner::gain_slope_callback(const dwa_planner::GainSlopeConfig& msg)
{
    GAIN_SLOPE = msg.gain_slope;
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

void DWAPlanner::current_checkpoint_callback(const std_msgs::Int32ConstPtr& msg)
{
    current_checkpoint = msg->data;
}

std::vector<DWAPlanner::State> DWAPlanner::dwa_planning(
        Window dynamic_window,
        Eigen::Vector3d goal,
        std::vector<std::vector<float>> obs_list)
{
    float min_cost = 1e6;
    float min_obs_cost = min_cost;
    float min_goal_cost = min_cost;
    float min_edge_cost = min_cost;
    float min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectories;
    std::vector<State> best_traj;

    if(USE_ACTIVE_GAIN)
    {
        //for active gain
        float max_goal_cost = 0.0;
        float max_edge_cost = 0.0;
        float max_speed_cost = 0.0;
        float max_obs_cost = 0.0;

        std::vector<Cost> costs;

        for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION){
            for(float y=dynamic_window.min_yawrate; y<=dynamic_window.max_yawrate; y+=YAWRATE_RESOLUTION){
                State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
                std::vector<State> traj;
                Cost cost;

                for(float t=0; t<=PREDICT_TIME; t+=DT){
                    motion(state, v, y);
                    traj.push_back(state);
                }
                trajectories.push_back(traj);

                float to_goal_cost = calc_to_goal_cost(traj, goal);
                float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
                float obstacle_cost = calc_obstacle_cost(traj, obs_list);
                float to_edge_cost = calc_to_edge_cost(traj, goal);

                if(to_goal_cost > max_goal_cost) max_goal_cost = to_goal_cost;
                if(to_goal_cost < min_goal_cost) min_goal_cost = to_goal_cost;

                if(speed_cost > max_speed_cost) max_speed_cost = speed_cost;
                if(speed_cost < min_speed_cost) min_speed_cost = speed_cost;

                if(obstacle_cost != 1e6 && obstacle_cost > max_obs_cost) max_obs_cost = obstacle_cost;
                if(min_obs_cost != 1e6 && obstacle_cost < min_obs_cost) min_obs_cost = obstacle_cost;

                if(to_edge_cost > max_edge_cost) max_edge_cost = to_edge_cost;
                if(to_edge_cost < min_edge_cost) min_edge_cost = to_edge_cost;

                cost.to_goal_cost = to_goal_cost;
                cost.speed_cost = speed_cost;
                cost.obstacle_cost = obstacle_cost;
                cost.to_edge_cost = to_edge_cost;
                cost.traj = traj;
                costs.push_back(cost);
            }
        }

        float pile_weight_obstacle_cost = 0.0;
        float pile_weight = 0.0;
        for(const auto& cost : costs)
        {
            float to_goal_cost = cost.to_goal_cost;
            float to_obstacle_distance = 1 / cost.obstacle_cost; //from obstacle_cost to distance
            float weight = (max_goal_cost - to_goal_cost) / max_goal_cost;
            if(weight < 0.0001) weight = 0.0001;
            pile_weight += weight;
            pile_weight_obstacle_cost += weight * to_obstacle_distance;
        }
        pile_weight_obstacle_cost /= pile_weight;
        std::vector<float> each_gain = calc_each_gain(pile_weight_obstacle_cost);

        for(const auto& cost : costs)
        {
            if((max_goal_cost - min_goal_cost) < 0.0001) normalization_to_goal_cost = 1.0;
            else  normalization_to_goal_cost = (cost.to_goal_cost - min_goal_cost) / (max_goal_cost - min_goal_cost);

            if((max_speed_cost - min_speed_cost) < 0.0001) normalization_speed_cost = 1.0;
            else  normalization_speed_cost = (cost.speed_cost - min_speed_cost) / (max_speed_cost - min_speed_cost);

            if((max_obs_cost - min_obs_cost) < 0.0001) normalization_obstacle_cost = 1.0;
            else  normalization_obstacle_cost = (cost.obstacle_cost - min_obs_cost) / (max_obs_cost - min_obs_cost);

            if((max_edge_cost - min_edge_cost) < 0.0001) normalization_to_edge_cost = 1.0;
            else  normalization_to_edge_cost = (cost.to_edge_cost - min_edge_cost) / (max_edge_cost - min_edge_cost);

            float final_cost =
                each_gain[0]*normalization_to_goal_cost +
                each_gain[2]*normalization_speed_cost +
                each_gain[3]*normalization_obstacle_cost +
                each_gain[1]*normalization_to_edge_cost;
            // ROS_INFO_STREAM(each_gain[2]*normalization_speed_cost);
            // float final_cost =
            //     each_gain[0]*cost.to_goal_cost +
            //     each_gain[2]*cost.speed_cost +
            //     each_gain[3]*cost.obstacle_cost +
            //     each_gain[1]*cost.to_edge_cost;

            if(min_cost >= final_cost){
                min_goal_cost = each_gain[0]*normalization_to_goal_cost;
                min_edge_cost = each_gain[1]*normalization_to_edge_cost;
                min_obs_cost = each_gain[3]*normalization_obstacle_cost;
                min_speed_cost = each_gain[2]*normalization_speed_cost;
                min_cost = final_cost;
                best_traj = cost.traj;
            }
            // if(min_cost >= final_cost){
            //     min_goal_cost = each_gain[0]*cost.to_goal_cost;
            //     min_edge_cost = each_gain[1]*cost.to_edge_cost;
            //     min_obs_cost = each_gain[3]*cost.obstacle_cost;
            //     min_speed_cost = each_gain[2]*cost.speed_cost;
            //     min_cost = final_cost;
            //     best_traj = cost.traj;
            // }

        }
    }

    else
    {

        float weight_edge_cost = calc_obs_to_edge(obs_list, goal);
        dwa_planner::Edge_Gain edge_gain;
        edge_gain.to_edge_gain = weight_edge_cost;
        edge_gain_pub.publish(edge_gain);

        ROS_INFO_STREAM("weight_edge_cost");
        ROS_INFO_STREAM(weight_edge_cost);

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
                float to_edge_cost = calc_to_edge_cost(traj, goal);
                float final_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost + TO_EDGE_COST_GAIN*weight_edge_cost*to_edge_cost;
                if(min_cost >= final_cost){
                    min_goal_cost = TO_GOAL_COST_GAIN*to_goal_cost;
                    min_edge_cost = TO_EDGE_COST_GAIN*weight_edge_cost*to_edge_cost;
                    min_obs_cost = OBSTACLE_COST_GAIN*obstacle_cost;
                    min_speed_cost = SPEED_COST_GAIN*speed_cost;
                    min_cost = final_cost;
                    best_traj = traj;
                }
            }
        }
    }

    ROS_INFO_STREAM("Cost: " << min_cost);
    ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
    ROS_INFO_STREAM("- Edge cost: " << min_edge_cost);
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

void DWAPlanner::turn_until_straight(const geometry_msgs::PoseStamped& goal, int& p_checkpoint, int& c_checkpoint, bool& flag)
{
    if(p_checkpoint == -1 || c_checkpoint == -1) flag = false;

    if(p_checkpoint == c_checkpoint) {
        if(turn_flag == true) {
            // double goal_yaw = tf::getYaw(goal.pose.orientation);
            // double diff_abs = fabs(goal_yaw);
            double to_goal_angle = atan2(goal.pose.position.y, goal.pose.position.x);
            double diff_abs = fabs(to_goal_angle);
            if(diff_abs < 0.2) turn_flag = false;
        }
        else flag = false;
    }
    else flag = true;
    p_checkpoint = c_checkpoint;
    // flag = false;
}

void DWAPlanner::process(void)
{
    ros::Rate loop_rate(HZ);

    assert((SPEED_GAIN_RATE + GOAL_GAIN_RATE + EDGE_GAIN_RATE) == double(1.0));
    while(ros::ok()){
        ROS_INFO("==========================================");
        if(USE_ACTIVE_GAIN)
        {
            f = boost::bind(&DWAPlanner::gain_slope_callback, this, _1);
            server.setCallback(f);
        }

        double start = ros::Time::now().toSec();
        bool input_updated = false;
        if(USE_SCAN_AS_INPUT && scan_updated){
            input_updated = true;
        }else if(!USE_SCAN_AS_INPUT && local_map_updated){
            input_updated = true;
        }
        if(input_updated && local_goal_subscribed && odom_updated){
            if(VISUALIZE_NEAREST_OBS)
            {
                robot_frames_marker.markers.clear();
                nearest_obs_marker.points.clear();
                nearest_obs_marker.colors.clear();
            }
            turn_until_straight(local_goal, previous_checkpoint, current_checkpoint, turn_flag);
            if(turn_flag){
                double goal_yaw = tf::getYaw(local_goal.pose.orientation);

                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = MAX_YAWRATE / 5.0 * goal_yaw / fabs(goal_yaw);
                ROS_INFO_STREAM("cmd_vel: (" << cmd_vel.linear.x << "[m/s], " << cmd_vel.angular.z << "[rad/s])");
                velocity_pub.publish(cmd_vel);

                scan_updated = false;
                local_map_updated = false;
                odom_updated = false;
            }else{
                Window dynamic_window = calc_dynamic_window(current_velocity);
                Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
                // Eigen::Vector3d goal(3.0, 0.0, 0);
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
                    if(VISUALIZE_NEAREST_OBS)
                    {
                        int count = 0;
                        for(const auto& state : best_traj)
                        {
                            if(count%OBS_SEARCH_REDUCTION_RATE != 0)
                            {
                                count++;
                                continue;
                            }
                            std::vector<Frame> robot_frame;
                            for(int i=0;i<4;i++)
                            {
                                Frame frame = transform_nearest_frame(state, ROBOT_FRAMES[i]);
                                robot_frame.push_back(frame);
                            }
                            push_back_to_frame_array(robot_frame);
                            float min_dist = 1e3;
                            std::vector<float> nearest_obs;
                            for(const auto& obs : obs_list){
                                float dist = calc_distance_from_robot(state, obs);
                                if(dist <= local_map.info.resolution){
                                    nearest_obs = obs;
                                    break;
                                }
                                if(min_dist > dist){
                                    min_dist = dist;
                                    nearest_obs = obs;
                                }
                            }
                            push_back_to_nearest_obs_marker(nearest_obs);
                            count++;
                        }
                    }
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
                if(VISUALIZE_NEAREST_OBS)
                {
                    robot_frame_pub.publish(robot_frames_marker);
                    nearest_obs_marker.header.stamp = ros::Time::now();
                    static int id = 0;
                    nearest_obs_marker.id = id++;
                    nearest_obs_pub.publish(nearest_obs_marker);
                }

                odom_updated = false;
            }
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
        // old_goal = goal;
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
    int count = 0;
    for(const auto& state : traj){
        if(count%OBS_SEARCH_REDUCTION_RATE != 0){
            count++;
            continue;
        }
        // if(VISUALIZE_NEAREST_OBS)
        // {
        //     std::vector<Frame> robot_frame;
        //     for(int i=0;i<4;i++)
        //     {
        //         Frame frame = transform_nearest_frame(state, ROBOT_FRAMES[i]);
        //         robot_frame.push_back(frame);
        //     }
        //     push_back_to_frame_array(robot_frame);
        // }
        float state_dist = 1e3;
        std::vector<float> nearest_obs;
        for(const auto& obs : obs_list){
            float dist = calc_distance_from_robot(state, obs);
            if(dist <= local_map.info.resolution){
                cost = 1e6;
                return cost;
            }
            min_dist = std::min(min_dist, dist);
            // if(VISUALIZE_NEAREST_OBS && state_dist > dist)
            // {
            //     state_dist = dist;
            //     nearest_obs = obs;
            // }
        }
        // if(VISUALIZE_NEAREST_OBS) push_back_to_nearest_obs_marker(nearest_obs);
        count++;
    }
    cost = 1.0 / min_dist;
    // ROS_INFO_STREAM(cost);
    return cost;
}

int DWAPlanner::judge_nearest_frame(const State& state, const std::vector<float>& obs)
{
    float obs_angle = atan2(obs[1]-state.y, obs[0]-state.x);
    obs_angle -= state.yaw;
    if(obs_angle > ROBOT_FRAMES[DIRECTION::RIGHT].min_angle && obs_angle < ROBOT_FRAMES[DIRECTION::RIGHT].max_angle) return DIRECTION::RIGHT;
    else if(obs_angle > ROBOT_FRAMES[DIRECTION::FRONT].min_angle && obs_angle < ROBOT_FRAMES[DIRECTION::FRONT].max_angle) return DIRECTION::FRONT;
    else if(obs_angle > ROBOT_FRAMES[DIRECTION::LEFT].min_angle && obs_angle < ROBOT_FRAMES[DIRECTION::LEFT].max_angle) return DIRECTION::LEFT;
    else return DIRECTION::REAR;
}

DWAPlanner::Frame DWAPlanner::transform_nearest_frame(const State& state, const Frame& frame)
{
    float x1 = frame.p1[0]*std::cos(state.yaw) - frame.p1[1]*std::sin(state.yaw) + state.x;
    float y1 = frame.p1[0]*std::sin(state.yaw) + frame.p1[1]*std::cos(state.yaw) + state.y;
    float x2 = frame.p2[0]*std::cos(state.yaw) - frame.p2[1]*std::sin(state.yaw) + state.x;
    float y2 = frame.p2[0]*std::sin(state.yaw) + frame.p2[1]*std::cos(state.yaw) + state.y;
    Frame transformed_frame(x1, y1, x2, y2);
    return transformed_frame;
}

float DWAPlanner::calc_distance_from_frame(const Frame& frame, const std::vector<float>& obs)
{
    float dx21 = frame.p2[0] - frame.p1[0];
    float dy21 = frame.p2[1] - frame.p1[1];
    float dx10 = frame.p1[0] - obs[0];
    float dy10 = frame.p1[1] - obs[1];
    float dx20 = frame.p2[0] - obs[0];
    float dy20 = frame.p2[1] - obs[1];

    double t = -(dx21*dx10 + dy21*dy10);
    if(t < 0) return std::hypot(dx10, dy10);
    else if(t > dx21*dx21 + dy21*dy21) return std::hypot(dx20, dy20);
    else return sqrt((dx21*dy10 - dy21*dx10) * (dx21*dy10 - dy21*dx10) / (dx21*dx21 + dy21*dy21));
}

bool DWAPlanner::is_inside(const State& state, const Frame& frame, const std::vector<float>& obs)
{
    std::vector<std::vector<float>> polygon = {{state.x, state.y},
                                               frame.p1,
                                               frame.p2,
                                              };
    int cn = 0;
    int n = polygon.size();
    for(int i=0; i<n; i++)
    {
        if(  ((polygon[i][1] <= obs[1]) && (polygon[(i+1)%n][1] > obs[1])) ||
             ((polygon[i][1] > obs[1]) && (polygon[(i+1)%n][1] <= obs[1])) )
        {
            float vt = (obs[1] - polygon[i][1]) / (polygon[(i+1)%n][1] - polygon[i][1]);
            if(obs[0] < (polygon[i][0] + (vt * (polygon[(i+1)%n][0] - polygon[i][0]) ) ) )
                cn++;
        }
    }
    if(cn%2 == 0) return false;
    else
    {
        ROS_WARN_STREAM("Oh no! I hit the obstacle!!");
        return true;
    }
}

float DWAPlanner::calc_distance_from_robot(const State& state, const std::vector<float>& obs)
{
    int direction = judge_nearest_frame(state, obs);
    Frame nearest_frame = transform_nearest_frame(state, ROBOT_FRAMES[direction]);
    if(is_inside(state, nearest_frame, obs)) return 0.0;
    else return calc_distance_from_frame(nearest_frame, obs);
    // float dist_from_base = std::hypot(obs[0]-state.x, obs[1]-state.y);
}

void DWAPlanner::push_back_to_nearest_obs_marker(const std::vector<float>& obs)
{
    geometry_msgs::Point point;
    point.x = obs[0];
    point.y = obs[1];
    point.z = 0.01;
    nearest_obs_marker.points.push_back(point);
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.b = 1.0;
    color.a = 1.0;
    nearest_obs_marker.colors.push_back(color);
}

void DWAPlanner::push_back_to_frame_array(const std::vector<Frame>& robot_frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = ROBOT_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "robot_frame";
    static int id = 0;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0.01;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0;
    marker.color = color;
    marker.lifetime = ros::Duration(1/HZ);
    for(const auto& frame : robot_frame)
    {
        geometry_msgs::Point p1, p2;
        p1.x = frame.p1[0];
        p1.y = frame.p1[1];
        p2.x = frame.p2[0];
        p2.y = frame.p2[1];
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.colors.push_back(color);
        marker.colors.push_back(color);
    }
    robot_frames_marker.markers.push_back(marker);
}

float DWAPlanner::calc_obs_to_edge(const std::vector<std::vector<float>>& obs_list, const Eigen::Vector3d& goal)
{
    float min_dist = 1e2;

    double a = std::tan(goal(2));
    double b = goal(1) - a*goal(0);

    for(auto& obs : obs_list)
    {
        double obs_edge_dist = std::sqrt(std::pow(obs[0],2.0) + std::pow(obs[1],2.0));
        if(obs_edge_dist < THRESHOLD_OBS_EDGE_DIST && obs[0] > 0.0)
        {
            double dist = std::fabs(a*obs[0] - obs[1] + b) / std::sqrt(std::pow(a,2.0) + std::pow(-1,2.0));
            if(dist < min_dist) min_dist = dist;

        }
    }


    double robot_edge_dist = std::fabs(a*0.0 - 0.0 + b) / std::sqrt(std::pow(a,2.0) + std::pow(-1,2.0));
    ROS_INFO_STREAM("robot_edge_dist");
    ROS_INFO_STREAM(min_dist);

    if(min_dist == 1e2)
    {
        if(robot_edge_dist*0.5 > 1.0) return 1.0;
        else return robot_edge_dist*0.5;
    }
    else
    {
        if(min_dist*robot_edge_dist*0.5 > 1.0) return 1.0;
        else return min_dist*robot_edge_dist*0.5;
    }
}

float DWAPlanner::calc_to_edge_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    double a = std::tan(goal(2));
    double b = goal(1) - a*goal(0);
    double pile_d = 0.0;
    int i = 1;

    for(const auto& state : traj)
    {
        double d = std::fabs(a*state.x - state.y + b) / std::sqrt(std::pow(a,2.0) + std::pow(-1,2.0));
        pile_d += d;
        i++;
    }
    // Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
    // double d = std::fabs(a*last_position(0) - last_position(1) + b) / std::sqrt(std::pow(a,2.0) + std::pow(-1,2.0));
    return pile_d/i;
}

std::vector<float> DWAPlanner::calc_each_gain(const float pile_weight_obstacle_cost)
{
    std::vector<float> each_gain;
    float gain = GAIN_SLOPE * pile_weight_obstacle_cost + GAIN_INTERCEPT;
    if(gain >= 1.0) gain = 1.0;
    float to_goal_gain = GOAL_GAIN_RATE * gain;
    float to_edge_gain = EDGE_GAIN_RATE * gain;
    float speed_gain = SPEED_GAIN_RATE * gain;
    float obstacle_gain = 1.0 - gain;
    each_gain.push_back(to_goal_gain);
    each_gain.push_back(to_edge_gain);
    each_gain.push_back(speed_gain);
    each_gain.push_back(obstacle_gain);

    dwa_planner::Gain _gain;
    _gain.to_goal_gain = to_goal_gain;
    _gain.to_edge_gain = to_edge_gain;
    _gain.speed_gain = speed_gain;
    _gain.obstacle_gain = obstacle_gain;
    gain_pub.publish(_gain);

    return each_gain;
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

