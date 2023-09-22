#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    :local_nh("~"), local_goal_subscribed(false), scan_updated(false), local_map_updated(false), odom_updated(false), footprint_subscribed(true), local_map_not_sub_count(0)
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
    local_nh.param("USE_FOOTPRINT", USE_FOOTPRINT, {false});
    local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
    local_nh.param("ANGLE_TO_GOAL_TH", ANGLE_TO_GOAL_TH, {M_PI});
    local_nh.param("OBS_SEARCH_REDUCTION_RATE", OBS_SEARCH_REDUCTION_RATE, {1});
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
    ROS_INFO_STREAM("USE_SCAN_AS_INPUT: " << USE_SCAN_AS_INPUT);
    ROS_INFO_STREAM("USE_FOOTPRINT: " << USE_FOOTPRINT);
    ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);
    ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD);
    ROS_INFO_STREAM("ANGLE_TO_GOAL_TH: " << ANGLE_TO_GOAL_TH);
    ROS_INFO_STREAM("OBS_SEARCH_REDUCTION_RATE: " << OBS_SEARCH_REDUCTION_RATE);

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
    predict_footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("predict_footprint", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);
    footprint_sub = nh.subscribe("/footprint", 1, &DWAPlanner::footprint_callback, this);
    if(USE_SCAN_AS_INPUT){
        scan_sub = nh.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
    }else{
        local_map_sub = nh.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);
    }

    if(USE_FOOTPRINT) footprint_subscribed = false;
}

DWAPlanner::State::State(void)
    :x(0.0), y(0.0), yaw(0.0), velocity(0.0), yawrate(0.0)
{
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
    local_map_not_sub_count = 0;
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

void DWAPlanner::footprint_callback(const geometry_msgs::PolygonStampedPtr& msg)
{
    base_footprint = *msg;
    footprint_subscribed = true;
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
            std::vector<State> traj;
            generate_trajectory(traj, v, y);
            trajectories.push_back(traj);

            float to_goal_cost, speed_cost, obs_cost, total_cost;
            evaluate_trajectory(traj, to_goal_cost, speed_cost, obs_cost, total_cost, goal, obs_list);

            if(min_cost >= total_cost){
                min_goal_cost = to_goal_cost;
                min_obs_cost = obs_cost;
                min_speed_cost = speed_cost;
                min_cost = total_cost;
                best_traj = traj;
            }
        }

        if(dynamic_window.min_yawrate < 0.0 and 0.0 < dynamic_window.max_yawrate)
        {
            std::vector<State> traj;
            generate_trajectory(traj, v, 0.0);
            trajectories.push_back(traj);

            float to_goal_cost, speed_cost, obs_cost, total_cost;
            evaluate_trajectory(traj, to_goal_cost, speed_cost, obs_cost, total_cost, goal, obs_list);

            if(min_cost >= total_cost){
                min_goal_cost = to_goal_cost;
                min_obs_cost = obs_cost;
                min_speed_cost = speed_cost;
                min_cost = total_cost;
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
        State state(0.0, 0.0, 0.0, 0.0, 0.0);
        traj.push_back(state);
        best_traj = traj;
    }
    return best_traj;
}

void DWAPlanner::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        ROS_WARN_STREAM("local_map_not_sub_count: " << local_map_not_sub_count);
        ROS_INFO("==========================================");
        double start = ros::Time::now().toSec();
        bool input_updated = false;

        if(!local_map_updated) local_map_not_sub_count++;

        if(USE_SCAN_AS_INPUT && scan_updated){
            input_updated = true;
        }else if(!USE_SCAN_AS_INPUT && local_map_not_sub_count < 2){
            input_updated = true;
        }
        if(input_updated && local_goal_subscribed && odom_updated && footprint_subscribed){
            Window dynamic_window = calc_dynamic_window(current_velocity);
            Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            ROS_WARN_STREAM("local goal: (" << goal[0] << " [m]," << goal[1] << " [m]," << goal[2]/M_PI*180 << " [deg])");

            geometry_msgs::Twist cmd_vel;
            double angle_to_goal = atan2(goal[1], goal[0]);
            if(goal.segment(0, 2).norm() > GOAL_THRESHOLD and (fabs(angle_to_goal) < ANGLE_TO_GOAL_TH)){
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
                if(USE_FOOTPRINT) predict_footprint_pub.publish(transform_footprint(best_traj.back()));
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
            if(!footprint_subscribed){
                ROS_WARN_THROTTLE(1.0, "Robot Footprint has not been updated");
            }
            geometry_msgs::Twist cmd_vel;
            velocity_pub.publish(cmd_vel);
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

float DWAPlanner::calc_speed_cost(const std::vector<State>& traj)
{
    float cost = fabs(TARGET_VELOCITY - fabs(traj[traj.size()-1].velocity));
    return cost;
}

float DWAPlanner::calc_obs_cost(const std::vector<State>& traj, const std::vector<std::vector<float>>& obs_list)
{
    float cost = 0.0;
    float min_dist = 1e3;
    int count = 0;
    for(const auto& state : traj){
        if(count%OBS_SEARCH_REDUCTION_RATE != 0){
            count++;
            continue;
        }
        for(const auto& obs : obs_list){
            float dist;
            if(USE_FOOTPRINT)
                dist = calc_dist_from_robot(obs, state);
            else
                dist = sqrt((state.x - obs[0])*(state.x - obs[0]) + (state.y - obs[1])*(state.y - obs[1]));

            if(dist <= local_map.info.resolution){
                cost = 1e6;
                return cost;
            }
            min_dist = std::min(min_dist, dist);
        }
        count++;
    }
    cost = 1.0 / min_dist;
    return cost;
}

void DWAPlanner::generate_trajectory(std::vector<State>& trajectory, const double velocity, const double yawrate)
{
        trajectory.clear();
        State state;
        for(float t=0; t<=PREDICT_TIME; t+=DT)
        {
            motion(state, velocity, yawrate);
            trajectory.push_back(state);
        }
}

void DWAPlanner::evaluate_trajectory(
        const std::vector<State>& trajectory,
        float& to_goal_cost,
        float& speed_cost,
        float& obs_cost,
        float& total_cost,
        const Eigen::Vector3d& goal,
        const std::vector<std::vector<float>>& obs_list)
{
    to_goal_cost = TO_GOAL_COST_GAIN * calc_to_goal_cost(trajectory, goal);
    speed_cost = SPEED_COST_GAIN * calc_speed_cost(trajectory);
    obs_cost = OBSTACLE_COST_GAIN * calc_obs_cost(trajectory, obs_list);
    total_cost = to_goal_cost + speed_cost + obs_cost;
}

geometry_msgs::Point DWAPlanner::calc_intersection(const std::vector<float>& obstacle, const State& state, geometry_msgs::PolygonStamped footprint)
{
    for (int i=0; i<footprint.polygon.points.size(); i++)
    {
        const Eigen::Vector3d vector_A(obstacle[0], obstacle[1], 0.0);
        const Eigen::Vector3d vector_B(state.x, state.y, 0.0);
        const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y, 0.0);
        Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
        if(i != footprint.polygon.points.size()-1)
            vector_D << footprint.polygon.points[i+1].x, footprint.polygon.points[i+1].y, 0.0;
        else
            vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;

        const double deno = (vector_B-vector_A).cross(vector_D-vector_C).z();
        const double s    = (vector_C-vector_A).cross(vector_D-vector_C).z()/deno;
        const double t    = (vector_B-vector_A).cross(vector_A-vector_C).z()/deno;

        geometry_msgs::Point point;
        point.x = vector_A.x() + s*(vector_B-vector_A).x();
        point.y = vector_A.y() + s*(vector_B-vector_A).y();

        // cross
        if(!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t))
            return point;

    }

    geometry_msgs::Point point;
    point.x = 1e6;
    point.y = 1e6;
    return point;
}

float DWAPlanner::calc_dist_from_robot(const std::vector<float>& obstacle, const State& state)
{
    const geometry_msgs::PolygonStamped footprint = transform_footprint(state);
    if(is_inside_of_robot(obstacle, footprint, state)){
        return 0.0;
    }else{
        geometry_msgs::Point intersection = calc_intersection(obstacle, state, footprint);
        return hypot((obstacle[0]-intersection.x),(obstacle[1]-intersection.y));
    }
}

geometry_msgs::PolygonStamped DWAPlanner::transform_footprint(const State& target_pose)
{
    geometry_msgs::PolygonStamped footprint = base_footprint;
    footprint.header.stamp = ros::Time::now();
    for(auto& point : footprint.polygon.points)
    {
        Eigen::VectorXf point_in(2);
        point_in << point.x, point.y;
        Eigen::Matrix2f rot;
        rot = Eigen::Rotation2Df(target_pose.yaw);
        const Eigen::VectorXf point_out = rot * point_in;

        point.x = point_out.x() + target_pose.x;
        point.y = point_out.y() + target_pose.y;
    }
    return footprint;
}

bool DWAPlanner::is_inside_of_robot(const std::vector<float>& obstacle, const geometry_msgs::PolygonStamped& footprint, const State& state)
{
    geometry_msgs::Point32 state_point;
    state_point.x = state.x;
    state_point.y = state.y;

    for(int i=0; i<footprint.polygon.points.size(); i++)
    {
        geometry_msgs::Polygon triangle;
        triangle.points.push_back(state_point);
        triangle.points.push_back(footprint.polygon.points[i]);

        if(i != footprint.polygon.points.size()-1)
            triangle.points.push_back(footprint.polygon.points[i+1]);
        else
            triangle.points.push_back(footprint.polygon.points[0]);

        if(is_inside_of_triangle(obstacle, triangle))
            return true;
    }

    return false;
}

bool DWAPlanner::is_inside_of_triangle(const std::vector<float>& target_point, const geometry_msgs::Polygon& triangle)
{
    if(triangle.points.size() != 3)
    {
        ROS_ERROR("Not triangle");
        exit(1);
    }

    const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
    const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
    const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
    const Eigen::Vector3d vector_P(target_point[0], target_point[1], 0.0);

    const Eigen::Vector3d vector_AB = vector_B - vector_A;
    const Eigen::Vector3d vector_BP = vector_P - vector_B;
    const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

    const Eigen::Vector3d vector_BC = vector_C - vector_B;
    const Eigen::Vector3d vector_CP = vector_P - vector_C;
    const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

    const Eigen::Vector3d vector_CA = vector_A - vector_C;
    const Eigen::Vector3d vector_AP = vector_P - vector_A;
    const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

    if((0<cross1.z() && 0<cross2.z() && 0<cross3.z()) || (cross1.z()<0 && cross2.z()<0 && cross3.z()<0))
        return true;
    else
        return false;
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
            const int i = int(floor((x - local_map.info.origin.position.x) / local_map.info.resolution));
            const int j = int(floor((y - local_map.info.origin.position.y) / local_map.info.resolution));
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
