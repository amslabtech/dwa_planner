#include <gtest/gtest.h>

#include <ros/ros.h>

#include "dwa_planner/dwa_planner.h"

TEST(TestSuite, test1)
{
    std::cout << "test calc dynamic window" << std::endl;
    DWAPlanner dwa;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    DWAPlanner::Window window = dwa.calc_dynamic_window(vel);
    EXPECT_LE(window.min_velocity, window.max_velocity);
    EXPECT_LE(window.min_yawrate, window.max_yawrate);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "dwa_planner_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(3.0).sleep();

    int r_e_t = RUN_ALL_TESTS();

    spinner.stop();

    ros::shutdown();

    return r_e_t;
}
