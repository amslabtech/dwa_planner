// Copyright 2020 amsl

#include "dwa_planner/dwa_planner.h"      

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<DWAPlanner>(rclcpp::NodeOptions());
  planner->process();
  rclcpp::shutdown();
  return 0;
}