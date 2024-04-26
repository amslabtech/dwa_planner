// Copyright 2020 amsl

#include "dwa_planner/dwa_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dwa_planner");
  DWAPlanner planner;
  planner.process();
  return 0;
}
