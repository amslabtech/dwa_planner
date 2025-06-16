cmake_minimum_required(VERSION 3.8 FATAL_ERROR) 
project(dwa_planner) 


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()


find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2 REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED) 
find_package(Eigen3 REQUIRED COMPONENTS system) 


include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS}
)


add_library(dwa_planner_lib
  src/dwa_planner.cpp
  src/parameters.cpp
)

ament_target_dependencies(dwa_planner_lib
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs

)






add_executable(dwa_planner_node src/dwa_planner_node.cpp)
target_link_libraries(dwa_planner_node PRIVATE dwa_planner_lib)

# ament_target_dependencies(dwa_planner_node
#   rclcpp
#   target_link_libraries(dwa_planner_node PRIVATE dwa_planner_lib)
# )


install(TARGETS
  dwa_planner_lib
  dwa_planner_node
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY
  include/
  DESTINATION include 
)



ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3 
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(dwa_planner_test test/test_dwa_planner.cpp

  )
  ament_target_dependencies(dwa_planner_test
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
  )
  target_link_libraries(dwa_planner_test dwa_planner_lib)
  
endif()


ament_package()