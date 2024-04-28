cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwa_planner_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(dwa_planner_lib
  src/dwa_planner.cpp
  src/parameters.cpp
)
add_dependencies(dwa_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(dwa_planner src/dwa_planner_node.cpp)
target_link_libraries(dwa_planner
  ${catkin_LIBRARIES}
  dwa_planner_lib
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
endif()
