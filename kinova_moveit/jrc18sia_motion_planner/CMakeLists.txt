cmake_minimum_required(VERSION 2.8.3)
project(jrc18sia_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  moveit_ros_planning
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jrc18sia_motion_planner
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include jrc18sia_motion_planner
  ${catkin_INCLUDE_DIRS}
  include ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library : motion planner
add_library(jrc18sia_motion_planner src/jrc18sia_motion_planner.cpp)
target_link_libraries(jrc18sia_motion_planner jrc18sia_kinematics_parser ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## motion planner test
add_executable(jrc18sia_motion_planner_test src/jrc18sia_motion_planner_test)
target_link_libraries(jrc18sia_motion_planner_test 
                      jrc18sia_motion_planner
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})

