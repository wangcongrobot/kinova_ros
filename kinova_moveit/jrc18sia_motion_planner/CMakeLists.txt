cmake_minimum_required(VERSION 2.8.3)
project(jrc18sia_motion_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  id_data_msgs
  roscpp
  rospy
	message_generation
  std_msgs
  actionlib
)

find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface   
             pluginlib
             cmake_modules
             geometric_shapes
             id_data_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    message_runtime
)

include_directories(
  include jrc18sia_motion_planner
  ${catkin_INCLUDE_DIRS}
  include ${Eigen_INCLUDE_DIRS}
)

# Header files direcotory
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
# include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kinova_driver_INCLUDE_DIRS})

# Library files directory
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library : custom DH kinematics
add_library(jrc18sia_kinematics_parser src/jrc18sia_kinematics_parser.cpp)
target_link_libraries(jrc18sia_kinematics_parser ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a C++ library : motion planner
add_library(jrc18sia_motion_planner src/jrc18sia_motion_planner.cpp)
target_link_libraries(jrc18sia_motion_planner jrc18sia_kinematics_parser ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(jrc18sia_motion_planner_test src/jrc18sia_motion_planner_test)
target_link_libraries(jrc18sia_motion_planner_test 
                      jrc18sia_motion_planner
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})