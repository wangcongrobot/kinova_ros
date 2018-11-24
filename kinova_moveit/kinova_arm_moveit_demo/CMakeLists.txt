cmake_minimum_required(VERSION 2.8.3)
project(kinova_arm_moveit_demo)

add_compile_options(-std=c++11)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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


generate_messages(
   DEPENDENCIES
   std_msgs
 )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)
find_package(kinova_driver REQUIRED)

# Add .msg, .srv, .action files if necessary
# add_message_files(...)
# add_service_files(...)
# add_action_files(...)
# generate_messages(...)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    message_runtime
)


###########
## Build ##
###########
include_directories(
  include
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

add_library(add_scene_objects src/add_scene_objects.cpp)
target_link_libraries(add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})


# add_executable(mainControl src/mainControl.cpp src/pick_place.cpp)
# target_link_libraries(mainControl add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
# install(TARGETS  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mainControl-sweep 
              src/mainControl-sweep.cpp
              src/functions.cpp)
target_link_libraries(mainControl-sweep add_scene_objects jrc18sia_motion_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



# python executable scripts
# install(PROGRAMS
#   scripts/joint_control.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
