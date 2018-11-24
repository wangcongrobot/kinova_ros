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

add_executable(motion_plan src/motion_plan.cpp)
target_link_libraries(motion_plan ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS motion_plan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(work_scene src/work_scene.cpp)
target_link_libraries(work_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS work_scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(pick_place src/pick_place.cpp include/pick_place.h)
#target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
#install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_accuracy src/test_accuracy.cpp )
target_link_libraries(test_accuracy ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS test_accuracy DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cartesianPlan src/cartesianPlan.cpp )
target_link_libraries(cartesianPlan ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS cartesianPlan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(add_scene_objects src/add_scene_objects.cpp)
target_link_libraries(add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})

#test example added by hekai
add_executable(basicControl src/basicControl.cpp src/pick_place.cpp )
target_link_libraries(basicControl add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS basicControl DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

add_executable(planWithObstacle src/planWithObstacle.cpp)
target_link_libraries(planWithObstacle add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS planWithObstacle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(visualControl 
               src/visualControl.cpp 
               src/pick_place.cpp
               src/parser.cpp)
target_link_libraries(visualControl add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS planWithObstacle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mainControl src/mainControl.cpp src/pick_place.cpp)
target_link_libraries(mainControl add_scene_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS planWithObstacle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# wangcong
add_executable(action_control 
               src/action_control.cpp 
               src/pick_place.cpp
               src/parser.cpp)
target_link_libraries(action_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS planWithObstacle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# wangcong
add_executable(mainControl-wangcong 
              src/mainControl-wangcong.cpp 
              src/pick_place.cpp
              src/functions.cpp
              src/globals.cpp
              src/notice_pub_sub.cpp
              src/jrc18sia_motion_planner.cpp)
target_link_libraries(mainControl-wangcong 
                      add_scene_objects 
                      parser 
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS planWithObstacle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(parser src/parser.cpp)
target_link_libraries(parser ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})



# python executable scripts
# install(PROGRAMS
#   scripts/joint_control.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
