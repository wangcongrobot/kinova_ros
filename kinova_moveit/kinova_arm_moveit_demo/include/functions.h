/*
=========================JDX pick and place chanllenge===============
==========================Kinova arm test============================
This is the main program for handling picking and place task in JDX
challenge.

Move some functions to this file
*/

#ifndef FUCTIONS_H
#define FUCTIONS_H

// Std C++ headers
#include <iostream>
#include <map>
#include <string>
#include <vector>

// ros head files
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "moveit_msgs/ExecuteTrajectoryAction.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>

// custom headers
#include "darknet_ros_msgs/TargetPoint.h" // kinect info
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include <add_scene_objects.h>    // handle scene obstacles
#include <pick_place.h>

#include "parser.h"
#include "globals.h"
#include "notice_pub_sub.h"

typedef int ErrorCode;
using namespace std;


// function declaration
void notice_data_clear(id_data_msgs::ID_Data* test);
void poseInit();
void moveToTarget(const geometry_msgs::Pose& target);
void moveToTarget(const std::string& target_name);
void moveToTarget(const geometry_msgs::PoseStamped& target);
void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
void error_deal(int error_nu);
void confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const string& str);
void confirmToAct(const geometry_msgs::Pose& goal, const string& str);
void handleCollisionObj(build_workScene& buildWorkScene);
void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
ErrorCode hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
    bool* msg_rec_flag, bool* finished_flag, notice_pub_sub* notice_test, string task);
int evaluateMoveitPlan(moveit::planning_interface::MoveGroup::Plan& plan);

void confirmToAct();

void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ,
                              double roll, double pitch, double yaw,
                              int number_point, int number_distance);


#endif // FUNCTIONS_H
