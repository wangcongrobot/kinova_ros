#ifndef GLOBALS_H
#define GLOBALS_H


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

#include "globals.h"


#define PREGRASP_OFFSET 0.2
#define PRESUCK_OFFSET_LOW 0.1
#define POSITION_TOLERANCE 0.04
#define ORIENTATION_TOLERANCE 0.1
#define MAX_PARALLEL_ATTEMPTS 10
#define DEBUG false
#define CONFIRM_ACT false
#define MAX_PLAN_STEPS 25

// globals
extern int error_no;
extern bool kinect_target_valid;

extern bool dashgo_act_finished_flag;
extern bool hand_eStop_flag;
extern bool hand_collision_left_flag;
extern bool hand_collision_right_flag;
extern bool hand_collision_start_flag;

// kinect distance
extern double joy_up_distance;
extern double joy_down_distance;
extern double joy_left_distance;
extern double joy_right_distance;
extern double joy_deep_distance;
extern double bin_center;
extern int grasp_by_dashgo_distance;

extern bool joy_low_flag;
extern bool joy_mid_flag;
extern bool joy_high_flag;
extern bool joy_left_flag;
extern bool joy_right_flag;
extern bool joy_center_flag;

extern bool grasp_by_dashgo_flag;

// hand globals
extern bool close_hand_flag;
extern bool open_hand_flag;
extern bool hand_msg_rec_flag;
extern bool hand_act_finished_flag;
extern bool kinect_rec_flag;

// alivn
extern bool soft_close_hand_flag;
extern bool switch_suck_flag;
extern bool begin_suck_flag;
extern bool stop_suck_flag;

// main loop globals: arm control section,id=4
extern bool arm_start_fetch_flag;  // data[0]=1
extern bool arm_stop_fetch_flag;   // data[0]=0
extern bool arm_keep_fetch_flag;   // data[0]=2
extern bool arm_release_obj_flag;  // data[0]=3
extern bool arm_msg_rec_flag;      // data[0]=14
extern bool arm_act_finished_flag; // data[0]=15
extern bool use_gripper_flag;       // alvin, grasp or suck depend on target

// geometry_msgs::Pose home_pose;
// geometry_msgs::Pose scan_pose1;
// geometry_msgs::Pose scan_pose2;
// geometry_msgs::Pose grasp_pose;
// geometry_msgs::Pose place_pose;
// geometry_msgs::Pose pregrasp_low;
// geometry_msgs::Pose pregrasp_high;
// geometry_msgs::Pose pregrasp_mid;
// geometry_msgs::Pose pregrasp_high_test;
// geometry_msgs::Pose pregrasp_mid_test;

// Pose used by Alvin
// Grasp
extern geometry_msgs::Pose pregrasp_pose;
extern geometry_msgs::Pose grasp_pose; // Plan target from kinect --global variable
extern geometry_msgs::Pose postgrasp_pose;
extern geometry_msgs::Pose gripper_rest_pose;  // pre-defined, pose after object is grasped
extern geometry_msgs::Pose gripper_place_pose; // pre-defined

// Suck
extern geometry_msgs::Pose presuck_pose;
extern geometry_msgs::Pose suck_pose; //
extern geometry_msgs::Pose postsuck_pose;
extern geometry_msgs::Pose sucker_place_pose; // pre-defined
extern geometry_msgs::Pose sucker_rest_pose;  // pre-defined, pose after object is grasped


// Move
extern geometry_msgs::Pose premove_pose;
extern geometry_msgs::Pose move_pose; //
extern geometry_msgs::Pose postmove_pose;

extern geometry_msgs::Pose test_pose;


extern geometry_msgs::Pose current_pose; // used to indicate arm state


extern std::string hand_current_mode;
extern std::vector<double> current_joint_values;



#endif