#include "globals.h"

int error_no = 0;
bool kinect_target_valid = true;

bool dashgo_act_finished_flag = false;
bool hand_eStop_flag = false;
bool hand_collision_left_flag = false;
bool hand_collision_right_flag = false;
bool hand_collision_start_flag = false;

// kinect distance
double joy_up_distance = 0;
double joy_down_distance = 0;
double joy_left_distance = 0;
double joy_right_distance = 0;
double joy_deep_distance = 0;
double bin_center = 0;
int grasp_by_dashgo_distance = 0;

bool joy_low_flag = false;
bool joy_mid_flag = false;
bool joy_high_flag = false;
bool joy_left_flag = false;
bool joy_right_flag = false;
bool joy_center_flag = false;

bool grasp_by_dashgo_flag = false;

// hand globals
bool close_hand_flag = false;
bool open_hand_flag = false;
bool hand_msg_rec_flag = false;
bool hand_act_finished_flag = false;
bool kinect_rec_flag = false;

// alivn
bool soft_close_hand_flag = false;
bool switch_suck_flag = false;
bool begin_suck_flag = false;
bool stop_suck_flag = false;

// main loop globals: arm control section,id=4
bool arm_start_fetch_flag = false;  // data[0]=1
bool arm_stop_fetch_flag = false;   // data[0]=0
bool arm_keep_fetch_flag = false;   // data[0]=2
bool arm_release_obj_flag = false;  // data[0]=3
bool arm_msg_rec_flag = false;      // data[0]=14
bool arm_act_finished_flag = false; // data[0]=15
bool use_gripper_flag = true;       // alvin, grasp or suck depend on target

std::vector<double> current_joint_values;

// Pose used by Alvin
geometry_msgs::Pose grasp_pose; // Plan target from kinect --global variable
geometry_msgs::Pose pregrasp_pose;
geometry_msgs::Pose gripper_rest_pose;  // pre-defined, pose after object is grasped
geometry_msgs::Pose gripper_place_pose; // pre-defined

geometry_msgs::Pose suck_pose; //
geometry_msgs::Pose presuck_pose;
geometry_msgs::Pose sucker_place_pose; // pre-defined
geometry_msgs::Pose sucker_rest_pose;  // pre-defined, pose after object is grasped

geometry_msgs::Pose current_pose; // used to indicate arm state
std::string hand_current_mode = "grasp";