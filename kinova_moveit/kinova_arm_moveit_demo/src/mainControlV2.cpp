/*
*************************JDX pick and place chanllenge*****************
*************************Kinova arm test*******************************
This is the main program for handling picking and place task in JDX
challenge.
*/

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

// Std C++ headers
#include <iostream>
#include <map>
#include <string>
#include <vector>

// custom headers
#include "darknet_ros_msgs/TargetPoint.h" // kinect info
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include <add_scene_objects.h>    // handle scene obstacles
#include <pick_place.h>

// #include "parser.h"
// #include "wpi_jaco_msgs/EStop.h"

#define PREGRASP_OFFSET 0.2
#define POSITION_TOLERANCE 0.04
#define ORIENTATION_TOLERANCE 0.1
#define MAX_PARALLEL_ATTEMPTS 10
#define DEBUG true

using namespace std;
// globals
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
bool grasp_flag = true;             // alvin

geometry_msgs::Pose home_pose;
geometry_msgs::Pose scan_pose1;
geometry_msgs::Pose scan_pose2;
// geometry_msgs::Pose grasp_pose;
// geometry_msgs::Pose place_pose;
geometry_msgs::Pose pregrasp_low;
geometry_msgs::Pose pregrasp_high;
geometry_msgs::Pose pregrasp_mid;
geometry_msgs::Pose pregrasp_high_test;
geometry_msgs::Pose pregrasp_mid_test;
// geometry_msgs::Pose pregrasp;

// Pose used by Alvin
geometry_msgs::Pose grasp_pose; // Plan target from kinect --global variable
geometry_msgs::Pose place_pose; // pre-defined
geometry_msgs::Pose rest_pose;  // pre-defined, pose after object is grasped
geometry_msgs::Pose suck_pose;  //
geometry_msgs::Pose presuck_pose;
geometry_msgs::Pose pregrasp_pose;
geometry_msgs::Pose current_pose; // used to indicate arm state

/***************************NOTICE CLASS****************************/
class notice_pub_sub {
public:
  boost::function<void(const id_data_msgs::ID_Data::ConstPtr &)>
      notice_pub_sub_msgCallbackFun;

  notice_pub_sub();
  void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
  void notice_display(id_data_msgs::ID_Data notice_msg, bool set);
  void notice_sub_spinner(char set);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac;

private:
  ros::NodeHandle notice_handle;
  ros::Subscriber notice_subscriber;
  ros::Publisher notice_publisher;
  ros::SubscribeOptions notice_ops;
  ros::AsyncSpinner *notice_spinner;
  ros::CallbackQueue notice_callbackqueue;
  void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg);
  // ros::ServiceClient jaco_estop_client;
};

notice_pub_sub::notice_pub_sub() {
  notice_pub_sub_msgCallbackFun =
      boost::bind(&notice_pub_sub::notice_msgCallback, this, _1);
  notice_ops = ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
      "/notice", 10, notice_pub_sub_msgCallbackFun, ros::VoidPtr(),
      &notice_callbackqueue);
  notice_subscriber = notice_handle.subscribe(notice_ops);
  notice_spinner = new ros::AsyncSpinner(1, &notice_callbackqueue);

  notice_publisher =
      notice_handle.advertise<id_data_msgs::ID_Data>("/notice", 10);
  // jaco_estop_client = notice_handle.serviceClient<wpi_jaco_msgs::EStop>(
  //     "/jaco_arm/software_estop");
}

void notice_pub_sub::notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data) {
  notice_publisher.publish(id_data);
}

void notice_pub_sub::notice_display(id_data_msgs::ID_Data notice_msg,
                                    bool set) {

  if (set) {
    printf("REC Notice message,ID: %d,Data: ", notice_msg.id);
    for (char i = 0; i < 8; i++) {
      printf("%d ", notice_msg.data[i]);
      if (i == 7)
        printf("\n");
    }
  }
}
void notice_pub_sub::notice_msgCallback(
    const id_data_msgs::ID_Data::ConstPtr &notice_msg) {

  id_data_msgs::ID_Data notice_message;
  notice_message.id = 0;
  for (char i = 0; i < 8; i++)
    notice_message.data[i] = 0;

  notice_message.id = notice_msg->id;
  for (char i = 0; i < 8; i++)
    notice_message.data[i] = notice_msg->data[i];

  notice_pub_sub::notice_display(notice_message, true);

  /***** communication with hand *****/
  if (notice_message.id == 1 && notice_message.data[0] == 1) // hand close flag
  {
    close_hand_flag = true;
  }
  if (notice_message.id == 1 &&
      notice_message.data[0] == 3) // hand close flag soft
  {
    soft_close_hand_flag = true;
  }
  if (notice_message.id == 1 && notice_message.data[0] == 0) // hand open flag
  {
    open_hand_flag = true;
  }
  if (notice_message.id == 1 &&
      notice_message.data[0] == 4) // switch to suck mode
  {
    switch_suck_flag = true;
  }
  if (notice_message.id == 1 && notice_message.data[0] == 8) // suck flag
  {
    begin_suck_flag = true;
  }
  if (notice_message.id == 1 &&
      notice_message.data[0] == 9) // suck release flag
  {
    stop_suck_flag = true;
  }
  if (notice_message.id == 1 && notice_message.data[0] == 0) // hand open flag
  {
    open_hand_flag = true;
  }
  if (notice_message.id == 1 &&
      notice_message.data[0] == 14) // hand receive flag
  {
    hand_msg_rec_flag = true;
  }
  if (notice_message.id == 1 &&
      notice_message.data[0] == 2) // hand finished flag
  {
    hand_act_finished_flag = true;
  }

  // hand collision
  if (notice_message.id == 5 &&
      notice_message.data[0] == 14) // hand collision flag
  {
    if (hand_collision_start_flag) {
      ac->cancelGoal();
      hand_eStop_flag = true;
      ROS_ERROR("hand_eStop_flag received");
    }
  }

  /***** joy *****/
  if (notice_message.id == 3 && notice_message.data[0] == 1 &&
      notice_message.data[1] == 1) {
    joy_low_flag = true;
    ROS_INFO("joy_low_flag=true");
  }
  if (notice_message.id == 3 && notice_message.data[0] == 1 &&
      notice_message.data[1] == 2) {
    joy_mid_flag = true;
  }
  if (notice_message.id == 3 && notice_message.data[0] == 1 &&
      notice_message.data[1] == 3) {
    joy_high_flag = true;
  }

  /***** dashgo_act_finished_flag *****/
  if (notice_message.id == 2 && notice_message.data[0] == 8) {
    dashgo_act_finished_flag = true;
    ROS_WARN_STREAM("dashgo_act_finished_flag=true");
  }

  // joy distance
  if (notice_message.id == 3 && notice_message.data[0] == 27) {
    if (notice_message.data[1] > 0 && notice_message.data[1] <= 15 &&
        ((notice_message.data[1] + notice_message.data[1]) > 30)) {
      ROS_INFO("joy_left_flag=true");
      joy_left_flag = true;
    } else if (notice_message.data[2] > 0 && notice_message.data[2] <= 15 &&
               ((notice_message.data[1] + notice_message.data[1]) > 30)) {
      joy_right_flag = true;
      ROS_INFO("joy_right_flag=true");
    }

    else {
      joy_center_flag = true;
      ROS_INFO("joy_center_flag=true");
    }

    if (notice_message.data[5] >= 15) {
      grasp_by_dashgo_flag = true;
      grasp_by_dashgo_distance = notice_message.data[5] - 10; // deep left
      ROS_INFO("grasp_by_dashgo_distance");
    }

    joy_left_distance = (double)notice_message.data[1] * 0.01;  // left
    joy_right_distance = (double)notice_message.data[2] * 0.01; // right
    joy_up_distance = (double)notice_message.data[3] * 0.01;    // up
    joy_down_distance = (double)notice_message.data[4] * 0.01;  // down
    joy_deep_distance = (double)notice_message.data[5] * 0.01;  // deep

    ROS_INFO("kinect_Y");
  }

  /***** communication with Jaco arm *****/

  if (notice_message.id == 4 && notice_message.data[0] == 1 &&
      notice_message.data[1] == 2) {
    ROS_INFO("Receive command: grasp object");
    float x = notice_message.data[2] / 1000.0; // object pose, coordinate x
    float y = notice_message.data[3] / 1000.0; // object pose, coordinate y
    float z = notice_message.data[4] / 1000.0; // object pose, coordinate z
    ROS_INFO("receive pose form kinect %f, %f, %f", x, y, z);
    if (fabs(x) < 0.5 && y < -0.3) {
      ROS_INFO("Receive valid object position from kinect");
      grasp_pose.position.x = x;
      grasp_pose.position.y = y;
      grasp_pose.position.z = z;
      // kinect_target_valid = true;
      arm_start_fetch_flag = true;
      grasp_flag = true; // hard obj
    }
  } else if (notice_message.id == 4 && notice_message.data[0] == 1 &&
             notice_message.data[1] == 10) {
    ROS_INFO("Receive command: suck object");
    float x = notice_message.data[2] / 1000.0; // object pose, coordinate x
    float y = notice_message.data[3] / 1000.0; // object pose, coordinate y
    float z = notice_message.data[4] / 1000.0; // object pose, coordinate z
    if (abs(x) < 0.5 && y < -0.3) {
      ROS_INFO("Receive valid object position from kinect");
      suck_pose.position.x = x;
      suck_pose.position.y = y;
      suck_pose.position.z = z;
      // kinect_target_valid = true;
      arm_start_fetch_flag = true;
      grasp_flag = false; // soft obj
    }
  }

  if (notice_message.id == 4 &&
      notice_message.data[0] == 0) // main loop stop arm to fetch flag
  {
    arm_stop_fetch_flag = true;
  }
  if (notice_message.id == 4 &&
      notice_message.data[0] == 2) // main loop keep arm to fetch flag
  {
    arm_keep_fetch_flag = true;
  }
  if (notice_message.id == 4 &&
      notice_message.data[0] == 3) // main loop let arm release joy flag
  {
    arm_release_obj_flag = true;
  }
}

void notice_pub_sub::notice_sub_spinner(char set) {
  if (set == 1)
    notice_spinner->start();
  if (set == 0)
    notice_spinner->stop();
}

// TODO
void poseInit() {
  grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      1.57, -1.0, 0.0);         // grasp orientation
  tf::Quaternion q(0, 0, 0, 1); // suck orientation
  suck_pose.orientation.x = q.x();
  suck_pose.orientation.y = q.y();
  suck_pose.orientation.z = q.z();
  suck_pose.orientation.w = q.w();

  // rest pose
  rest_pose.orientation = grasp_pose.orientation;
  rest_pose.position.x = -0.3;
  rest_pose.position.y = -0.2;
  rest_pose.position.z = 0.6;

  //   place_pose; // pre-defined
  place_pose = rest_pose;
  place_pose.position.x = 0.1;
  place_pose.position.y = -0.4;
  place_pose.position.z = 0.6;

  //   rest_pose;  // pre-defined, pose after object is grasped
}

// function declaration
void notice_data_clear(id_data_msgs::ID_Data *test);

void moveToTarget(const geometry_msgs::Pose &target);
void moveToTarget(const std::string &target_name);
void moveToTarget(const geometry_msgs::PoseStamped &target);
void moveLineTarget(const geometry_msgs::Pose &start,
                   const geometry_msgs::Pose &goal);

int confirmToAct(const geometry_msgs::Pose &start,
                  const geometry_msgs::Pose &goal, const string & str = "NULL") {
  cout <<"\n" << "=================MOVE TO " + str + "==================" << "\n";
  ROS_INFO_STREAM("Move from: " << start << "to " << goal);
  ROS_INFO_STREAM("Confirm info and press n to execute plan");
  string pause_;
  cin >> pause_;
  if ("n" == pause_) {
    ROS_INFO_STREAM("Valid plan, begin to execute");
  } else {
    return 0;
  }
}

int confirmToAct(const geometry_msgs::Pose &goal, const string &str = "NULL") {
  cout << "\n" << "=================MOVE TO " + str + "==================" << "\n";  
  ROS_INFO_STREAM("Move to target" << goal);
  ROS_INFO_STREAM("Confirm info and press n to execute plan");
  string pause_;
  cin >> pause_;
  if ("n" == pause_) {
    ROS_INFO_STREAM("Valid plan, begin to execute");
  } else {
    return 0;
  }
}

// fill trajectory msg
// control_msgs::FollowJointTrajectoryGoal goal;
// Parser parser;

void handleCollisionObj(build_workScene &buildWorkScene) {

  //   buildWorkScene.clear_WorkScene("table");
  //   buildWorkScene.clear_WorkScene("kinectBody");
  //   buildWorkScene.clear_WorkScene("kinectObs");

  geometry_msgs::Pose tablePose, kinectBasePose, kinectBodyPose;

  tablePose.position.x = 0.4;
  tablePose.position.y = 0;
  tablePose.position.z = 0.1; // avoid collision between arm and surface
  buildWorkScene.add_boxModel("table", 1.0, 0.5, 0.01, tablePose);

  // add kinect stick obstacle
  kinectBasePose.position.x = 0.4;
  kinectBasePose.position.y = 0;
  kinectBasePose.position.z = 0.4;
  tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 1.57);
  // tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 0);
  // ROS_INFO_STREAM("Kinect obs pose: " << qkinect);
  kinectBasePose.orientation.x = qkinect[0];
  kinectBasePose.orientation.y = qkinect[1];
  kinectBasePose.orientation.z = qkinect[2];
  kinectBasePose.orientation.w = qkinect[3];
  buildWorkScene.add_boxModel("kinectBase", 0.8, 0.03, 0.02, kinectBasePose);

  // add kinect body obstacle
  kinectBodyPose.position.x = 0.4;   // 0.35 VS 0.3
  kinectBodyPose.position.y = -0.05; //-0.05 VS 0
  kinectBodyPose.position.z = 0.70;  // 0.75
  tf::Quaternion qbody = tf::createQuaternionFromRPY(0, 0, -1.57);
  // tf::Quaternion qbody = tf::createQuaternionFromRPY(0, -0.26, 0);
  // ROS_INFO_STREAM("Kinect obs pose: " << qbody);
  kinectBodyPose.orientation.x = qbody[0];
  kinectBodyPose.orientation.y = qbody[1];
  kinectBodyPose.orientation.z = qbody[2];
  kinectBodyPose.orientation.w = qbody[3];
  buildWorkScene.add_boxModel("kinectBody", 0.1, 0.3, 0.1, kinectBodyPose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jaco_moveit_control_main");
  ros::NodeHandle nh;
  kinova::PickPlace pick_place(nh);

  //   ros::Subscriber subFromkinect =
  //       nh.subscribe("/notice/targetPoint", 1, kinectCallback);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  notice_pub_sub notice_test; // initial a nitice class
  int loop_hz = 100;
  ros::Rate loop_rate(loop_hz);
  id_data_msgs::ID_Data notice_data; // initial notice data
  notice_data.id = 0;
  for (char i = 0; i < 8; i++)
    notice_data.data[i] = 0;

  poseInit(); // initial predifined poses

  /**********************ADD COLLISION***************************/
  ROS_INFO("Add collision objects  into the world (kinect and mobile base)");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  build_workScene buildWorkScene(nh);
  handleCollisionObj(buildWorkScene); // add objects
  // add collision objects to world
  planning_scene_interface.addCollisionObjects(
      buildWorkScene.collision_objects);
  ROS_INFO("Collision setup finished");

  /************************STAND BY POSE***************************/
  moveit::planning_interface::MoveGroup group("arm");
  // moveit::planning_interface::MoveGroup::Plan my_plan;

  // group.setGoalPositionTolerance(0.01);    // 3cm
  // group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
  // //   group.setPlannerId("RRTConnectkConfigDefault");
  // group.allowReplanning(true);
  // group.setPlanningTime(2.0);
  // group.setMaxVelocityScalingFactor(0.7); // TODO 0.3
  // group.setStartStateToCurrentState();
  // group.setNamedTarget("home");
  // group.move();

  /*************************ARM tASK LOOP***************/
  int loop_cnt = 0;
  string pose_name = "NO DEFINE";
  while (ros::ok()) {
    loop_cnt++;
    if (loop_cnt % 100 == 0) {
      ROS_INFO("Begin main loop, ready to receive command from ge_test");
      // notice main loop that received msg
      ROS_INFO_STREAM("arm_start_fetch flag: " << arm_start_fetch_flag);
    }

    if (arm_start_fetch_flag) {
      ROS_INFO("Start grasp objects from shelf/desk");
      notice_data_clear(&notice_data);
      notice_data.id = 4;
      notice_data.data[0] = 14;
      notice_test.notice_pub_sub_pulisher(notice_data);

      // // adjust mobile base position if necessary
      // int left = 0; // unit: cm
      // int right = 0;
      // // low --- fisrt floor
      // if (grasp_pose.position.z < 0.2) {
      //   if (grasp_pose.position.y < -0.85) // TODO 0.82
      //   {
      //     left = (int)((0.85 - grasp_pose.position.y) * 100);
      //     ROS_INFO_STREAM(mobile move left);
      //     notice_data.id = 2;
      //     notice_data.data[0] = 5;
      //     notice_data.data[1] = left; // -y direction in arm base frame
      //     notice_test.notice_pub_sub_pulisher(notice_data);
      //   } else {
      //     right = (int)((grasp_pose.position.y - 0.85) * 100);
      //     notice_data.id = 2;
      //     notice_data.data[0] = 6;
      //     notice_data.data[1] = right;
      //     notice_test.notice_pub_sub_pulisher(notice_data);
      //   }
      // }

      // // mid ---- desk
      // else if (grasp_pose.position.z >= 0.15 &&
      //          grasp_pose.position.z < 0.45) {
      //   if (grasp_pose.position.y < -0.8) // TODO 0.82
      //   {
      //     left = (int)((0.85 - grasp_pose.position.y) * 100);
      //     ROS_INFO_STREAM(mobile move left);
      //     notice_data.id = 2;
      //     notice_data.data[0] = 5;
      //     notice_data.data[1] = left; // -y direction in arm base frame
      //     notice_test.notice_pub_sub_pulisher(notice_data);
      //   } else {
      //     right = (int)((grasp_pose.position.y - 0.85) * 100);
      //     notice_data.id = 2;
      //     notice_data.data[0] = 6;
      //     notice_data.data[1] = right;
      //     notice_test.notice_pub_sub_pulisher(notice_data);
      //   }
      // }
      // // high -- second floor
      // else if (grasp_pose.position.z >= 0.45 &&
      //          grasp_pose.position.z < 0.65) {
      //   if (grasp_pose.position.y < -0.65) // TODO 0.82
      //   {
      //     left = (int)((0.85 - grasp_pose.position.y) * 100);
      //     ROS_INFO_STREAM(mobile move left);
      //     notice_data.id = 2;
      //     notice_data.data[0] = 5;
      //     notice_data.data[1] = left; // -y direction in arm base frame
      //     notice_test.notice_pub_sub_pulisher(notice_data);
      //   } else {
      //     right = (int)((grasp_pose.position.y - 0.85) * 100);
      //     notice_data.id = 2;
      //     notice_data.data[0] = 6;
      //     notice_data.data[1] = right;
      //     notice_test.notice_pub_sub_pulisher(notice_data);
      //   }
      // }

      /**********************GRASP START****************/
      if (grasp_flag) {
        pregrasp_pose = grasp_pose;
        pregrasp_pose.position.y += PREGRASP_OFFSET;
        // ROS_INFO_STREAM("pregrasp pose: " << pregrasp_pose);

        // 1. pregrasp
        pose_name = "PREGRASP POSE";
        confirmToAct(pregrasp_pose, pose_name);
        moveToTarget(pregrasp_pose); // plan to pre-grasp pose

        // 2. move forward
        geometry_msgs::Pose start;
        if (DEBUG) {
          start = pregrasp_pose; // virtual pose
        } else {
          start =  pick_place.get_ee_pose(); // real pose from driver info.
          start.orientation = pregrasp_pose.orientation;
        }
        geometry_msgs::Pose goal = grasp_pose;
        pose_name = "TARGET AND GRASP";
        confirmToAct(start, goal, pose_name);
        moveLineTarget(start, goal);

        // 3. close hand
        notice_data_clear(&notice_data);
        notice_data.id = 1;
        notice_data.data[0] = 3;
        notice_test.notice_pub_sub_pulisher(notice_data);
        ROS_INFO_ONCE("notice hand to close (1 1)");

        // data receive judge
        int wait_count = 0;
        while (ros::ok()) {

          if (hand_msg_rec_flag == true) // 1 14
          {
            hand_msg_rec_flag = false;
            break;
          }
          wait_count++;
          if (wait_count % 100 == 0) // send msg again after waiting 1s
          {
            ROS_ERROR("jaco didn't receive hand msg,Retrying...");
            notice_test.notice_pub_sub_pulisher(notice_data);
          }
          notice_test.notice_sub_spinner(1);
          loop_rate.sleep();
        }

        // wait for hand to finished
        while (ros::ok()) {
          ROS_INFO_ONCE("waiting for hand to close");
          if (hand_act_finished_flag) // 1 2
          {
            hand_act_finished_flag = false;
            break;
          }
          notice_test.notice_sub_spinner(1);
          loop_rate.sleep();
        }

        // 4. move to stand-by pose
        if (DEBUG) {
          start = grasp_pose;
        } else {
          start = pick_place.get_ee_pose();
          start.orientation = pregrasp_pose.orientation;
        }

        goal = pregrasp_pose;
        pose_name = "PREGRASP (BACK)";
        confirmToAct(start, goal,pose_name);
        moveLineTarget(start, goal);
        pose_name = "REST POSE";
        confirmToAct(pregrasp_pose, rest_pose, pose_name);
        moveToTarget(rest_pose);

      } else {
        // suck mode
        presuck_pose = suck_pose;
        presuck_pose.position.z += PREGRASP_OFFSET;
        presuck_pose.position.y += PREGRASP_OFFSET;

        // 1. presuck
        pose_name = "PRESUCK POSE";
        confirmToAct(presuck_pose, pose_name);
        moveToTarget(presuck_pose); // plan to pre-grasp pose

        // 2. move forward and downside
        geometry_msgs::Pose start;
        if (DEBUG) {
          start = presuck_pose;
        } else {
          start = pick_place.get_ee_pose();
          start.orientation = presuck_pose.orientation;
        }
        geometry_msgs::Pose goal = presuck_pose;
        goal.position.y -= PREGRASP_OFFSET;
        pose_name = "SUCK FORWARD";
        confirmToAct(start, goal, pose_name);
        moveLineTarget(start, goal); // forward

        if (DEBUG) {
          start = goal;
        } else {
          geometry_msgs::Pose start = pick_place.get_ee_pose();
          start.orientation = presuck_pose.orientation;
        }
        goal = suck_pose;
        pose_name = "SUCK DOWN";
        confirmToAct(start, goal, pose_name);
        moveLineTarget(start, goal); // down

        // 3.swich to suck mode
        notice_data.id = 1;
        notice_data.data[0] = 4;
        notice_test.notice_pub_sub_pulisher(notice_data);
        ROS_INFO_ONCE("notice hand to close (1 1)");

        // wait switch finish
        while (ros::ok()) {
          ROS_INFO_ONCE("waiting switch to suck");
          if (hand_act_finished_flag) // 1 2
          {
            hand_act_finished_flag = false;
            break;
          }
          notice_test.notice_sub_spinner(1);
          loop_rate.sleep();
        }
        // suck
        notice_data_clear(&notice_data);
        notice_data.id = 1;
        notice_data.data[0] = 8;
        notice_test.notice_pub_sub_pulisher(notice_data);
        ROS_INFO_ONCE("notice hand to suck (1 1)");

        // data receive judge
        int wait_count = 0;
        while (ros::ok()) {

          if (hand_msg_rec_flag == true) // 1 14
          {
            hand_msg_rec_flag = false;
            break;
          }
          wait_count++;
          if (wait_count % 4 == 0) // send msg again after waiting 1s
          {
            ROS_ERROR("jaco didn't receive hand msg,Retrying...");
            notice_test.notice_pub_sub_pulisher(notice_data);
          }
          notice_test.notice_sub_spinner(1);
          loop_rate.sleep();
        }

        // wait for hand to finished
        while (ros::ok()) {
          ROS_INFO_ONCE("waiting for hand to close");
          if (hand_act_finished_flag) // 1 2
          {
            hand_act_finished_flag = false;
            break;
          }
          notice_test.notice_sub_spinner(1);
          loop_rate.sleep();
        }

        // 4. move to stand-by pose
        if (DEBUG) {
          start = presuck_pose;
        } else {
          start = pick_place.get_ee_pose();
          start.orientation = presuck_pose.orientation;
        }
        goal = suck_pose;
        goal.position.z += PREGRASP_OFFSET;
        pose_name = "SUCK UP BACK";
        confirmToAct(start, goal, pose_name);
        moveLineTarget(start, goal); // up

        if (DEBUG) {
          start = goal; // last goal
        } else {
          start = pick_place.get_ee_pose();
          start.orientation = presuck_pose.orientation;
        }
        goal = presuck_pose;
        pose_name = "SUCK BACKWARD BACK";
        confirmToAct(start, goal, pose_name);
        moveLineTarget(start, goal); // back
        pose_name = "SUCK REST POSE";
        confirmToAct(presuck_pose, rest_pose);
        moveToTarget(rest_pose);
      }

      // notice main loop that fetch action finished
      notice_data_clear(&notice_data);
      notice_data.id = 4;
      notice_data.data[0] = 15;
      notice_test.notice_pub_sub_pulisher(notice_data);
      arm_start_fetch_flag = false;
      ROS_INFO_ONCE("grasp task finished");
    }

    // wait for main loop to keep pose
    // while (ros::ok()) {
    //   ROS_INFO_ONCE("waiting for main loop to start arm to keep pose");
    //   if (arm_keep_fetch_flag) // 4 2
    //   {
    //     arm_keep_fetch_flag = false;
    //     break;
    //   }
    //   notice_test.notice_sub_spinner(1);
    //   loop_rate.sleep();
    // }

    if (arm_keep_fetch_flag) {
      notice_data_clear(&notice_data);
      notice_data.id = 4;
      notice_data.data[0] = 14;
      notice_test.notice_pub_sub_pulisher(notice_data);
      ROS_INFO("keep task begin");

      notice_data_clear(&notice_data);
      notice_data.id = 4;
      notice_data.data[0] = 15;
      notice_test.notice_pub_sub_pulisher(notice_data);
      arm_keep_fetch_flag = false;
      ROS_INFO("keep task finish");
    }

    if (arm_release_obj_flag) {

      // notice main loop that received msg
      notice_data_clear(&notice_data);
      notice_data.id = 4;
      notice_data.data[0] = 14;
      notice_test.notice_pub_sub_pulisher(notice_data);
      ROS_ERROR("---------- 4 14");

      // 1. place
      pose_name = "PLACE POSE";
      confirmToAct(place_pose, pose_name);
      moveToTarget(place_pose);

      // 2. open hand
      notice_data_clear(&notice_data);
      if (grasp_flag) {
        notice_data.id = 1;
        notice_data.data[0] = 0;
        notice_test.notice_pub_sub_pulisher(notice_data);
      } else {
        notice_data.id = 1;
        notice_data.data[0] = 9;
        notice_test.notice_pub_sub_pulisher(notice_data);
      }
      // hand data receive judge
      int wait_count = 0;
      while (ros::ok()) {

        if (hand_msg_rec_flag == true) {
          hand_msg_rec_flag = false;
          break;
        }

        wait_count++;
        if (wait_count % 100 == 0) // send msg again after waiting 1s
        {
          ROS_ERROR("jaco didn't receive msg,Retrying...");
          notice_test.notice_pub_sub_pulisher(notice_data);
        }
        // if (wait_count >= 3) {
        //   wait_count = 0;
        //   break;
        // }
        notice_test.notice_sub_spinner(1);
        loop_rate.sleep();
      }
      // wait for hand to finished
      wait_count = 0;
      while (ros::ok()) {
        if (wait_count % 100 == 0) {
          ROS_INFO("waiting for hand to open/release");
        }
        if (hand_act_finished_flag) // 1 2
        {
          hand_act_finished_flag = false;
          break;
        }
        notice_test.notice_sub_spinner(1);
        loop_rate.sleep();
      }

      // notice main loop that place action finished
      notice_data_clear(&notice_data);
      notice_data.id = 4;
      notice_data.data[0] = 15;
      notice_test.notice_pub_sub_pulisher(notice_data);
      ROS_INFO("placing action finished\n");

      // back to rest pose
      pose_name = "REST POSE";
      confirmToAct(place_pose, rest_pose, pose_name);
      moveToTarget(rest_pose);
      arm_release_obj_flag = false;
      // sleep(10); // TODO change time to wait
    }
    notice_test.notice_sub_spinner(1);
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}

void notice_data_clear(id_data_msgs::ID_Data *test) {
  test->id = 0;
  for (int i = 0; i < 8; i++)
    test->data[i] = 0;
}

int hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data *notice_data_test,
                                    bool *msg_rec_flag, bool *finished_flag,
                                    notice_pub_sub *notice_test) {
  id_data_msgs::ID_Data notice_data;
  int loop_hz = 10;
  ros::Rate loop_rate(loop_hz);

  notice_data_clear(&notice_data);
  notice_data.id = notice_data_test->id;
  for (int i = 0; i < 8; i++)
    notice_data.data[i] = notice_data_test->data[i];
  notice_test->notice_pub_sub_pulisher(notice_data);
  // hand data receive judge
  int wait_count = 0;
  while (ros::ok()) {

    if (*msg_rec_flag == true) {
      *msg_rec_flag = false;
      break;
    }

    wait_count++;
    if (wait_count % 5 == 0) // send msg again after waiting 1s
    {
      switch (notice_data.id) {
      case 1:
        ROS_ERROR("Hand didn't receive msg,Retrying...");
        break;
      case 2:
        ROS_ERROR("Dashgo didn't receive msg,Retrying...");
        break;
      case 3:
        ROS_ERROR("Kinect didn't receive msg,Retrying...");
        break;
      case 4:
        ROS_ERROR("Kinova arm didn't receive msg,Retrying...");
        break;
      default:
        break;
      }
      notice_test->notice_pub_sub_pulisher(notice_data);
    }

    if (wait_count >= 10) {
      error_no = notice_data.id;
      wait_count = 0;
      goto next;
    }
    notice_test->notice_sub_spinner(1);
    loop_rate.sleep();
  }
  // hand action finish judge
  while (ros::ok()) {
    if (*finished_flag == true) {
      *finished_flag = false;
      break;
    }

    notice_test->notice_sub_spinner(1);
    loop_rate.sleep();
  }

next:
  return error_no;
}

void moveToTarget(const geometry_msgs::PoseStamped &target) {
  moveit::planning_interface::MoveGroup group("arm");
  group.setGoalPositionTolerance(0.01);    // 3cm
  group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
  //   group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(1.0);
  group.setMaxVelocityScalingFactor(0.5);
  group.setNumPlanningAttempts(1);

  group.setStartStateToCurrentState();
  group.setPoseTarget(target);

  moveit::planning_interface::MoveGroup::Plan plan, temp_plan;
  int loops = 10;
  bool success = false;

  ros::Duration best_time(100.0);
  ros::Duration current_time(0.0);

  for (int i = 0; i < loops; i++) {
    bool suc = group.plan(temp_plan);
    if (suc) {
      success = true;
      current_time =
          temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
      if (current_time < best_time) {
        plan = temp_plan;
        best_time = current_time;
        ROS_INFO_STREAM(current_time);
      }
    }
  }
  group.execute(plan);
}

// Update--Alvin
void moveToTarget(const geometry_msgs::Pose &target) {
  // ROS_INFO_STREAM("Move to : " << target.position.x << ", " << target.position.y
  //                              << ", " << target.position.z << ", "
  //                              << target.orientation.x << ", "
  //                              << target.orientation.y);
  moveit::planning_interface::MoveGroup group("arm");
  group.setGoalPositionTolerance(0.01);    // 3cm
  group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
  group.setMaxVelocityScalingFactor(0.5);
  // group.setNumPlanningAttempts(1);
  group.setStartStateToCurrentState();
  group.setPoseTarget(target);

  ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink()
                                      << " to a target pose expressed in "
                                      << group.getPlanningFrame()
                                      << " with obstacles");

  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlanningTime(3.0);

  int loops = 10;
  // bool success = false;

  bool success = group.plan(my_plan);

  for (int i = 0; i < loops; i++) {
    success = group.plan(my_plan);
    if (success) {
      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
      break;
    } else {
      ROS_INFO("Plan failed at try: %d", i);
      // continue;
    }
  }

  if (!success)
    ROS_INFO("No plan found after 10 tries");
  // Execute the plan
  ros::Time start = ros::Time::now();
  group.execute(my_plan);
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}

void moveToTarget(const std::string &target_name) {
  moveit::planning_interface::MoveGroup group("arm");
  group.setGoalPositionTolerance(0.01);    // 3cm
  group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(1.0);
  group.setMaxVelocityScalingFactor(0.5);
  group.setNumPlanningAttempts(1);

  group.setStartStateToCurrentState();
  group.setNamedTarget(target_name);

  moveit::planning_interface::MoveGroup::Plan plan, temp_plan;
  int loops = 10;
  bool success = false;

  ros::Duration best_time(100.0);
  ros::Duration current_time(0.0);

  for (int i = 0; i < loops; i++) {
    bool suc = group.plan(temp_plan);
    if (suc) {
      success = true;
      current_time =
          temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
      if (current_time < best_time) {
        plan = temp_plan;
        best_time = current_time;
        ROS_INFO_STREAM(current_time);
      }
    }
  }
  group.execute(plan);
}

void moveLineTarget(const geometry_msgs::Pose &start,
                   const geometry_msgs::Pose &goal) {
  ROS_INFO("Begin cartesian line plan");
  // ROS_INFO_STREAM("Print n to excute the straight line plan");
  // string pause_;
  // cin >> pause_;
  // if ("n" == pause_) {
  //   ROS_INFO("READY..............");
  // } else {
  //   return 0;
  // }

  // Cartesian Path
  geometry_msgs::Pose way_pose = start;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(way_pose); // first pose waypoint
  int num_waypoint = 5;
  float delta_x = (goal.position.x - start.position.x) / (num_waypoint - 1);
  float delta_y = (goal.position.y - start.position.y) / (num_waypoint - 1);
  float delta_z = (goal.position.z - start.position.z) / (num_waypoint - 1);

  // interplotate between current pose and target pose
  for (int i = 0; i < num_waypoint - 1; i++) {
    way_pose.position.x += delta_x;
    way_pose.position.y += delta_y;
    way_pose.position.z += delta_z;
    waypoints.push_back(way_pose);
  }

  moveit::planning_interface::MoveGroup group("arm");
  group.setGoalPositionTolerance(0.01);    // 3cm
  group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
  group.setMaxVelocityScalingFactor(0.5);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = group.computeCartesianPath(waypoints, eef_step,
                                               jump_threshold, trajectory);

  moveit::planning_interface::MoveGroup::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  group.execute(cartesian_plan);
}
