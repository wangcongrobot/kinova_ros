/*
 * Receive target point from kinect and move arm to the target
 * and grasp
*/
#include "darknet_ros_msgs/TargetPoint.h"
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h"
#include "id_data_msgs/ID_Data.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <add_scene_objects.h> // handle scene obstacles
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Std C++ headers
#include <map>
#include <string>
#include <vector>

using namespace std;
#define PREGRASP_OFFSET 0.10;

geometry_msgs::PoseStamped goal_pose;
darknet_ros_msgs::TargetPoint canPoint;
bool receive_msg_flag = false;

void kinectCallback(const darknet_ros_msgs::TargetPoints::ConstPtr &msg) {
  //  TODO
  if (receive_msg_flag == false) {
    for (int i = 0; msg->target_points.size(); i++) {
      if (msg->target_points[i].Class == "can") {
        canPoint = msg->target_points[i];
        break;
      }
    }
    goal_pose.header.frame_id = "root";
    goal_pose.pose.position.x = canPoint.camera_x;
    goal_pose.pose.position.y = canPoint.camera_y + PREGRASP_OFFSET;
    goal_pose.pose.position.z = canPoint.camera_z;
    goal_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);
    receive_msg_flag = true;
    ROS_INFO_STREAM("Object postion: " << canPoint.camera_x << ", "
                                       << canPoint.camera_y << ", "
                                       << canPoint.camera_z);
  } 
  else
    return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualControl");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber subKinect =
      nh.subscribe("/darknet_ros/target_points", 100, kinectCallback);

  // if (atof(argv[2]) > -0.15) {
  //   ROS_INFO("Wrong posiiton, y value should be less then -0.15");
  //   return EXIT_FAILURE;
  // }

  // select group of joints
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroup group_arm("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group =
      group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // choose your preferred planner, the following planner works bad
  // group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPoseReferenceFrame("world");
  group_arm.setPoseTarget(goal_pose);
  ROS_INFO_STREAM("Object postion: " << goal_pose.pose.position.x << ", "
                                       << goal_pose.pose.position.y << ", "
                                       << goal_pose.pose.position.z);
  // the end-effector frame is declared in moveit config, can not be changed
  // group_arm.setEndEffectorLink("j2n6s300_link_6");

  // add collision objects to robot working space
  ros::NodeHandle n;
  ROS_INFO("Insert scene objects in workspace");
  build_workScene buildWorkScene(n);
  geometry_msgs::Pose tablePose, kinectBasePose, kinectBodyPose;

  tablePose.position.x = 0;
  tablePose.position.y = 0;
  tablePose.position.z = 0.1; // avoid collision between arm and surface
  buildWorkScene.add_boxModel("table", 1.5, 1.5, 0.01, tablePose);

  // add kinect stick obstacle
  kinectBasePose.position.x = 0.35;
  kinectBasePose.position.y = 0;
  kinectBasePose.position.z = 0.4;
  tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 1.57);
  // tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 0);
  // ROS_INFO_STREAM("Kinect obs pose: " << qkinect);
  kinectBasePose.orientation.x = qkinect[0];
  kinectBasePose.orientation.y = qkinect[1];
  kinectBasePose.orientation.z = qkinect[2];
  kinectBasePose.orientation.w = qkinect[3];
  buildWorkScene.add_boxModel("kinectBase", 0.6, 0.03, 0.02, kinectBasePose);

  // add kinect body obstacle
  kinectBodyPose.position.x = 0.35;  // 0.35 VS 0.3
  kinectBodyPose.position.y = -0.05; //-0.05 VS 0
  kinectBodyPose.position.z = 0.75;  // 0.75
  tf::Quaternion qbody = tf::createQuaternionFromRPY(0, 0, -1.57);
  // tf::Quaternion qbody = tf::createQuaternionFromRPY(0, -0.26, 0);
  // ROS_INFO_STREAM("Kinect obs pose: " << qbody);
  kinectBodyPose.orientation.x = qbody[0];
  kinectBodyPose.orientation.y = qbody[1];
  kinectBodyPose.orientation.z = qbody[2];
  kinectBodyPose.orientation.w = qbody[3];
  buildWorkScene.add_boxModel("kinectBody", 0.1, 0.3, 0.1, kinectBodyPose);

  // add collision objects to world
  planning_scene_interface.addCollisionObjects(
      buildWorkScene.collision_objects);

  ROS_INFO("Collision setup finished");

  ROS_INFO_STREAM("Planning to move " << group_arm.getEndEffectorLink()
                                      << " to a target pose expressed in "
                                      << group_arm.getPlanningFrame()
                                      << " with obstacles");

  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(0.7);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  // set maximum time to find a plan
  group_arm.setPlanningTime(5.0);
  bool success = group_arm.plan(my_plan);

  if (!success)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  /**************************stretch stage*******************/
  sleep(5);
  geometry_msgs::PoseStamped strech_pose = goal_pose;
  strech_pose.pose.position.y -= PREGRASP_OFFSET;
  strech_pose.pose.position.y += 0.03;
  strech_pose.pose.orientation.w = 1;
  strech_pose.pose.orientation.x = 0;
  strech_pose.pose.orientation.y = 0;
  strech_pose.pose.orientation.z = 0;
  group_arm.setPoseTarget(strech_pose);
  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(0.5);
  bool st_success = group_arm.plan(my_plan);

  if (!st_success)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan

  group_arm.move();

  spinner.stop();

  return EXIT_SUCCESS;
}
