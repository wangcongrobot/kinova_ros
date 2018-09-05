/*
=====================MoveIt plan sence interface===============
Test moveit plan with obstacles in working space.
For JDX task, the obstacles include: kinect in the rear of the arm,
AGV mobile plantform beneath of the robot arm.
*/
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_with_obstacle");

  if (argc < 7) {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun Jaco moveit plan_arm_ik  x y z  r p y");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of "
             "/arm_tool_link expressed in world frame");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  //   define the target pose
  // default validate pose: -0.2 -0.4 0.5  1.57 -1.57 0
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "root";
  goal_pose.pose.position.x = atof(argv[1]);
  goal_pose.pose.position.y = atof(argv[2]);
  goal_pose.pose.position.z = atof(argv[3]);
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      atof(argv[4]), atof(argv[5]), atof(argv[6]));

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // select group of joints
  moveit::planning_interface::MoveGroup group_arm("arm");
  // choose your preferred planner, the following planner works bad
  // group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPoseReferenceFrame("world");
  group_arm.setPoseTarget(goal_pose);
  // the end-effector frame is declared in moveit config, can not be changed
  // group_arm.setEndEffectorLink("j2n6s300_link_6");

  // add collision objects to robot working space
  ros::NodeHandle n;
  ROS_INFO("Insert scene objects in workspace");
  build_workScene buildWorkScene(n);
  geometry_msgs::Pose tablePose, kinectBasePose, kinectBodyPose;

  tablePose.position.x = 0;
  tablePose.position.y = 0;
  tablePose.position.z = -0.03 / 2.0;
  buildWorkScene.add_boxModel("table", 1.5, 1.5, 0.03, tablePose);

  // add kinect stick obstacle
  kinectBasePose.position.x = 0.4;
  kinectBasePose.position.y = 0;
  kinectBasePose.position.z = 0.3;
  tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 1.57);
  //   ROS_INFO_STREAM("Kinect obs pose: " << qkinect);
  kinectBasePose.orientation.x = qkinect[0];
  kinectBasePose.orientation.y = qkinect[1];
  kinectBasePose.orientation.z = qkinect[2];
  kinectBasePose.orientation.w = qkinect[3];
  buildWorkScene.add_boxModel("kinectBase", 0.6, 0.05, 0.02, kinectBasePose);

  // add kinect body obstacle
  kinectBodyPose.position.x = 0.40;
  kinectBodyPose.position.y = -0.05;
  kinectBodyPose.position.z = 0.65;
  tf::Quaternion qbody = tf::createQuaternionFromRPY(0, 0, -1.57);
  //   ROS_INFO_STREAM("Kinect obs pose: " << qbody);
  kinectBodyPose.orientation.x = qbody[0];
  kinectBodyPose.orientation.y = qbody[1];
  kinectBodyPose.orientation.z = qbody[2];
  kinectBodyPose.orientation.w = qbody[3];
  buildWorkScene.add_boxModel("kinectBody", 0.1, 0.3, 0.1, kinectBodyPose);

  ROS_INFO("Collision setup finished");

  ROS_INFO_STREAM("Planning to move " << group_arm.getEndEffectorLink()
                                      << " to a target pose expressed in "
                                      << group_arm.getPlanningFrame()
                                      << "with obstacles");

  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(0.5);

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

  spinner.stop();

  return EXIT_SUCCESS;
}