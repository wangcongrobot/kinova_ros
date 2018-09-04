/*
=====================MoveIt plan sence interface===============
Test moveit plan with obstacles in working space.
For JDX task, the obstacles include: kinect in the rear of the arm,
AGV mobile plantform beneath of the robot arm.
*/

#include "id_data_msgs/ID_Data.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
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

void addCollisionObjects(const moveit_msgs::CollisionObject *co_,
                         const char *name, float length, float width,
                         float height, geometry_msgs::Pose prePose) {

  co_.header.frame_id = "root";
  co_.header.stamp = ros::Time::now();

  co_.id = name;
  ROS_INFO_STREAM("Add collision object: " << co_.id);

  co_.primitives.resize(1);
  co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<
                                      shape_msgs::SolidPrimitive::BOX>::value);
  co_.operation = moveit_msgs::CollisionObject::ADD;

  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = length;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;

  co_.primitive_poses.resize(1);
  co_.primitive_poses[0].position.x = prePose.position.x;
  co_.primitive_poses[0].position.y = prePose.position.y;
  co_.primitive_poses[0].position.z = prePose.position.z;
  co_.primitive_poses[0].orientation.x = prePose.orientation.x;
  co_.primitive_poses[0].orientation.y = prePose.orientation.y;
  co_.primitive_poses[0].orientation.z = prePose.orientation.z;
  co_.primitive_poses[0].orientation.w = prePose.orientation.w;
}

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
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
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

  //   add collision objects to robot working space
  ROS_INFO_STREAM("Add collision objects into robot working space");
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  //   add kinect obstacles
  moveit_msgs::CollisionObject kinect_obs;
  geometry_msgs::Pose kinect_pose;
  kinect_pose.position.x = 0.6;
  kinect_pose.position.y = 0.0;
  kinect_pose.position.z = 0.5;
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 1.57, 0); 
  kinect_pose.orientation.x = q.x;
  kinect_pose.orientation.y = q.y;
  kinect_pose.orientation.z = q.z;
  kinect_pose.orientation.w = q.w;
  ROS_INFO_STREAM("Obstacle orientation:" << kinect_pose);
  addCollisionObjects(kinect_obs, "kinect_obs", 1.0,
                                                  0.8, 0.03, kinect_pose);

  //   push back all the collisition object into a vector container
  collision_objects.push_back(kinect_obs);

  //   add objects to robot working space
  moveit::planning_interface::PlanningSceneInterface plan_scene_interface;
  plan_scene_interface.addCollisionObjects(collision_objects);

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