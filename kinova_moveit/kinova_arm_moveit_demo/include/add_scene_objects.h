#ifndef ADD_SCENE_OBJECTS_H
#define ADD_SCENE_OBJECTS_H

#include "ros/callback_queue.h"
#include "ros/ros.h"
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

class build_workScene {
public:
  build_workScene(ros::NodeHandle &nh);
  void add_boxModel(const char *name, float length, float width, float height,
                    geometry_msgs::Pose prePose);
  void add_meshModel(const char *name, const char *path,
                     geometry_msgs::Pose prePose);
  void clear_WorkScene(const char *objName);

  std::vector<moveit_msgs::CollisionObject> collision_objects;

private:
  /* data */
  ros::NodeHandle nh_;
  ros::Publisher pub;
  ros::Publisher pub_aco_;
  ros::Publisher pub_planning_scene_diff_;

  moveit_msgs::CollisionObject co_;
  geometry_msgs::PoseStamped can_pose_;
  // work scene
  moveit_msgs::AttachedCollisionObject aco_;
  moveit_msgs::PlanningScene planning_scene_msg_;
};

#endif // ADD_SCENE_OBJECTS_H