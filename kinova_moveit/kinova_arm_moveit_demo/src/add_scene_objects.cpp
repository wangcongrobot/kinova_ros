#include <add_scene_objects.h>

build_workScene::build_workScene(ros::NodeHandle &nh) : nh_(nh) {
  pub = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);
  pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(
      "/attached_collision_object", 10);
  pub_planning_scene_diff_ =
      nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Duration(1.0).sleep();
}

void build_workScene::add_boxModel(const char *name, float length, float width,
                                   float height, geometry_msgs::Pose prePose) {
  co_.header.frame_id = "root";
  co_.header.stamp = ros::Time::now();

  co_.id = name;
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub.publish(co_);

  // add table
  co_.primitives.resize(1);
  co_.primitive_poses.resize(1);
  co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<
                                      shape_msgs::SolidPrimitive::BOX>::value);
  co_.operation = moveit_msgs::CollisionObject::ADD;

  //   primitive size
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] =
      length; // table:2.4*2.4*0.03
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;

  //   primitive pose
  co_.primitive_poses[0].position.x = prePose.position.x; // x=0,y=0,z=-0.03/2.0
  co_.primitive_poses[0].position.y = prePose.position.y;
  co_.primitive_poses[0].position.z = prePose.position.z;
  co_.primitive_poses[0].orientation.x = prePose.orientation.x;
  co_.primitive_poses[0].orientation.y = prePose.orientation.y;
  co_.primitive_poses[0].orientation.z = prePose.orientation.z;
  co_.primitive_poses[0].orientation.w = prePose.orientation.w;

  pub.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
  //   add individual object to object container
  collision_objects.push_back(co_);
}

void build_workScene::add_meshModel(const char *name, const char *path,
                                    geometry_msgs::Pose prePose) {
  co_.id = name;
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub.publish(co_);

  // add hollowPrism
  co_.primitives.resize(1);
  co_.primitive_poses.resize(1);
  co_.operation = moveit_msgs::CollisionObject::ADD;

  shapes::Mesh *hollowPrism_shape = shapes::createMeshFromResource(path);
  shapes::ShapeMsg hollowPrism_mesh_msg;
  shapes::constructMsgFromShape(hollowPrism_shape, hollowPrism_mesh_msg);
  shape_msgs::Mesh hollowPrism_mesh =
      boost::get<shape_msgs::Mesh>(hollowPrism_mesh_msg);

  co_.meshes.push_back(hollowPrism_mesh);
  co_.mesh_poses.push_back(prePose);
  can_pose_.pose.position.x = co_.primitive_poses[0].position.x;
  can_pose_.pose.position.y = co_.primitive_poses[0].position.y;
  can_pose_.pose.position.z = co_.primitive_poses[0].position.z;

  pub.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
}

void build_workScene::clear_WorkScene(const char *objName) {
  co_.id = objName;
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub.publish(co_);

  planning_scene_msg_.world.collision_objects.push_back(co_);
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
}
