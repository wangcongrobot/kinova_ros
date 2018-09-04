// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Cartesian_plan");

  if ( argc < 7 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun Jaco moveit plan_arm_ik  x y z  r p y");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of /arm_tool_link expressed in world frame");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
  goal_pose.pose.position.x = atof(argv[1]);
  goal_pose.pose.position.y = atof(argv[2]);
  goal_pose.pose.position.z = atof(argv[3]);
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroup group_arm("arm");
  //choose your preferred planner
//   group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPoseReferenceFrame("world");
  group_arm.setPoseTarget(goal_pose);
  group_arm.setEndEffectorLink("j2n6s300_link_6");

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm.getPlanningFrame());

  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(0.5);


  moveit::planning_interface::MoveGroup::Plan my_plan;
  //set maximum time to find a plan
  group_arm.setPlanningTime(5.0);
  bool success = group_arm.plan(my_plan);

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}