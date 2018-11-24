
#include "jrc18sia_motion_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jrc18sia_motion_planner_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

//    JRCPickPlace jrc_pick_place(nh);

//    jrc_pick_place.pickPlacePipleline();

    JRCMotionPlanner motion_planner(nh);
    motion_planner.cartesionPathPlanner(0.0,0.0,0.0,90,0,0,100,2);
    geometry_msgs::Pose pregrasp_pose;
    pregrasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57,-1.57,0);
    motion_planner.cartesionPathPlanner(0.1,-0.1,0.1,100,2);
    motion_planner.cartesionPathPlanner(0,-0.15,0,100,2);
    geometry_msgs::PoseStamped current_pose;
    current_pose = motion_planner.getCurrentPoseFromMoveit();
    pregrasp_pose.position = current_pose.pose.position;
//    pregrasp_pose.position.y -= 0.15;
//    pregrasp_pose.position.z -= 0.1;
    motion_planner.moveToTargetBestTime(pregrasp_pose);
    motion_planner.cartesionPathPlanner(0.0,0.0,0.0,90,0,0,100,2);
//    return 0;
    current_pose = motion_planner.getCurrentPoseFromMoveit();
    ROS_INFO_STREAM(current_pose);
    current_pose.pose.position.y += 0.1;
    motion_planner.moveLineTarget(current_pose);
    current_pose.pose.position.y -= 0.1;
    motion_planner.moveLineTarget(current_pose);

    motion_planner.cartesionPathPlanner(0.01,0.01,0.1,100,2);
    motion_planner.cartesionPathPlanner(0.0,0.01,0.01,90,0,0,100,2);
//    motion_planner.cartesionPathPlanner(0.0,0.1,0.0,90,0,0,100,2);

    current_pose = motion_planner.getCurrentPoseFromMoveit();
    current_pose.pose.position.z -= 0.1;
    motion_planner.moveToTargetBestTime(current_pose);
    motion_planner.moveToTargetNamed("Home");

    ros::Duration timer(0.5);

    ros::shutdown();
    return 0;
}
