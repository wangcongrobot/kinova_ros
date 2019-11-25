#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>

/*
Publish the end-effector position of jaco arm (arm_6_link postiont), used for
camera calibration
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ee_pos_pub");
    ros::NodeHandle n;

    std::string robot_name("j2n6s300");
    tf::TransformListener listener;

    // position publisher
    ros::Publisher ee_pub = n.advertise<geometry_msgs::PointStamped>("ee_pos", 1);

    geometry_msgs::PointStamped ee_point;
    ee_point.header.frame_id = robot_name + "_link_6";

    // we'll just use the most recent transform available for our simple example
    ee_point.header.stamp = ros::Time();

    // just the origon of the frame
    ee_point.point.x = 0.0;
    ee_point.point.y = 0.0;
    ee_point.point.z = -0.18; // translate form arm_6_link to gripper frame

    ros::Rate rate(30);
    while (n.ok()) {
        geometry_msgs::PointStamped calibrate_point;
        try {
            listener.transformPoint(robot_name + "_link_base", ee_point, calibrate_point);

            ROS_INFO("ee_origin: (%.2f, %.2f. %.2f) -----> base_link: (%.3f, %.3f, "
                     "%.3f) at time %.2f",
                ee_point.point.x, ee_point.point.y, ee_point.point.z, calibrate_point.point.x,
                calibrate_point.point.y, calibrate_point.point.z, calibrate_point.header.stamp.toSec());
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Received an exception trying to transform a point from "
                      "\"ee_frame\" to \"base_link\": %s",
                ex.what());
        }
        ee_pub.publish(calibrate_point);
        rate.sleep();
    }
    return 0;
}
