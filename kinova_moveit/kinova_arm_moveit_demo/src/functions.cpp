/*

Some functions

*/

#include "functions.h"

//////////////////////////////////////////////FUNCTIONS/////////////////////////////

void handleCollisionObj(build_workScene& buildWorkScene)
{

    //   buildWorkScene.clear_WorkScene("table");
    //   buildWorkScene.clear_WorkScene("kinectBody");
    //   buildWorkScene.clear_WorkScene("kinectObs");

    geometry_msgs::Pose tablePose, kinectBasePose, kinectBodyPose, sliderPose;

    tablePose.position.x = 0.4;
    tablePose.position.y = 0;
    tablePose.position.z = 0.1; // avoid collision between arm and surface
    buildWorkScene.add_boxModel("table", 1.0, 0.5, 0.01, tablePose);

    // add kinect stick obstacle
    kinectBasePose.position.x = 0.4;
    kinectBasePose.position.y = 0.1;
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
    kinectBodyPose.position.y = 0.1; //-0.05 VS 0
    kinectBodyPose.position.z = 0.80;  // 0.75
    tf::Quaternion qbody = tf::createQuaternionFromRPY(0, 0, -1.57);
    // tf::Quaternion qbody = tf::createQuaternionFromRPY(0, -0.26, 0);
    // ROS_INFO_STREAM("Kinect obs pose: " << qbody);
    kinectBodyPose.orientation.x = qbody[0];
    kinectBodyPose.orientation.y = qbody[1];
    kinectBodyPose.orientation.z = qbody[2];
    kinectBodyPose.orientation.w = qbody[3];
    buildWorkScene.add_boxModel("kinectBody", 0.1, 0.3, 0.1, kinectBodyPose);

    // add kinect stick obstacle
    sliderPose.position.x = 0.35;
    sliderPose.position.y = -0.25;
    sliderPose.position.z = 0.20;
    tf::Quaternion slider = tf::createQuaternionFromRPY(0, 1.57, 1.57);
    // tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 0);
    // ROS_INFO_STREAM("Kinect obs pose: " << qkinect);
    sliderPose.orientation.x = slider[0];
    sliderPose.orientation.y = slider[1];
    sliderPose.orientation.z = slider[2];
    sliderPose.orientation.w = slider[3];
    buildWorkScene.add_boxModel("slider", 0.20, 0.03, 0.02, sliderPose);

}