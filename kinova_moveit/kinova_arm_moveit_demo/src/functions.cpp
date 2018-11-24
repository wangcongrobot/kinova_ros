/*

Some functions

*/

#include "functions.h"
#include "globals.h"

//////////////////////////////////////////////FUNCTIONS/////////////////////////////
void notice_data_clear(id_data_msgs::ID_Data* test)
{
    test->id = 0;
    for (int i = 0; i < 8; i++) test->data[i] = 0;
}

void handleCollisionObj(build_workScene& buildWorkScene)
{

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

void error_deal(int error_nu)
{
    switch (error_nu) {
    case 1: {
        ROS_ERROR("Hand doesn't work normally!");
        break;
    }
    case 2: {
        ROS_ERROR("Dashgo doesn't work normally!");
        break;
    }
    case 3: {
        ROS_ERROR("Kinect doesn't work normally!");
        break;
    }
    case 4: {
        ROS_ERROR("Kinova Arm doesn't work normally!");
        break;
    }
    default:
        break;
    }
}

void poseInit()
{
    geometry_msgs::Pose temp;
    temp.orientation
        = tf::createQuaternionMsgFromRollPitchYaw(1.5, -0.01, -0.1); // grasp orientation

    sweep_pose.orientation = temp.orientation;

    move_pose.orientation = temp.orientation;
    postmove_pose.orientation = temp.orientation;
    test_pose.orientation = temp.orientation;

    test_pose.position.x = 0.2;
    test_pose.position.y = -0.25;
    test_pose.position.z = 0.3;

    postmove_pose.position.x = 0.2;
    postmove_pose.position.y = -0.25;
    postmove_pose.position.z = 0.55; // desk 0.35

    // pose before sweep or suck for each floor
    home_pose.orientation = sweep_pose.orientation;
    home_pose.position.x = -0.2;
    home_pose.position.y = -0.3;
    home_pose.position.z = 0.4;

    // suck_pose.orientation
    //     = tf::createQuaternionMsgFromRollPitchYaw(1.57, -2.5, 0.0); // suck orientation
    suck_pose.orientation.x = 0.190;
    suck_pose.orientation.y = -0.639;
    suck_pose.orientation.z = 0.705;
    suck_pose.orientation.w = 0.240;

    box_pose.orientation = suck_pose.orientation;
    box_pose.position.x = -0.2;
    box_pose.position.y = -0.3;
    box_pose.position.z = 0.4;
}