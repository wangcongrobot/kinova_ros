/*
 * Receive target point from kinect and move arm to the target
 * and grasp
*/

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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

// custom headers
#include "darknet_ros_msgs/TargetPoint.h" // kinect info
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include <add_scene_objects.h>    // handle scene obstacles
#include <pick_place.h>

using namespace std;
#define PREGRASP_OFFSET 0.20
#define DEBUG true
typedef int ErrorCode;
// hand flags
bool hand_rec_msg_flag = false;
bool hand_act_finished_flag = false;
bool arm_rec_msg_flag = false;

int error_no = 0;
string robot_name = "j2n6s300";
geometry_msgs::Pose goal_pose;
darknet_ros_msgs::TargetPoint canPoint;

void kinectCallback(const darknet_ros_msgs::TargetPoints::ConstPtr& msg)
{
    if (arm_rec_msg_flag == false) {
        for (int i = 0; msg->target_points.size(); i++) {
            if (msg->target_points[i].Class == "can") {
                canPoint = msg->target_points[i];
                ROS_INFO_STREAM("can position: " << canPoint);
                break;
            }
        }
        goal_pose.position.x = canPoint.camera_x;
        goal_pose.position.y = canPoint.camera_y;
        goal_pose.position.z = canPoint.camera_z + 0.03;
        goal_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);
        if (goal_pose.position.x != 0 && goal_pose.position.y < -0.1) {
            arm_rec_msg_flag = true;
            ROS_INFO_STREAM("Get object pose from kinect");
        }
    } else
        return;
}

class notice_pub_sub {
public:
    boost::function<void(const id_data_msgs::ID_Data::ConstPtr&)> notice_pub_sub_msgCallbackFun;

    notice_pub_sub();
    void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
    void notice_display(id_data_msgs::ID_Data notice_msg, bool set);
    void notice_sub_spinner(char set);
    void notice_data_clear(id_data_msgs::ID_Data* test);

private:
    ros::NodeHandle notice_handle;
    ros::Subscriber notice_subscriber;
    ros::Publisher notice_publisher;
    ros::SubscribeOptions notice_ops;
    ros::AsyncSpinner* notice_spinner;
    ros::CallbackQueue notice_callbackqueue;
    void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr& notice_msg);
};

notice_pub_sub::notice_pub_sub()
{
    notice_pub_sub_msgCallbackFun = boost::bind(&notice_pub_sub::notice_msgCallback, this, _1);
    notice_ops = ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
        "/notice", 50, notice_pub_sub_msgCallbackFun, ros::VoidPtr(), &notice_callbackqueue);
    notice_subscriber = notice_handle.subscribe(notice_ops);
    notice_spinner = new ros::AsyncSpinner(1, &notice_callbackqueue);

    notice_publisher = notice_handle.advertise<id_data_msgs::ID_Data>("/notice", 50);
}

void notice_pub_sub::notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data)
{
    notice_publisher.publish(id_data);
}

void notice_pub_sub::notice_display(id_data_msgs::ID_Data notice_msg, bool set)
{

    if (set) {
        printf("REC Notice message,ID: %d,Data: ", notice_msg.id);
        for (char i = 0; i < 8; i++) {
            printf("%d ", notice_msg.data[i]);
            if (i == 7) printf("\n");
        }
    }
}
void notice_pub_sub::notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr& notice_msg)
{

    id_data_msgs::ID_Data notice_message;
    notice_message.id = 0;
    for (char i = 0; i < 8; i++) notice_message.data[i] = 0;

    notice_message.id = notice_msg->id;
    for (char i = 0; i < 8; i++) notice_message.data[i] = notice_msg->data[i];

    notice_pub_sub::notice_display(notice_message, true);

    if (notice_message.id == 1 && notice_message.data[0] == 14) // hand rec
    {
        hand_rec_msg_flag = true;
    }

    if (notice_message.id == 1 && notice_message.data[0] == 2) // hand task over
    {
        hand_act_finished_flag = true;
    }
}

void notice_pub_sub::notice_sub_spinner(char set)
{
    if (set == 1) notice_spinner->start();
    if (set == 0) notice_spinner->stop();
}

void notice_pub_sub::notice_data_clear(id_data_msgs::ID_Data* test)
{
    test->id = 0;
    for (int i = 0; i < 8; i++) test->data[i] = 0;
}

/****************************function declaraton**********************/
void notice_data_clear(id_data_msgs::ID_Data* test);

void moveToTarget(const geometry_msgs::Pose& target);
void moveToTarget(const std::string& target_name);
void moveToTarget(const geometry_msgs::PoseStamped& target);
void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
void confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const string& str);
void confirmToAct(const geometry_msgs::Pose& goal, const string& str);
void handleCollisionObj(build_workScene& buildWorkScene);
ErrorCode hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
    bool* msg_rec_flag, bool* finished_flag, notice_pub_sub* notice_test);
void error_deal(int error_no);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualControl");
    notice_pub_sub notice_test;
    id_data_msgs::ID_Data notice_data_pub;

    // dfault target value
    // goal_pose.position.x = -0.2;
    // goal_pose.position.y = -0.4;
    // goal_pose.position.z = 0.5;
    goal_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);

    if (argc < 4) {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun Jaco moveit plan_arm_ik  x y z  r p y");
        ROS_INFO(" ");
        ROS_INFO("\twhere the list of arguments specify the target pose of "
                 "/arm_tool_link expressed in world frame");
        ROS_INFO("Plan with default target point ");

    } else {
        ROS_INFO("Plan with manually defined target point ");
        goal_pose.position.x = atof(argv[1]);
        goal_pose.position.y = atof(argv[2]);
        goal_pose.position.z = atof(argv[3]);
    }

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // test hk
    kinova::PickPlace pick_place(nh);

    ros::Subscriber subKinect = nh.subscribe("/darknet_ros/target_points", 100, kinectCallback);
    ros::spinOnce();

    ROS_INFO_STREAM("Target object position:" << goal_pose.position.x << "," << goal_pose.position.y
                                              << "," << goal_pose.position.z);

    /**********************ADD COLLISION***************************/
    ROS_INFO("Add collision objects  into the world (kinect and mobile base)");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    build_workScene buildWorkScene(nh);
    handleCollisionObj(buildWorkScene); // add objects
    planning_scene_interface.addCollisionObjects(
        buildWorkScene.collision_objects); // add collision objects into world
    ROS_INFO("Collision setup finished");

    // select group of joints
    geometry_msgs::Pose pregrasp_pose = goal_pose;
    pregrasp_pose.position.y += PREGRASP_OFFSET;
    geometry_msgs::Pose gripper_rest_pose = pregrasp_pose;
    gripper_rest_pose.position.x = -0.3;
    gripper_rest_pose.position.y = -0.3;
    gripper_rest_pose.position.z = 0.5;

    // 1. pregrasp
    string pose_name = "PREGRASP POSE";
    confirmToAct(pregrasp_pose, pose_name);
    moveToTarget(pregrasp_pose); // plan to pre-grasp pose

    // 2. move forward
    geometry_msgs::Pose start;
    if (DEBUG) {
        start = pregrasp_pose; // virtual pose
    } else {
        start = pick_place.get_ee_pose(); // real pose from driver info.
        start.orientation = pregrasp_pose.orientation;
    }
    geometry_msgs::Pose goal = goal_pose;
    pose_name = "TARGET AND GRASP";
    confirmToAct(start, goal, pose_name);
    moveLineTarget(start, goal);

    // 3. GRASP TEST HK
    ROS_INFO("Begin grasp demo, press n to next:");
    string pause_;
    cin >> pause_;

    if ("n" == pause_) {
        ROS_INFO("Begin grasp demo");

    } else {
        return EXIT_FAILURE;
    }

    notice_test.notice_data_clear(&notice_data_pub);
    notice_data_pub.id = 1;
    notice_data_pub.data[0] = 3;
    // notice_test.notice_pub_sub_pulisher(notice_data_pub);
    ErrorCode err = hand_MsgConform_ActFinishedWait(
        &notice_data_pub, &hand_rec_msg_flag, &hand_act_finished_flag, &notice_test);
    error_deal(err);
    ros::Duration(5).sleep();

    // 4. move to rest pose
    if (DEBUG) {
        start = goal_pose;
    } else {
        start = pick_place.get_ee_pose();
        start.orientation = pregrasp_pose.orientation;
    }

    goal = goal_pose;
    goal.position.z += 0.06;
    pose_name = "PREGRASP (UP)";
    confirmToAct(start, goal, pose_name);
    moveLineTarget(start, goal);

    start = goal;
    goal.position.y += PREGRASP_OFFSET;
    pose_name = "PREGRASP (BACK)";
    confirmToAct(start, goal, pose_name);
    moveLineTarget(start, goal);

    pose_name = "REST POSE";
    confirmToAct(pregrasp_pose, gripper_rest_pose, pose_name);
    moveToTarget(gripper_rest_pose);

    spinner.stop();
    return EXIT_SUCCESS;
}

// Cartesian line plan
void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
    ROS_INFO("Begin cartesian line plan");
    geometry_msgs::Pose way_pose = start;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(way_pose); // first pose waypoint
    int num_waypoint = 5;
    float delta_x = (goal.position.x - start.position.x) / (num_waypoint - 1);
    float delta_y = (goal.position.y - start.position.y) / (num_waypoint - 1);
    float delta_z = (goal.position.z - start.position.z) / (num_waypoint - 1);

    // interplotate between current pose and target pose
    for (int i = 0; i < num_waypoint - 1; i++) {
        way_pose.position.x += delta_x;
        way_pose.position.y += delta_y;
        way_pose.position.z += delta_z;
        waypoints.push_back(way_pose);
    }

    moveit::planning_interface::MoveGroup group("arm");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    moveit::planning_interface::MoveGroup::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    group.execute(cartesian_plan);
}

void confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const string& str = "NULL")
{
    cout << "\n"
         << "=================MOVE TO " + str + "=================="
         << "\n";
    ROS_INFO_STREAM("Move from: " << start << "to " << goal);
    ROS_INFO_STREAM("Confirm info. and press n to execute plan");
    string pause_;
    cin >> pause_;
    if ("n" == pause_) {
        ROS_INFO_STREAM("Valid plan, begin to execute");
    } else {
        return;
    }
}

void confirmToAct(const geometry_msgs::Pose& goal, const string& str = "NULL")
{
    cout << "\n"
         << "=================MOVE TO " + str + "=================="
         << "\n";
    ROS_INFO_STREAM("Move to target " << goal);
    ROS_INFO_STREAM("Confirm info and press n to execute plan");
    string pause_;
    cin >> pause_;
    if ("n" == pause_) {
        ROS_INFO_STREAM("Valid plan, begin to execute");
    } else {
        return;
    }
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

// Update--Alvin: Plan with Pose target
void moveToTarget(const geometry_msgs::Pose& target)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    group.setMaxVelocityScalingFactor(0.5);
    // group.setNumPlanningAttempts(1);
    group.setStartStateToCurrentState();
    group.setPoseTarget(target);

    ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink() << " with respect to frame  "
                                        << group.getPlanningFrame() << " with obstacles");

    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setPlanningTime(3.0);

    int loops = 10; // planing tries
    bool success = group.plan(my_plan);

    for (int i = 0; i < loops; i++) {
        success = group.plan(my_plan);
        if (success) {
            ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
            break;
            // TODO: choose plans according to measures
        } else {
            ROS_INFO("Plan failed at try: %d", i);
        }
    }

    if (!success) ROS_INFO("No plan found after 10 tries");
    // Execute the plan
    ros::Time start = ros::Time::now();
    group.execute(my_plan);
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}

void notice_data_clear(id_data_msgs::ID_Data* test)
{
    test->id = 0;
    for (int i = 0; i < 8; i++) test->data[i] = 0;
}

ErrorCode hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
    bool* msg_rec_flag, bool* finished_flag, notice_pub_sub* notice_test)
{
    id_data_msgs::ID_Data notice_data;
    int loop_hz = 10;
    ros::Rate loop_rate(loop_hz);

    notice_data_clear(&notice_data);
    notice_data.id = notice_data_test->id;
    for (int i = 0; i < 8; i++) notice_data.data[i] = notice_data_test->data[i];
    notice_test->notice_pub_sub_pulisher(notice_data);

    // hand data receive judge
    int wait_count = 0;
    while (ros::ok()) {
        if (*msg_rec_flag == true) {
            *msg_rec_flag = false;
            wait_count = 0; // reset time for next loop
            break;
        }

        wait_count++;
        if (wait_count % 50 == 0) // send msg again after waiting 1s
        {
            ROS_ERROR("Hand didn't receive msg, retrying...");
            notice_test->notice_pub_sub_pulisher(notice_data);
        }

        if (wait_count >= 10000) {
            error_no = notice_data.id;
            goto next;
        }
        notice_test->notice_sub_spinner(1);
        loop_rate.sleep();
    }
    // hand action finish judge
    while (ros::ok()) {
        if (*finished_flag == true) {
            *finished_flag = false;
            break;
        }
        wait_count++;
        if (wait_count % 100 == 0) // send msg again after waiting 1s
        {
            ROS_ERROR("Waiting for hand to grasp/suck...");
        }
        notice_test->notice_sub_spinner(1);
        loop_rate.sleep();
    }

next:
    return error_no;
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

geometry_msgs::Pose getCurrentState()
{
    geometry_msgs::Pose current_pose;
    tf::TransformListener listener;
    // try {
    //     listener.transformPoint(robot_name + "_link_base", ee_pose, current_pose);

    //     ROS_INFO("current pose: (%.2f, %.2f. %.2f) -----> base_link: (%.3f, %.3f,
    //              "
    //              "%.3f) at time %.2f",
    //         ee_pose.point.x, ee_pose.point.y, ee_pose.point.z, current_pose.point.x,
    //         current_pose.point.y, current_pose.point.z, current_pose.header.stamp.toSec());
    // } catch (tf::TransformException& ex) {
    //     ROS_ERROR("Received an exception trying to transform a point from "
    //               "\"ee_frame\" to \"base_link\": %s",
    //         ex.what());
    // }

    tf::StampedTransform transform;
    try {
        listener.lookupTransform("/root", "/j2n6s300_link_6", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    current_pose.position.x = transform.getOrigin().x();
    current_pose.position.y = transform.getOrigin().y();
    current_pose.position.z = transform.getOrigin().z();

    tf::Quaternion q = transform.getRotation();
    current_pose.orientation.x = q.x();
    current_pose.orientation.y = q.y();
    current_pose.orientation.z = q.z();
    current_pose.orientation.w = q.w();

    // end effector point
    // pick_place.get_current_pose();
    return current_pose;
}
