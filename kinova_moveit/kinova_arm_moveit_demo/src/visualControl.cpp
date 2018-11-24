/*
 * Receive target point from kinect and move arm to the target
 * and grasp
*/

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometry_msgs/PointStamped.h>
// #include <tf2/tf2.h>
// #include <tf2/transform_listener.h>

#include <tf2_eigen/tf2_eigen.h> 
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
// #include <actionlib/client/SimpleActionClient.h>
#include <actionlib/client/simple_action_client.h>

// moveit
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Std C++ headers
#include <map>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// custom headers
#include "darknet_ros_msgs/TargetPoint.h" // kinect info
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include "parser.h"
#include <add_scene_objects.h> // handle scene obstacles
#include <pick_place.h>

using namespace std;
#define PREGRASP_OFFSET 0.20
#define DEBUG false
// const double Pi = 3141592653;
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
        for (int i = 0; i < msg->target_points.size(); i++) {
            if (msg->target_points[i].Class == "can") {
                canPoint = msg->target_points[i];
                ROS_INFO_STREAM("can position: " << canPoint);
                break;
            }
        }
        goal_pose.position.x = canPoint.camera_x;
        goal_pose.position.y = canPoint.camera_y;
        goal_pose.position.z = canPoint.camera_z + 0.02;
        goal_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);
        if (goal_pose.position.x != 0 && goal_pose.position.y < -0.1) {
            arm_rec_msg_flag = true;
            ROS_INFO_STREAM("Get object pose from kinect");
        }
    }
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

std::vector<double> current_joint_values;

void currentJointValuesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_values.clear();
    double temp_joint = 0;
    for (int i = 0; i < msg->position.size(); i++) {
        temp_joint = msg->position[i];
        current_joint_values.push_back(temp_joint);
    }
    // ROS_INFO_STREAM("Get current joint values.");
    // for(size_t i=0; i<6; i++)
    // {
    //     cout << current_joint_values[i] << endl;
    // }
}
/*
* roll: axis X, pitch: axis Y, yaw: Z
*
*/
void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ);
void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ,
                              double roll, double pitch, double yaw,
                              int number_point, int number_distance);
/****************************function declaraton**********************/
void notice_data_clear(id_data_msgs::ID_Data* test);

void moveToTarget(const geometry_msgs::Pose& target);
void moveToTarget(const std::string& target_name);
void moveToTarget(const geometry_msgs::PoseStamped& target);
void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
void confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const string& str);
void confirmToAct(const geometry_msgs::Pose& goal, const string& str);
void confirmToAct();
void handleCollisionObj(build_workScene& buildWorkScene);
ErrorCode hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
    bool* msg_rec_flag, bool* finished_flag, notice_pub_sub* notice_test);
void error_deal(int error_no);
int evaluateMoveitPlan(moveit::planning_interface::MoveGroup::Plan& plan); // return plan steps
void moveLineFromCurrentState(
    double distanceX, double distanceY, double distanceZ, int number_point, int number_distance);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualControl");
    ros::NodeHandle nh;

    notice_pub_sub notice_test;
    id_data_msgs::ID_Data notice_data_pub;

    double distanceX = 0;
    double distanceY = 0;
    double distanceZ = 0;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;
    int number_point = 0;
    int number_distance = 0;


    ros::Subscriber sub_joint_values
        = nh.subscribe("/j2n6s300_driver/out/joint_state", 10, currentJointValuesCallback);

    // dfault target value
    // goal_pose.position.x = -0.2;
    // goal_pose.position.y = -0.4;
    // goal_pose.position.z = 0.5;
    goal_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0.0);
    // goal_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);

    if (argc < 9) {
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

        distanceX            = atof(argv[1]);
        distanceY            = atof(argv[2]);
        distanceZ            = atof(argv[3]);
        roll                 = atof(argv[4]); // X
        pitch                = atof(argv[5]); // Y
        yaw                  = atof(argv[6]); // Z
        number_point         = atof(argv[7]);
        number_distance      = atof(argv[8]);
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    kinova::PickPlace pick_place(nh);
    // test hk

    // ros::Subscriber subKinect = nh.subscribe("/darknet_ros/target_points", 100, kinectCallback);
    // ros::Rate rate(200);
    // while (ros::ok()) {
    //     if (arm_rec_msg_flag) {
    //         ROS_INFO("Target position returned");
    //         break;
    //     }
    //     ros::spinOnce();
    //     rate.sleep();
    // }

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

    // predifined poses
    geometry_msgs::Pose pregrasp_pose = goal_pose;
    pregrasp_pose.position.y += PREGRASP_OFFSET;
    geometry_msgs::Pose gripper_rest_pose = pregrasp_pose;
    gripper_rest_pose.position.x = -0.2;
    gripper_rest_pose.position.y = -0.3;
    gripper_rest_pose.position.z = 0.4;

    // test getCurrentPose() func. --Alvin
    moveit::planning_interface::MoveGroup group("arm");

    // 1. pregrasp
    geometry_msgs::Pose start;
    if (DEBUG) {
        start = group.getCurrentPose().pose;
    } else {
        start = pick_place.get_ee_pose(); // real pose from driver info.
    }
    string pose_name = "PREGRASP POSE";

    // confirmToAct(start, pregrasp_pose, pose_name);
    // moveToTarget(pregrasp_pose); // plan to pre-grasp pose

    // 2. move forward
    if (DEBUG) {
        start = pregrasp_pose; // virtual pose
    } else {
        start = pick_place.get_ee_pose(); // real pose from driver info.
        start.orientation = pregrasp_pose.orientation;
    }

    geometry_msgs::Pose goal = goal_pose;
    pose_name = "TARGET AND GRASP";
    // confirmToAct(start, goal, pose_name);
    // moveLineTarget(start, goal);

    // while(ros::ok())
    {
        ros::spinOnce();
        moveLineFromCurrentState(distanceX, distanceY, distanceZ, roll, pitch, yaw, number_point, number_distance);
        // ros::Duration(3).sleep();
        // exit(0);
    }

    // ros::Duration(3).sleep();
    exit(0);

    // 3. GRASP TEST HK
    // ROS_INFO("Begin grasp demo, press n to next:");
    // string pause_;
    // cin >> pause_;

    // if ("n" == pause_) {
    //     ROS_INFO("Begin grasp demo");

    // } else {
    //     return EXIT_FAILURE;
    // }

    // notice_test.notice_data_clear(&notice_data_pub);
    // notice_data_pub.id = 1;
    // notice_data_pub.data[0] = 3;
    // // notice_test.notice_pub_sub_pulisher(notice_data_pub);
    // ErrorCode err = hand_MsgConform_ActFinishedWait(
    //     &notice_data_pub, &hand_rec_msg_flag, &hand_act_finished_flag, &notice_test);
    // error_deal(err);
    // ros::Duration(5).sleep();

    // 4. move to rest pose
    if (DEBUG) {
        start = goal_pose;
    } else {
        start = pick_place.get_ee_pose();
        start.orientation = pregrasp_pose.orientation;
    }

    goal = start;
    goal.position.z += 0.06;
    pose_name = "PREGRASP (UP)";
    confirmToAct(start, goal, pose_name);
    moveLineTarget(start, goal);
    ros::Duration(5).sleep();

    if (DEBUG) {
        start = goal;
    } else {
        start = pick_place.get_ee_pose();
        start.orientation = pregrasp_pose.orientation;
    }
    goal = start;
    goal.position.y += PREGRASP_OFFSET;
    pose_name = "PREGRASP (BACK)";
    confirmToAct(start, goal, pose_name);
    moveLineTarget(start, goal);
    ros::Duration(5).sleep();

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
    int num_waypoint = 8;
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
    group.setPlannerId("RRTConnectkConfigDefault");
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 1.0;
    const double eef_step = 0.01;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_ERROR("fraction = %f", fraction);
    moveit::planning_interface::MoveGroup::Plan cartesian_plan, temp_plan;
    cartesian_plan.trajectory_ = trajectory;
    int plan_steps = trajectory.joint_trajectory.points.size();
    ROS_INFO_STREAM("Line plan steps: " << plan_steps);
    if (plan_steps < 25) {
        ROS_INFO_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps");
        group.execute(cartesian_plan);
    } else {
        exit(0);
    }
}

void confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const string& str = "NULL")
{
    cout << "\n"
         << "=================MOVE TO " + str + "=================="
         << "\n";
    ROS_INFO_STREAM("Move from: " << start << "to " << goal);
    ROS_INFO_STREAM("Confirm start and end info and press n to start plan");
    string pause_;
    cin >> pause_;
    if ("n" == pause_) {
        ROS_INFO_STREAM("Valid info, begin to plan ");
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
    ROS_INFO_STREAM("Confirm start and end info and press n to start plan");
    string pause_;
    cin >> pause_;
    if ("n" == pause_) {
        ROS_INFO_STREAM("Valid info, begin to plan");
    } else {
        return;
    }
}
void confirmToAct()
{
    ROS_INFO_STREAM("Confirm start and end info and press n to start plan");
    string pause_;
    cin >> pause_;
    if ("n" == pause_) {
        ROS_INFO_STREAM("Valid info, begin to plan");
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
    group.setPlannerId("RRTConnectkConfigDefault");
    // group.setNumPlanningAttempts(1);
    group.setStartStateToCurrentState();
    group.setPoseTarget(target);

    ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink() << " with respect to frame  "
                                        << group.getPlanningFrame() << " with obstacles");

    moveit::planning_interface::MoveGroup::Plan my_plan, temp_plan;
    group.setPlanningTime(3.0);

    int loops = 100; // planing tries
    bool success = false;
    bool plan_valid = false;
    int plan_steps = 0;
    int current_steps = 0;
    int min_steps = 100;

    // replan until valid paln is returned
    for (int i = 0; i < loops; i++) {
        success = group.plan(temp_plan);
        if (success) {
            current_steps = evaluateMoveitPlan(temp_plan);
            if (current_steps < min_steps) {
                my_plan = temp_plan;
                min_steps = current_steps;
                ROS_INFO_STREAM("Try " << i << ": plan found in " << my_plan.planning_time_
                                       << " seconds with " << min_steps << " steps");
                // plan_valid = true;
                // break;
            }
            // TODO: choose plans according to measures
        } else {
            ROS_INFO("Plan failed at try: %d", i);
        }
    }
    if (min_steps < 25) plan_valid = true;

    if (!plan_valid) {
        ROS_INFO("No valid plan found after 20 tries");
        exit(0);
    }

    // Execute the plan
    ROS_INFO("Print n to execute the plan");
    string pause_;
    cin >> pause_;
    if (pause_ == "n") {
        ros::Time start = ros::Time::now();
        group.execute(my_plan);
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    }
}

int evaluateMoveitPlan(moveit::planning_interface::MoveGroup::Plan& plan)
{
    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
    // std::vector<trajectory_msgs::JointTrajectoryPoint> points =
    // trajectory.joint_trajectory.points;
    return trajectory.joint_trajectory.points.size();
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
        wait_count++;
        if (*msg_rec_flag == true) {
            *msg_rec_flag = false;
            wait_count = 0; // reset time for next loop
            break;
        }

        if (wait_count % 10 == 0) // send msg again after waiting 1s
        {
            ROS_ERROR("Hand didn't receive msg, retrying...");
            notice_test->notice_pub_sub_pulisher(notice_data);
        }

        if (wait_count >= 1000) {
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
        if (wait_count % 20 == 0) // send msg again after waiting 1s
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

// void moveLineFromCurrentState(
//     double distanceX, double distanceY, double distanceZ, int number_point, int number_distance)
// {
//     actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(
//         "/j2n6s300/follow_joint_trajectory", true);
//     ROS_INFO("Waiting for action server to start.");
//     ac.waitForServer();
//     ROS_INFO("Action server started, sending trajectory.");

//     moveit::planning_interface::MoveGroup group("arm");
//     moveit::planning_interface::MoveGroup::Plan plan;
//     std::vector<double> joint_values = group.getCurrentJointValues();
//     control_msgs::FollowJointTrajectoryGoal goal;
//     moveit_msgs::RobotTrajectory robot_trajectory;

//     // robot_trajectory::RobotTrajectory rt(model_loader.getModel(), "arm");
//     // moveit_msgs::RobotTrajectory trajectory;
//     double trajectory_velocity_scaling_ = 0.2;

//     robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
//     // rt.setRobotTrajectoryMsg(start_state, plan1.trajectory_);

//     // trajectory_processing::IterativeParabolicTimeParameterization iptp;
//     // iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
//     // rt.getRobotTrajectoryMsg(plan1.trajectory_);

//     Eigen::VectorXd qPre(6);
//     qPre << 4.89, 3.29, 1.34, 4.19, 1.54, 1.30;
//     // qPre << joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4],
//     // joint_values[5];
//     cout << "qPre: " << qPre << endl;
//     Parser parser;
//     Eigen::Matrix4d transformation = parser.Foward(qPre);
//     cout << "transformation: " << transformation << endl;
//     goal.trajectory.header.frame_id = "j2n6s300_base";
//     goal.trajectory.header.stamp = ros::Time::now();
//     goal.trajectory.joint_names.clear();
//     robot_trajectory.joint_trajectory.header.frame_id = "j2n6s300_base";
//     robot_trajectory.joint_trajectory.header.stamp = ros::Time::now();
//     robot_trajectory.joint_trajectory.joint_names.clear();

//     for (int k = 0; k < 6; k++) {
//         stringstream jointName;
//         jointName << "j2n6s300_joint_" << (k + 1);
//         goal.trajectory.joint_names.push_back(jointName.str());
//         robot_trajectory.joint_trajectory.joint_names.push_back(jointName.str());
//     }

//     goal.trajectory.points.clear();
//     robot_trajectory.joint_trajectory.points.clear();

//     int number1 = number_point, number2 = number_distance;
//     for (int i = 0; i < number1; i++) {
//         transformation(0, 3) += distanceX / number1;
//         transformation(1, 3) += distanceY / number1;
//         transformation(2, 3) += distanceZ / number1;
//         Eigen::VectorXd q(6);
//         q = parser.Inverse(transformation, qPre);
//         cout << "=========" << endl;
//         if (q(0) > 10) continue;

//         // printf("loops:%d",i);
//         for (int j = 0; j < number2; j++) {
//             trajectory_msgs::JointTrajectoryPoint point;
//             for (int k = 0; k < 6; k++) {
//                 point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number2 * j);
//                 point.velocities.push_back(0.05); // 0.1
//                 point.accelerations.push_back(0.05);
//             }
//             point.time_from_start = ros::Duration();
//             goal.trajectory.points.push_back(point);
//             robot_trajectory.joint_trajectory.points.push_back(point);
//         }
//         // printf("\n\njoint values : %d\n",i);
//         std::cout << q << std::endl;
//         qPre = q;
//     }
//     trajectory_processing::IterativeParabolicTimeParameterization iptp;
//     // iptp.computeTimeStamps(robot_trajectory, 0.5);
//     iptp.computeTimeStamps(robot_trajectory);
//     // robot_trajectory.getRobotTrajectoryMsg(plan.trajectory_);
//     plan.trajectory_ = robot_trajectory;
//     ROS_INFO("Send goal to kinova");
//     group.execute(plan);
//     // ac.sendGoal(goal);
//     // ac.waitForResult(ros::Duration(10));
//     ROS_INFO_ONCE("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");
// }

void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ,
                              double roll, double pitch, double yaw,
                              int number_point, int number_distance)
{   
    roll = (roll / 180.0 * Pi); // Ｘ
    pitch = (pitch / 180.0 * Pi); // Y
    yaw = (yaw / 180.0 * Pi); // Z
    cout << "Received RPY (XYZ) angle: " << "\n" << roll * 180 / Pi << " " << pitch * 180 / Pi << " " << yaw * 180 / Pi << endl;

    tf::Quaternion end_quat_tf;
    /**@brief Set the quaternion using fixed axis RPY
    * @param roll Angle around X 
    * @param pitch Angle around Y
    * @param yaw Angle around Z*/
    end_quat_tf.setRPY(roll, pitch, yaw);

    // int number_point = 20;
    // int number_distance = 5;

    // MoveIt!
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup::Plan plan;
    group.setStartStateToCurrentState();
    std::vector<double> joint_values = group.getCurrentJointValues();
    cout << "Current pose: " << "\n" << group.getCurrentPose() << endl;
    // cout << "Current RPY: " << "\n" << group.getCurrentRPY() << endl;
    control_msgs::FollowJointTrajectoryGoal goal;
    moveit_msgs::RobotTrajectory trajectory_msg;

    Eigen::Matrix3d rotation_matrix_eigen;
    tf::Matrix3x3 rotation_matrix_tf;
    tf::Quaternion q_tf;
    
    Eigen::VectorXd qPre(6);
    // qPre << 4.89, 3.29, 1.34, 4.19, 1.54, 1.30;
    // qPre << joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4],
    qPre << current_joint_values[0], current_joint_values[1], 
            current_joint_values[2], current_joint_values[3], 
            current_joint_values[4], current_joint_values[5];
    cout << "qPre: " << qPre << endl;
    Parser parser;
    Eigen::Matrix4d transformation = parser.Foward(qPre);
    cout << "transformation1: " << "\n" << transformation << endl;
    rotation_matrix_eigen << transformation(0,0), transformation(0,1), transformation(0,2),
                             transformation(1,0), transformation(1,1), transformation(1,2),
                             transformation(2,0), transformation(2,1), transformation(2,2);

    Eigen::Quaterniond eigen_quat(rotation_matrix_eigen);
    cout << "eigen Quaterniond1:" << "\n" << eigen_quat.x() << " " << eigen_quat.y() << " " << eigen_quat.z() << " " << eigen_quat.w() << endl;

    // Eigen => tf
    tf::Quaternion start_quat_tf(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    cout << "tf::Quaternion:" << "\n" << start_quat_tf.x() << " " << start_quat_tf.y() << " " << start_quat_tf.z() << " " << start_quat_tf.w() << endl;

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double R, P, Y;
    tf::Matrix3x3 tf_rotation_matrix(start_quat_tf);
    tf_rotation_matrix.getRPY(R, P, Y);
    cout << "tf RPY (XYZ) angle: " << "\n" << R * 180 / Pi << " " << P * 180 / Pi << " " << Y * 180 / Pi << endl;

    goal.trajectory.header.frame_id = "j2n6s300_base";
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.joint_names.clear();
    trajectory_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    trajectory_msg.joint_trajectory.header.stamp = ros::Time::now();
    trajectory_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++) 
    {
        stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
        trajectory_msg.joint_trajectory.joint_names.push_back(jointName.str());
    }

    goal.trajectory.points.clear();
    trajectory_msg.joint_trajectory.points.clear();

    int number1 = number_point, number2 = number_distance;
    tf::Quaternion temp_quat_tf;
    for (int i = 0; i < number1; i++)
    {
        // translation
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;
        // rotation
        temp_quat_tf = start_quat_tf.slerp(end_quat_tf, (1.0 / (float)number1) * (i+1));
        tf::Matrix3x3 rotation_matrix_tf(temp_quat_tf);
        rotation_matrix_tf.getRPY(R, P, Y);
        cout << "Current tf RPY (XYZ) angle 2: " << "\n" << R * 180 / Pi << " " << P * 180 / Pi << " " << Y * 180 / Pi << endl;

        Eigen::Quaterniond eigen_target_q(temp_quat_tf.w(), temp_quat_tf.x(), temp_quat_tf.y(), temp_quat_tf.z());
        rotation_matrix_eigen = eigen_target_q.toRotationMatrix();

        for(int m=0; m<3; m++)
        {
            for(int n=0; n<3; n++)
            {
                transformation(m,n) = rotation_matrix_eigen(m,n);
            }
        }
        // transformation()
        Eigen::VectorXd q(6);
        q = parser.Inverse(transformation, qPre);
        // cout << "=========" << endl;
        if (q(0) > 10) continue;

        // printf("loops:%d",i);
        for (int j = 0; j < number2; j++) {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++) {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number2 * j);
                point.velocities.push_back(0.0);
                point.accelerations.push_back(0.0);
            }
            point.time_from_start = ros::Duration();
            goal.trajectory.points.push_back(point);
            trajectory_msg.joint_trajectory.points.push_back(point);
        }
        // printf("\n\njoint values : %d\n",i);
        // std::cout << q  << "\n" << std::endl;
        qPre = q;
    }
    cout << "trajectory points number: " << trajectory_msg.joint_trajectory.points.size() << endl;

    double trajectory_velocity_scaling_ = 1.0;
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // iptp.computeTimeStamps(robot_trajectory, 0.5);
    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt,trajectory_velocity_scaling_);
    ROS_INFO("Computed time stamped %s", IptpSuccess ? "SUCCED" : "FAILED");
    rt.getRobotTrajectoryMsg(trajectory_msg);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Send goal to kinova");
    // sleep(1.0);
    confirmToAct();
    group.execute(plan);
    // sleep(1.0);
    // ac.sendGoal(goal);
    // ac.waitForResult(ros::Duration(10));
    ROS_INFO_ONCE("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");
}


void moveLineFromCurrentState_backup(double distanceX, double distanceY, double distanceZ,
                              double yaw, double pitch, double roll)
{   
    yaw = (yaw / 180.0 * Pi);
    pitch = (pitch / 180.0 * Pi);
    roll = (roll / 180.0 * Pi);
    int number_point = 20;
    int number_distance = 5;
    // double yaw_ = yaw;
    // double pitch_ = pitch; 
    // double roll_ = roll;
    // Euler angle
    Eigen::Vector3d target_euler_angle(yaw, pitch, roll);
    Eigen::Matrix3d rotation_matrix;

    // Euler to Rotation Matrix
    // Eigen::AngleAxisd rollAngle(eulerAngle(2), Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle(eulerAngle(1), Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle(eulerAngle(0), Eigen::Vector3d::UnitZ());
    // rotation_matrix = yawAngle * pitchAngle * rollAngle;

    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup::Plan plan;
    group.setStartStateToCurrentState();
    std::vector<double> joint_values = group.getCurrentJointValues();
    cout << "Current pose: " << "\n" << group.getCurrentPose() << endl;
    // cout << "Current RPY: " << "\n" << group.getCurrentRPY() << endl;
    control_msgs::FollowJointTrajectoryGoal goal;
    moveit_msgs::RobotTrajectory trajectory_msg;

    // robot_trajectory::RobotTrajectory rt(model_loader.getModel(), "arm");
    // moveit_msgs::RobotTrajectory trajectory;

    // trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
    // rt.getRobotTrajectoryMsg(plan1.trajectory_);

    Eigen::VectorXd qPre(6);
    // qPre << 4.89, 3.29, 1.34, 4.19, 1.54, 1.30;
    // qPre << joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4],
    qPre << current_joint_values[0], current_joint_values[1], 
            current_joint_values[2], current_joint_values[3], 
            current_joint_values[4], current_joint_values[5];
    cout << "qPre: " << qPre << endl;
    Parser parser;
    Eigen::Matrix4d transformation = parser.Foward(qPre);
    cout << "transformation1: " << "\n" << transformation << endl;
    rotation_matrix << transformation(0,0), transformation(0,1), transformation(0,2),
                       transformation(1,0), transformation(1,1), transformation(1,2),
                       transformation(2,0), transformation(2,1), transformation(2,2);
    // cout << "rotation matrix1: " << "\n" << rotation_matrix << endl;
    Eigen::Vector3d current_euler_angle = rotation_matrix.eulerAngles(2,1,0);
    cout << "Current Euler angle1: " << "\n" << (current_euler_angle.transpose() * 180.0 / Pi) << endl;
    for(int i=0;i<3; i++) // TODO
    {
        if(current_euler_angle(i) < -Pi) current_euler_angle(i) += Pi;
        if(current_euler_angle(i) >  Pi) current_euler_angle(i) -= Pi;
        // if(current_euler_angle(i) >  (Pi / 2)) current_euler_angle(i) -= Pi;
        // if(current_euler_angle(i) >  Pi) current_euler_angle(i) -= Pi;
    }
    cout << "Current Euler angle2: " << "\n" << (current_euler_angle.transpose() * 180.0 / Pi) << endl;

    cout << "Target Euler angle2: " << "\n" << (target_euler_angle.transpose() * 180.0 / Pi) << endl;

    goal.trajectory.header.frame_id = "j2n6s300_base";
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.joint_names.clear();
    trajectory_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    trajectory_msg.joint_trajectory.header.stamp = ros::Time::now();
    trajectory_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++) 
    {
        stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
        trajectory_msg.joint_trajectory.joint_names.push_back(jointName.str());
    }

    goal.trajectory.points.clear();
    trajectory_msg.joint_trajectory.points.clear();

    int number1 = number_point, number2 = number_distance;
    target_euler_angle = current_euler_angle;
    for (int i = 0; i < number1; i++) 
    {
        // translation
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;
        // rotation
        target_euler_angle(0) += ((yaw - current_euler_angle(0)) / number1);
        target_euler_angle(1) += ((pitch - current_euler_angle(1)) / number1);
        target_euler_angle(2) += ((roll - current_euler_angle(2)) / number1);

        // Euler to Rotation Matrix
        Eigen::AngleAxisd rollAngle(target_euler_angle(2), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(target_euler_angle(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(target_euler_angle(0), Eigen::Vector3d::UnitZ());
        rotation_matrix = yawAngle * pitchAngle * rollAngle;
        // cout << "rotation matrix2: " << "\n" << rotation_matrix << endl;
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                transformation(i,j) = rotation_matrix(i,j);
            }
        }
        // transformation()
        Eigen::VectorXd q(6);
        q = parser.Inverse(transformation, qPre);
        // cout << "=========" << endl;
        if (q(0) > 10) continue;

        // printf("loops:%d",i);
        for (int j = 0; j < number2; j++) {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++) {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number2 * j);
                point.velocities.push_back(0.0); 
                point.accelerations.push_back(0.0);
            }
            point.time_from_start = ros::Duration();
            goal.trajectory.points.push_back(point);
            trajectory_msg.joint_trajectory.points.push_back(point);
        }
        // printf("\n\njoint values : %d\n",i);
        // std::cout << q  << "\n" << std::endl;
        qPre = q;
    }
    cout << "trajectory points number: " << trajectory_msg.joint_trajectory.points.size() << endl;

    double trajectory_velocity_scaling_ = 0.3;
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // iptp.computeTimeStamps(robot_trajectory, 0.5);
    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt,trajectory_velocity_scaling_);
    ROS_INFO("Computed time stamped %s", IptpSuccess ? "SUCCED" : "FAILED");
    rt.getRobotTrajectoryMsg(trajectory_msg);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Send goal to kinova");
    // sleep(1.0);
    group.execute(plan);
    // sleep(1.0);
    // ac.sendGoal(goal);
    // ac.waitForResult(ros::Duration(10));
    ROS_INFO_ONCE("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");
}

void moveLineFromCurrentState111(double distanceX, double distanceY, double distanceZ,
                              double yaw, double pitch, double roll)
{   
    yaw = (yaw / 180.0 * Pi);
    pitch = (pitch / 180.0 * Pi);
    roll = (roll / 180.0 * Pi);
    int number_point = 20;
    int number_distance = 5;
    // double yaw_ = yaw;
    // double pitch_ = pitch; 
    // double roll_ = roll;
    // Euler angle
    Eigen::Vector3d target_euler_angle(yaw, pitch, roll);
    Eigen::Matrix3d rotation_matrix;
    tf::Matrix3x3 rotation_matrix_tf;
    tf::Quaternion q_tf;
    // Euler to Rotation Matrix
    // Eigen::AngleAxisd rollAngle(eulerAngle(2), Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle(eulerAngle(1), Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle(eulerAngle(0), Eigen::Vector3d::UnitZ());
    // rotation_matrix = yawAngle * pitchAngle * rollAngle;

    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup::Plan plan;
    group.setStartStateToCurrentState();
    std::vector<double> joint_values = group.getCurrentJointValues();
    cout << "Current pose: " << "\n" << group.getCurrentPose() << endl;
    // cout << "Current RPY: " << "\n" << group.getCurrentRPY() << endl;
    control_msgs::FollowJointTrajectoryGoal goal;
    moveit_msgs::RobotTrajectory trajectory_msg;

    // robot_trajectory::RobotTrajectory rt(model_loader.getModel(), "arm");
    // moveit_msgs::RobotTrajectory trajectory;

    // trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
    // rt.getRobotTrajectoryMsg(plan1.trajectory_);

    Eigen::VectorXd qPre(6);
    // qPre << 4.89, 3.29, 1.34, 4.19, 1.54, 1.30;
    // qPre << joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4],
    qPre << current_joint_values[0], current_joint_values[1], 
            current_joint_values[2], current_joint_values[3], 
            current_joint_values[4], current_joint_values[5];
    cout << "qPre: " << qPre << endl;
    Parser parser;
    Eigen::Matrix4d transformation = parser.Foward(qPre);
    cout << "transformation1: " << "\n" << transformation << endl;
    rotation_matrix << transformation(0,0), transformation(0,1), transformation(0,2),
                       transformation(1,0), transformation(1,1), transformation(1,2),
                       transformation(2,0), transformation(2,1), transformation(2,2);
    // tf_rotation_matrix.setVale(transformation(0,0), transformation(0,1), transformation(0,2),
    //                 transformation(1,0), transformation(1,1), transformation(1,2),
    //                 transformation(2,0), transformation(2,1), transformation(2,2));
    cout << "eigen rotation maytix:" << "\n" << rotation_matrix << endl;
    Eigen::Quaterniond eigen_quat(rotation_matrix);
    Eigen::Quaterniond q1 = eigen_quat * eigen_quat;
    Eigen::Quaterniond q2 = eigen_quat;
    q2 = q1.slerp(0.1, eigen_quat);
    cout << "eigen Quaterniond1:" << "\n" << eigen_quat.x() << " " << eigen_quat.y() << " " << eigen_quat.z() << " " << eigen_quat.w() << endl;
    cout << "eigen Quaterniond2:" << "\n" << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << endl;    
    cout << "eigen Quaterniond3:" << "\n" << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << endl;   
    tf::Quaternion current_tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    cout << "tf::Quaternion:" << "\n" << current_tf_quat.x() << " " << current_tf_quat.y() << " " << current_tf_quat.z() << " " << current_tf_quat.w() << endl;
    // cout << current_tf_quat.get
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double R, P, Y;
    tf::Matrix3x3 tf_rotation_matrix(current_tf_quat);
    tf_rotation_matrix.getRPY(R, P, Y);
    // cout << "tf::Matrix3x3:" << endl;
    // // cout << tf_rotation_matrix << endl;
    // for(int i=0; i<3;i++)
    // {
    //     cout << endl;
    //     for(int j=0; j<3;j++)
    //     {
    //         cout << tf_rotation_matrix(i,j) << " ";
    //     }
    //     cout << endl;
    // }

    cout << "tf: Euler angle RPY: " << "\n" << R * 180 / Pi << " " << P * 180 / Pi << " " << Y * 180 / Pi << endl;
    // rotation_matrix_tf(0,0,0,
    //                    1,0,0,
    //                    0,0,1);
    // rotation_matrix_tf(transformation(0,0), transformation(0,1), transformation(0,2),
    //                    transformation(1,0), transformation(1,1), transformation(1,2),
    //                    transformation(2,0), transformation(2,1), transformation(2,2));
    // tf::matrixEigenToTF(rotation_matrix, rotation_matrix_tf);
    // cout << "rotation matrix1: " << "\n" << rotation_matrix << endl;
    Eigen::Vector3d current_euler_angle = rotation_matrix.eulerAngles(2,1,0);
    tf::Vector3 current_euler_angle_tf(R, P, Y);
    tf::Quaternion target_quat_tf;
    target_quat_tf.setRPY(R+0.5, P, Y);
    cout << "tf::Quaternion:" << "\n" << current_tf_quat.x() << " " << current_tf_quat.y() << " " << current_tf_quat.z() << " " << current_tf_quat.w() << endl;
    cout << "tf::Quaternion:" << "\n" << target_quat_tf.x() << " " << target_quat_tf.y() << " " << target_quat_tf.z() << " " << target_quat_tf.w() << endl;
    current_tf_quat = current_tf_quat.slerp(target_quat_tf, 0.5);
    cout << "tf::Quaternion:" << "\n" << current_tf_quat.x() << " " << current_tf_quat.y() << " " << current_tf_quat.z() << " " << current_tf_quat.w() << endl;
    cout << "tf::Quaternion:" << "\n" << target_quat_tf.x() << " " << target_quat_tf.y() << " " << target_quat_tf.z() << " " << target_quat_tf.w() << endl;
    // cout << "Current Euler angle1 (from Eigen): " << "\n" << (current_euler_angle.transpose() * 180.0 / Pi) << endl;

    // cout << "Target Euler angle2: " << "\n" << (target_euler_angle.transpose() * 180.0 / Pi) << endl;

    goal.trajectory.header.frame_id = "j2n6s300_base";
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.joint_names.clear();
    trajectory_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    trajectory_msg.joint_trajectory.header.stamp = ros::Time::now();
    trajectory_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++) 
    {
        stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
        trajectory_msg.joint_trajectory.joint_names.push_back(jointName.str());
    }

    goal.trajectory.points.clear();
    trajectory_msg.joint_trajectory.points.clear();

    int number1 = number_point, number2 = number_distance;
    target_euler_angle = current_euler_angle;
    for (int i = 0; i < number1; i++) 
    {
        // translation
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;
        // rotation
        target_euler_angle(0) += ((yaw - current_euler_angle(0)) / number1);
        target_euler_angle(1) += ((pitch - current_euler_angle(1)) / number1);
        target_euler_angle(2) += ((roll - current_euler_angle(2)) / number1);
        current_tf_quat = current_tf_quat.slerp(target_quat_tf, 0.1);
        tf::Matrix3x3 tf_rotation_matrix1(current_tf_quat);
        tf_rotation_matrix1.getRPY(R, P, Y);
        cout << "tf: Euler angle RPY 2: " << "\n" << R * 180 / Pi << " " << P * 180 / Pi << " " << Y * 180 / Pi << endl;

        Eigen::Quaterniond eigen_target_q(current_tf_quat.w(), current_tf_quat.x(), current_tf_quat.y(), current_tf_quat.z());
        rotation_matrix = eigen_target_q.toRotationMatrix();
        
        // Euler to Rotation Matrix
        // Eigen::AngleAxisd rollAngle(target_euler_angle(2), Eigen::Vector3d::UnitX());
        // Eigen::AngleAxisd pitchAngle(target_euler_angle(1), Eigen::Vector3d::UnitY());
        // Eigen::AngleAxisd yawAngle(target_euler_angle(0), Eigen::Vector3d::UnitZ());
        // rotation_matrix = yawAngle * pitchAngle * rollAngle;
        // cout << "rotation matrix2: " << "\n" << rotation_matrix << endl;
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                transformation(i,j) = rotation_matrix(i,j);
            }
        }
        // transformation()
        Eigen::VectorXd q(6);
        q = parser.Inverse(transformation, qPre);
        // cout << "=========" << endl;
        if (q(0) > 10) continue;

        // printf("loops:%d",i);
        for (int j = 0; j < number2; j++) {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++) {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number2 * j);
                point.velocities.push_back(0.0); 
                point.accelerations.push_back(0.0);
            }
            point.time_from_start = ros::Duration();
            goal.trajectory.points.push_back(point);
            trajectory_msg.joint_trajectory.points.push_back(point);
        }
        // printf("\n\njoint values : %d\n",i);
        // std::cout << q  << "\n" << std::endl;
        qPre = q;
    }
    cout << "trajectory points number: " << trajectory_msg.joint_trajectory.points.size() << endl;

    double trajectory_velocity_scaling_ = 0.3;
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // iptp.computeTimeStamps(robot_trajectory, 0.5);
    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt,trajectory_velocity_scaling_);
    ROS_INFO("Computed time stamped %s", IptpSuccess ? "SUCCED" : "FAILED");
    rt.getRobotTrajectoryMsg(trajectory_msg);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Send goal to kinova");
    // sleep(1.0);
    confirmToAct();
    group.execute(plan);
    // sleep(1.0);
    // ac.sendGoal(goal);
    // ac.waitForResult(ros::Duration(10));
    ROS_INFO_ONCE("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");
}