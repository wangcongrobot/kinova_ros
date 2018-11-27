/**************************************************************
* Final version code for arm control
* Two operation mode: sweep and suck
****************************************************************/

// Std C++ headers
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// custom headers

#include "functions.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include <actionlib/client/simple_action_client.h>
#include <add_scene_objects.h> // handle scene obstacles
#include <geometry_msgs/Pose.h>
#include <jrc18sia_motion_planner.h>

#define PRESWEEP_OFFSET 0.2
#define POSITION_TOLERANCE 0.04
#define ORIENTATION_TOLERANCE 0.1
#define MAX_PARALLEL_ATTEMPTS 10
#define DEBUG true
#define SHELF_EDGE_Y -0.4

using namespace std;

// Flag variables
int error_no = 0;
set<int> suck_obj_list;
const string objects[] = { "book", "toothbrush", "can", "strips", "chips", "oreo", "pacific",
    "shampoo", "tissue", "sausage", "toothpaste", "teether", "milk", "jelly" };

bool kinect_target_valid = true;
bool dashgo_act_finished_flag = false;

// kinect distance
double joy_up_distance = 0;
double joy_down_distance = 0;
double joy_left_distance = 0;
double joy_right_distance = 0;
double joy_deep_distance = 0;

// alvin
bool switch_suck_flag = false;
bool begin_suck_flag = false;
bool stop_suck_flag = false;

// main loop globals: arm control section, id=4
bool arm_start_sweep_flag = false; // data[0]=1
bool arm_stop_sweep_flag = false;  // data[0]=0
bool arm_start_suck_flag = false;
bool arm_stop_suck_flag = false;
bool arm_move2_rest_flag = false;   // move to rest pose for recognition
bool arn_move2_home_flag = false;   // move to home pose, prepare for sweep or suck
bool arm_msg_rec_flag = false;      // data[0]=14
bool arm_act_finished_flag = false; // data[0]=15

// hand ~ sucker
bool hand_msg_rec_flag = false;
bool hand_act_finished_flag = false;

geometry_msgs::Pose home_pose_low; // move to home task
geometry_msgs::Pose home_pose_mid;
geometry_msgs::Pose home_pose_high;
geometry_msgs::Pose home_pose;
geometry_msgs::Pose scan_pose; // move to rest task
geometry_msgs::Pose box_pose;  // box pose to place object

// Pose used by Alvin
geometry_msgs::Pose sweep_pose; // Plan target from kinect --global variable
geometry_msgs::Pose presweep_pose;

geometry_msgs::Pose suck_pose; // For suck task
geometry_msgs::Pose presuck_pose;

geometry_msgs::Pose current_pose; // used to indicate arm state

/***************************NOTICE CLASS****************************/
typedef int ErrorCode;
class notice_pub_sub {

private:
    ros::NodeHandle notice_handle;
    ros::Subscriber notice_subscriber;
    ros::Publisher notice_publisher;
    ros::SubscribeOptions notice_ops;
    ros::AsyncSpinner* notice_spinner;
    ros::CallbackQueue notice_callbackqueue;

    // Get from ROS parameter
    double calibration_adjust_x;
    double calibration_adjust_y;
    double calibration_adjust_z;

public:
    boost::function<void(const id_data_msgs::ID_Data::ConstPtr&)> notice_pub_sub_msgCallbackFun;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;

    notice_pub_sub()
    {
        notice_pub_sub_msgCallbackFun = boost::bind(&notice_pub_sub::notice_msgCallback, this, _1);
        notice_ops = ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
            "/notice", 10, notice_pub_sub_msgCallbackFun, ros::VoidPtr(), &notice_callbackqueue);
        notice_subscriber = notice_handle.subscribe(notice_ops);
        notice_spinner = new ros::AsyncSpinner(1, &notice_callbackqueue);

        notice_publisher = notice_handle.advertise<id_data_msgs::ID_Data>("/notice", 10);
        // jaco_estop_client = notice_handle.serviceClient<wpi_jaco_msgs::EStop>(
        //     "/jaco_arm/software_estop");
        notice_handle.getParam("calibration_adjust_x", calibration_adjust_x);
        notice_handle.getParam("calibration_adjust_y", calibration_adjust_y);
        notice_handle.getParam("calibration_adjust_z", calibration_adjust_z);
        std::cout << "calibration_adjust : " << calibration_adjust_x << " " << calibration_adjust_y
                  << " " << calibration_adjust_z << std::endl;
    }

    void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data)
    {
        notice_publisher.publish(id_data);
    }

    void notice_display(id_data_msgs::ID_Data notice_msg, bool set)
    {

        if (set) {
            printf("REC Notice message,ID: %d,Data: ", notice_msg.id);
            for (char i = 0; i < 8; i++) {
                printf("%d ", notice_msg.data[i]);
                if (i == 7) printf("\n");
            }
        }
    }

    void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr& notice_msg)
    {

        id_data_msgs::ID_Data notice_message;
        notice_message.id = 0;
        for (char i = 0; i < 8; i++) notice_message.data[i] = 0;

        notice_message.id = notice_msg->id;
        for (char i = 0; i < 8; i++) notice_message.data[i] = notice_msg->data[i];

        notice_pub_sub::notice_display(notice_message, true);

        /************************Hand messsage**********************/

        /***** Object level *****/

        /***** dashgo_act_finished_flag *****/
        if (notice_message.id == 2 && notice_message.data[0] == 8) {
            dashgo_act_finished_flag = true;
            std::cout << "dashgo_act_finished_flag=true" << std::endl;
        }

        /********************** Adjust mobile bsee *****************/

        /***** communication with Jaco arm *****/

        if (notice_message.id == 4 && notice_message.data[0] == 1
            && suck_obj_list.count(notice_message.data[1]) == 0) {
            std::cout << "Receive command: sweep object" << std::endl;
            float x = notice_message.data[2] / 1000.0
                      + calibration_adjust_x; // object pose, coordinate x
            float y = notice_message.data[3] / 1000.0
                      + calibration_adjust_y; // object pose, coordinate y
            float z = notice_message.data[4] / 1000.0
                      + calibration_adjust_z; // object pose, coordinate z
            std::cout << "receive pose form kinect : " << x << " " << y << " " << z << std::endl;
            if (fabs(x) < 0.5 && y < -0.3) {
                std::cout << "Receive valid object position from kinect" << std::endl;
                sweep_pose.position.x = x;
                sweep_pose.position.y = y;
                sweep_pose.position.z = z;
                // kinect_target_valid = true;
                arm_start_sweep_flag = true;
            }
        }

        if (notice_message.id == 4 && notice_message.data[0] == 1
            && suck_obj_list.count(notice_message.data[1]) > 0) {
            std::cout << "Receive command: suck object" << std::endl;
            float x = notice_message.data[2] / 1000.0
                      + calibration_adjust_x; // object pose, coordinate x
            float y = notice_message.data[3] / 1000.0
                      + calibration_adjust_y; // object pose, coordinate y
            float z = notice_message.data[4] / 1000.0
                      + calibration_adjust_z; // object pose, coordinate z
            if (abs(x) < 0.5 && y < -0.3) {
                std::cout << "Receive valid object position from kinect" << std::endl;
                suck_pose.position.x = x;
                suck_pose.position.y = y + 0.03; // correction
                suck_pose.position.z = z + 0.1;  // correction
                // kinect_target_valid = true;
                arm_start_sweep_flag = true;
            }
        }

        if (notice_message.id == 4
            && notice_message.data[0] == 0) // main loop stop arm to fetch flag
        {
            arm_stop_sweep_flag = true;
        }
    }

    void notice_sub_spinner(char set)
    {
        if (set == 1) notice_spinner->start();
        if (set == 0) notice_spinner->stop();
    }
    void notice_data_clear(id_data_msgs::ID_Data* test)
    {
        test->id = 0;
        for (int i = 0; i < 8; i++) test->data[i] = 0;
    }
};

void poseInit()
{
    geometry_msgs::Pose temp;
    temp.orientation
        = tf::createQuaternionMsgFromRollPitchYaw(1.5, -0.01, -0.1); // grasp orientation

    sweep_pose.orientation = temp.orientation;

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

/********************************* MAIN PROGRAM ****************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaco_moveit_control_main");
    ros::NodeHandle nh;

    // read sweep object list from sweep_object.txt
    ifstream suck_object_file("/home/robot/suck_object_list.txt");
    if (!suck_object_file.is_open()) {
        ROS_ERROR("Failed to open suck object list file");
        return -1;
    }

    std::cout << "Objects to be suck: " << std::endl;
    string obj_str;
    int obj_num;
    while (getline(suck_object_file, obj_str)) {
        sscanf(obj_str.c_str(), "%d", &obj_num);
        suck_obj_list.insert(obj_num);
        ROS_INFO_STREAM("    " << objects[obj_num]);
    }
    suck_object_file.close();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    JRCMotionPlanner motion_planner(nh);

    // exit(0);

    notice_pub_sub notice_test; // initial a notice class
    int loop_hz = 100;
    ros::Rate loop_rate(loop_hz);
    id_data_msgs::ID_Data notice_data; // initial an empty data

    poseInit(); // initial predifined poses

    /**********************ADD COLLISION***************************/
    std::cout << "Add collision objects  into the world (kinect and mobile base)" << std::endl;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    build_workScene buildWorkScene(nh);
    handleCollisionObj(buildWorkScene); // add object
    planning_scene_interface.addCollisionObjects(
        buildWorkScene.collision_objects); // add collision objects to world
    std::cout << "Collision setup finished" << std::endl;

    /*************************ARM TASK LOOP***************/
    motion_planner.cartesionPathPlanner(
        0.0, 0.0, 0.0, 90, 0, 0, 100, 2); // move to pregrasp orientation

    // prepare position
    // motion_planner.cartesionPathPlanner(0.1,-0.1,0.1,100,2);
    int loop_cnt = 0;
    int wait_count = 0;
    string pose_name = "NO DEFINE";
    while (ros::ok()) {
        loop_cnt++;
        if (loop_cnt % 100 == 0) {
            std::cout << "Begin main loop, ready to receive command from ge_test" << std::endl;
        }
        if (arm_start_sweep_flag) {
            // sweep mode
            std::cout << "Start sweep objects from shelf/desk" << std::endl;
            notice_test.notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 14;
            notice_test.notice_pub_sub_pulisher(notice_data);

            // adjust mobile base position if necessary
            /******************************
            int left = 0; // unit: cm
            int right = 0;
            // low --- fisrt floor
            if (sweep_pose.position.z < 0.2) {
              if (sweep_pose.position.y < -0.85) // TODO 0.82
              {
                left = (int)((0.85 - sweep_pose.position.y) * 100);
                ROS_INFO_STREAM(mobile move left);
                notice_data.id = 2;
                notice_data.data[0] = 5;
                notice_data.data[1] = left; // -y direction in arm base frame
                notice_test.notice_pub_sub_pulisher(notice_data);
              } else {
                right = (int)((sweep_pose.position.y - 0.85) * 100);
                notice_data.id = 2;
                notice_data.data[0] = 6;
                notice_data.data[1] = right;
                notice_test.notice_pub_sub_pulisher(notice_data);
              }
            }

            // mid ---- desk
            else if (sweep_pose.position.z >= 0.15 &&
                     sweep_pose.position.z < 0.45) {
              if (sweep_pose.position.y < -0.8) // TODO 0.82
              {
                left = (int)((0.85 - sweep_pose.position.y) * 100);
                ROS_INFO_STREAM(mobile move left);
                notice_data.id = 2;
                notice_data.data[0] = 5;
                notice_data.data[1] = left; // -y direction in arm base frame
                notice_test.notice_pub_sub_pulisher(notice_data);
              } else {
                right = (int)((sweep_pose.position.y - 0.85) * 100);
                notice_data.id = 2;
                notice_data.data[0] = 6;
                notice_data.data[1] = right;
                notice_test.notice_pub_sub_pulisher(notice_data);
              }
            }
            // high -- second floor
            else if (sweep_pose.position.z >= 0.45 &&
                     sweep_pose.position.z < 0.65) {
              if (sweep_pose.position.y < -0.65) // TODO 0.82
              {
                left = (int)((0.85 - sweep_pose.position.y) * 100);
                ROS_INFO_STREAM(mobile move left);
                notice_data.id = 2;
                notice_data.data[0] = 5;
                notice_data.data[1] = left; // -y direction in arm base frame
                notice_test.notice_pub_sub_pulisher(notice_data);
              } else {
                right = (int)((sweep_pose.position.y - 0.85) * 100);
                notice_data.id = 2;
                notice_data.data[0] = 6;
                notice_data.data[1] = right;
                notice_test.notice_pub_sub_pulisher(notice_data);
              }
            }
            *********************/

            // sweep start
            presweep_pose = sweep_pose; // sweep pose is the target pose from kinect
            presweep_pose.position.y += PRESWEEP_OFFSET;
            presweep_pose.position.z += 0.1;
            // ROS_INFO_STREAM("pregrasp pose: " << presweep_pose);

            // 1. pregrasp
            // pose_name = "PRE-SWEEP POSE";
            // confirmToAct(presweep_pose, pose_name);
            // motion_planner.moveToTargetBestTime(presweep_pose); // plan to pre-grasp pose
            // motion_planner.moveLineTarget(presweep_pose);
            double x,y,z;
            geometry_msgs::Pose current_pose = motion_planner.getCurrentPoseFromDriver();
            x = presweep_pose.position.x - current_pose.position.x;
            y = presweep_pose.position.y - current_pose.position.y;
            z = presweep_pose.position.z - current_pose.position.z;
            motion_planner.cartesionPathPlanner(x,y,z,100,2);

            // 2. move forward
            geometry_msgs::Pose start;
            if (DEBUG) {
                start = presweep_pose; // virtual pose
            } else {
                start = motion_planner.getCurrentPoseFromDriver(); // real pose from driver info.
                start.orientation = presweep_pose.orientation;
            }
            geometry_msgs::Pose goal = sweep_pose;
            goal.position.y -= PRESWEEP_OFFSET;
            // pose_name = "FORWARD";
            // confirmToAct(start, goal, pose_name);
            // motion_planner.moveLineTarget(start, goal);
            x = goal.position.x - start.position.x;
            y = goal.position.y - start.position.y;
            z = goal.position.z - start.position.z;
            motion_planner.cartesionPathPlanner(x,y,z,100,2);

            // 3. move down
            if (DEBUG) {
                start = presweep_pose; // virtual pose
                start.position.z += 0.1;
            } else {
                start = motion_planner.getCurrentPoseFromDriver(); // real pose from driver info.
                start.orientation = presweep_pose.orientation;
            }
            goal = sweep_pose;
            // pose_name = "DOWN";
            // confirmToAct(start, goal, pose_name);
            // motion_planner.moveLineTarget(start, goal);
            x = goal.position.x - start.position.x;
            y = goal.position.y - start.position.y;
            z = goal.position.z - start.position.z;
            motion_planner.cartesionPathPlanner(x,y,z,100,2);

            // 4. sweep to box
            if (DEBUG) {
                start = sweep_pose;
            } else {
                start = motion_planner.getCurrentPoseFromDriver();
                start.orientation = presweep_pose.orientation;
            }

            goal = start;
            goal.position.y = SHELF_EDGE_Y;
            // pose_name = "SWEEP (BACK)";
            // confirmToAct(start, goal, pose_name);
            // motion_planner.moveLineTarget(start, goal);
            x = goal.position.x - start.position.x;
            y = goal.position.y - start.position.y;
            z = goal.position.z - start.position.z;
            motion_planner.cartesionPathPlanner(x,y,z,100,2);
            
            // 5. move to home pose
            if (DEBUG) {
                start = sweep_pose;
                start.position.y = SHELF_EDGE_Y;
            } else {
                start = motion_planner.getCurrentPoseFromDriver();
                start.orientation = presweep_pose.orientation;
            }
            // pose_name = "HOME POSE";
            // confirmToAct(presweep_pose, home_pose, pose_name);
            motion_planner.moveToTargetBestTime(home_pose); // nonlinear plan

            // notice main loop that sweep task finished
            notice_test.notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 15;
            notice_test.notice_pub_sub_pulisher(notice_data);
            arm_start_sweep_flag = false;
            std::cout << "Sweep task finished" << std::endl;
        }

        if (arm_start_suck_flag) {
            // suck mode
            presuck_pose = suck_pose;
            presuck_pose.position.z += PRESWEEP_OFFSET;
            presuck_pose.position.y += PRESWEEP_OFFSET;

            // 1. presuck
            // pose_name = "PRESUCK POSE";
            // confirmToAct(presuck_pose, pose_name);
            motion_planner.moveToTargetBestTime(presuck_pose); // plan to pre-grasp pose

            // 2. move forward
            geometry_msgs::Pose start;
            if (DEBUG) {
                start = presuck_pose;
            } else {
                start = motion_planner.getCurrentPoseFromDriver();
                start.orientation = presuck_pose.orientation;
            }
            geometry_msgs::Pose goal = presuck_pose;
            goal.position.y -= PRESWEEP_OFFSET;
            // pose_name = "SUCK FORWARD";
            // confirmToAct(start, goal, pose_name);
            motion_planner.moveLineTarget(start, goal); // forward

            // 3. move down
            if (DEBUG) {
                start = goal;
            } else {
                geometry_msgs::Pose start = motion_planner.getCurrentPoseFromDriver();
                start.orientation = presuck_pose.orientation;
            }
            goal = suck_pose;
            // pose_name = "SUCK DOWN";
            // confirmToAct(start, goal, pose_name);
            motion_planner.moveLineTarget(start, goal); // down

            // 4. suck
            notice_test.notice_data_clear(&notice_data);
            notice_data.id = 1;
            notice_data.data[0] = 8;
            notice_test.notice_pub_sub_pulisher(notice_data);
            std::cout << "notice sucker to suck (1 8)" << std::endl;

            // data receive judge
            wait_count = 0;
            while (ros::ok()) {
                if (hand_msg_rec_flag == true) // 1 14
                {
                    hand_msg_rec_flag = false;
                    break;
                }
                wait_count++;
                if (wait_count % 100 == 0) // send msg again after waiting 1s
                {
                    ROS_ERROR("jaco didn't receive hand msg,Retrying...");
                    notice_test.notice_pub_sub_pulisher(notice_data);
                }
                notice_test.notice_sub_spinner(1);
                loop_rate.sleep();
            }

            // wait for hand to finished
            wait_count = 0;
            while (ros::ok()) {
                wait_count++;
                if (wait_count % 100 == 0) {
                    std::cout << "Waiting for suck..." << std::endl;
                }
                if (hand_act_finished_flag) // 1 2
                {
                    hand_act_finished_flag = false;
                    break;
                }
                notice_test.notice_sub_spinner(1);
                loop_rate.sleep();
            }

            // 5. move up
            if (DEBUG) {
                start = presuck_pose;
            } else {
                start = motion_planner.getCurrentPoseFromDriver();
                start.orientation = presuck_pose.orientation;
            }
            goal = suck_pose;
            goal.position.z += PRESWEEP_OFFSET;
            // pose_name = "SUCK UP ";
            // confirmToAct(start, goal, pose_name);
            motion_planner.moveLineTarget(start, goal); // up

            // 6. move to box(above) if (DEBUG)
            if (DEBUG) {
                start = goal; // last goal
            } else {
                start = motion_planner.getCurrentPoseFromDriver();
                start.orientation = presuck_pose.orientation;
            }
            goal = presuck_pose;
            // pose_name = "SUCK  BACK";
            // confirmToAct(start, goal, pose_name);
            motion_planner.moveLineTarget(start, goal); // back
            // pose_name = "SUCK box POSE";
            // confirmToAct(presuck_pose, sucker_rest_pose);
            motion_planner.moveToTargetBestTime(box_pose);

            // 7. release, suck stop
            // TODO

            // notice main loop that suck task finished
            notice_test.notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 15;
            notice_test.notice_pub_sub_pulisher(notice_data);
            arm_start_sweep_flag = false;
            std::cout << "Suck task finished" << std::endl;
        }

        notice_test.notice_sub_spinner(1);
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}

/********************* functions ******************************/