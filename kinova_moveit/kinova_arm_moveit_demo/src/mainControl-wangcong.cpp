/*
=========================JDX pick and place chanllenge===============
==========================Kinova arm test============================
This is the main program for handling picking and place task in JDX
challenge.
*/

#include "add_scene_objects.h"    // handle scene obstacles
#include "pick_place.h"

#include "functions.h"
#include "notice_pub_sub.h"


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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaco_moveit_control_main");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    kinova::PickPlace pick_place(nh); // official pick and place class
    notice_pub_sub notice_test;       // initial a nitice class
    int loop_hz = 100;
    ros::Rate loop_rate(loop_hz);
    
    ros::Subscriber sub_joint_values
    = nh.subscribe("/j2n6s300_driver/out/joint_state", 10, currentJointValuesCallback);


    id_data_msgs::ID_Data notice_data; // initial notice data
    notice_data.id = 0;
    for (char i = 0; i < 8; i++) notice_data.data[i] = 0;

    poseInit(); // initial predifined poses

    // /**********************ADD COLLISION***************************/
    ROS_INFO("Add collision objects  into the world (kinect and mobile base)");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    build_workScene buildWorkScene(nh);
    handleCollisionObj(buildWorkScene); // add objects
    planning_scene_interface.addCollisionObjects(
        buildWorkScene.collision_objects); // add collision objects into world
    ROS_INFO("Collision setup finished");

    // /************************STAND BY POSE***************************/
    moveit::planning_interface::MoveGroup group("arm");

    // ros::spinOnce();
    // moveLineFromCurrentState(-0.1,0,-0.1,90,-30,5,100,2);
    // exit(0);

    /*************************ARM tASK LOOP***************/
    int loop_cnt = 0;
    int wait_count = 0;
    string pose_name = "NO DEFINE";
    while (ros::ok()) {
        loop_cnt++;
        if (loop_cnt % 100 == 0) {
            ROS_INFO("Begin main loop, ready to receive command from ge_test");
            // notice main loop that received msg
            ROS_INFO_STREAM("arm_start_fetch flag: " << arm_start_fetch_flag);
        }

        /*************************GRASP*********************************/
        if (arm_start_fetch_flag) {
            ROS_INFO("Start grasp objects from shelf/desk");
            notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 14;
            notice_test.notice_pub_sub_pulisher(notice_data);

            if (use_gripper_flag) {
                pregrasp_pose = grasp_pose;
                pregrasp_pose.position.y += PREGRASP_OFFSET;
                // ROS_INFO_STREAM("pregrasp pose: " << pregrasp_pose);

                // switch to grasp mode
                if (hand_current_mode != "grasp") {
                    notice_data.id = 1;
                    notice_data.data[0] = 0;
                    notice_test.notice_pub_sub_pulisher(notice_data);
                    ROS_INFO("switch to grasp mode (1 4)");

                    // wait switch finish
                    ErrorCode err = hand_MsgConform_ActFinishedWait(&notice_data,
                        &hand_msg_rec_flag, &hand_act_finished_flag, &notice_test, "SWITCH");
                    error_deal(err);
                    hand_current_mode = "grasp";
                }

                // 1. pregrasp
                geometry_msgs::Pose start;
                if (DEBUG) {
                    start = group.getCurrentPose().pose; // virtual pose
                } else {
                    start = pick_place.get_ee_pose(); // real pose from driver info.
                }
                pose_name = "PREGRASP POSE";
                confirmToAct(start, pregrasp_pose, pose_name);
                moveToTarget(pregrasp_pose); // plan to pre-grasp pose

                // 2. move forward
                if (DEBUG) {
                    start = pregrasp_pose; // virtual pose
                } else {
                    start = pick_place.get_ee_pose(); // real pose from driver info.
                    start.orientation = pregrasp_pose.orientation;
                }
                geometry_msgs::Pose goal = grasp_pose;
                pose_name = "TARGET AND GRASP";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal);

                // 3. close hand
                notice_data_clear(&notice_data);
                notice_data.id = 1;
                notice_data.data[0] = 3;
                notice_test.notice_pub_sub_pulisher(notice_data);
                ROS_WARN("notice hand to close (1 3)");
                ErrorCode err = hand_MsgConform_ActFinishedWait(&notice_data, &hand_msg_rec_flag,
                    &hand_act_finished_flag, &notice_test, "FETCH");
                error_deal(err);

                // 4. move to stand-by pose
                if (DEBUG) {
                    start = grasp_pose;
                } else {
                    start = pick_place.get_ee_pose();
                    start.orientation = pregrasp_pose.orientation;
                }

                goal = start;
                goal.position.z += 0.04;
                pose_name = "MOVE TO REST (UP)";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal);

                if (DEBUG) {
                    start = goal;
                } else {
                    start = pick_place.get_ee_pose();
                    start.orientation = pregrasp_pose.orientation;
                }
                goal = start;
                goal.position.y += PREGRASP_OFFSET;
                pose_name = "MOVE OT REST (BACK)";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal);

                pose_name = "REST POSE";
                confirmToAct(goal, gripper_rest_pose, pose_name);
                moveToTarget(gripper_rest_pose);

            } else {
                // suck mode, use_gripper_flag = false
                presuck_pose = suck_pose;
                presuck_pose.position.z += PRESUCK_OFFSET_LOW;
                presuck_pose.position.y += PREGRASP_OFFSET;

                // 2.swich to suck mode
                if (hand_current_mode != "suck") {
                    notice_data.id = 1;
                    notice_data.data[0] = 4;
                    notice_test.notice_pub_sub_pulisher(notice_data);
                    ROS_INFO("switch to suck mode (1 4)");
                    // wait switch finish
                    ErrorCode err = hand_MsgConform_ActFinishedWait(&notice_data,
                        &hand_msg_rec_flag, &hand_act_finished_flag, &notice_test, "SWITCH");
                    error_deal(err);
                    hand_current_mode = "suck";
                }

                // 1. presuck
                geometry_msgs::Pose start;
                if (DEBUG) {
                    start = group.getCurrentPose().pose;
                } else {
                    start = pick_place.get_ee_pose(); // real pose from driver info.
                }
                pose_name = "PRESUCK POSE";
                confirmToAct(start, presuck_pose, pose_name);
                moveToTarget(presuck_pose); // plan to pre-grasp pose

                // 3. move forward and downside
                if (DEBUG) {
                    start = presuck_pose;
                } else {
                    start = pick_place.get_ee_pose();
                    start.orientation = presuck_pose.orientation;
                }
                geometry_msgs::Pose goal = presuck_pose;
                goal.position.y -= PREGRASP_OFFSET;
                pose_name = "SUCK FORWARD";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal); // forward

                if (DEBUG) {
                    start = goal;
                } else {
                    start = pick_place.get_ee_pose();
                    start.orientation = presuck_pose.orientation;
                }
                goal = suck_pose;
                pose_name = "SUCK DOWN";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal); // down

                // 4. begin suck
                notice_data_clear(&notice_data);
                notice_data.id = 1;
                notice_data.data[0] = 8;
                notice_test.notice_pub_sub_pulisher(notice_data);
                ROS_INFO("notice sucker to suck (1 8)");

                // wait suck finish
                // TODO:no suck feedback
                ErrorCode err = hand_MsgConform_ActFinishedWait(&notice_data, &hand_msg_rec_flag,
                    &hand_act_finished_flag, &notice_test, "FETCH");
                error_deal(err);

                // 5. move to stand-by pose
                if (DEBUG) {
                    start = suck_pose;
                } else {
                    start = pick_place.get_ee_pose();
                    start.orientation = presuck_pose.orientation;
                }
                goal = start;
                goal.position.z += 0.08;
                pose_name = "SUCK UP BACK";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal); // up

                if (DEBUG) {
                    start = goal; // last goal
                } else {
                    start = pick_place.get_ee_pose();
                    start.orientation = presuck_pose.orientation;
                }
                goal = start;
                goal.position.y += PREGRASP_OFFSET;
                pose_name = "SUCK BACKWARD BACK";
                confirmToAct(start, goal, pose_name);
                moveLineTarget(start, goal); // back

                pose_name = "SUCK REST POSE";
                confirmToAct(goal, sucker_rest_pose, pose_name);
                moveToTarget(sucker_rest_pose);
            }

            // notice main loop that fetch action finished
            notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 15;
            notice_test.notice_pub_sub_pulisher(notice_data);
            arm_start_fetch_flag = false;
            ROS_INFO("grasp task finished");
        }

        /*************************KEEP*********************************/
        if (arm_keep_fetch_flag) {
            notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 14;
            notice_test.notice_pub_sub_pulisher(notice_data);
            ROS_INFO("keep task begin");

            notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 15;
            notice_test.notice_pub_sub_pulisher(notice_data);
            arm_keep_fetch_flag = false;
            ROS_INFO("keep task finish");
        }

        /*************************RELEASE*********************************/
        if (arm_release_obj_flag) {

            // notice main loop that received msg
            notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 14;
            notice_test.notice_pub_sub_pulisher(notice_data);
            ROS_ERROR("---------- 4 14");

            // 1. place: move to place pose
            if (use_gripper_flag) {
                pose_name = "GRIPPER PLACE POSE";
                confirmToAct(gripper_place_pose, pose_name);
                moveToTarget(gripper_place_pose);
            } else {
                pose_name = "SUCKER PLACE POSE";
                confirmToAct(sucker_place_pose, pose_name);
                moveToTarget(sucker_place_pose);
            }

            // 2. open hand
            notice_data_clear(&notice_data);
            if (use_gripper_flag) {
                notice_data.id = 1;
                notice_data.data[0] = 0;
            } else {
                notice_data.id = 1;
                notice_data.data[0] = 9;
            }

            // publish and wait for hand task finish signal
            ErrorCode err = hand_MsgConform_ActFinishedWait(
                &notice_data, &hand_msg_rec_flag, &hand_act_finished_flag, &notice_test, "RELEASE");
            error_deal(err);

            // back to rest pose
            if (use_gripper_flag) {
                pose_name = "GRIPPER REST POSE";
                confirmToAct(gripper_place_pose, gripper_rest_pose, pose_name);
                moveToTarget(gripper_rest_pose);
            } else {
                pose_name = "SUCKER REST POSE";
                confirmToAct(sucker_place_pose, sucker_rest_pose, pose_name);
                moveToTarget(sucker_rest_pose);
            }

            // notice main loop that place action finished
            notice_data_clear(&notice_data);
            notice_data.id = 4;
            notice_data.data[0] = 15;
            notice_test.notice_pub_sub_pulisher(notice_data);
            ROS_INFO("placing action finished\n");
            arm_release_obj_flag = false;
            // sleep(10); // TODO change time to wait
        }

        notice_test.notice_sub_spinner(1);
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}
