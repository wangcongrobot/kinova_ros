/********************************************************
*
*
* Final version code for arm control
* Two operation mode: sweep and suck
*
*
**********************************************************/

// Std C++ headers
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// custom headers

#include "functions.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include "jrc18sia_target_info.h" // Objects
#include <actionlib/client/simple_action_client.h>
#include <add_scene_objects.h> // handle scene obstacles
#include <geometry_msgs/Pose.h>
#include <jrc18sia_motion_planner/jrc18sia_motion_planner.h>

#define PRESUCK_OFFSET 0.15       //  pre-scuk positon in y direction
#define POST_SWEEP_Y -0.30        // distance at the edge of the desk in y direction
#define SWEEP_CENTER_TO_DESK 0.08 // distance between hand center to desk plane in z direction
#define SUCK_CENTER_TO_DESK 0.07  // distance between hand center to desk plane in z direction

using namespace std;

bool dashgo_act_finished_flag = false;

// main loop globals: arm control section, id=4
bool arm_start_sweep_flag  = false; // data[0]=1
bool arm_stop_sweep_flag   = false; // data[0]=0
bool arm_start_suck_flag   = false;
bool arm_stop_suck_flag    = false;
bool arm_msg_rec_flag      = false; // data[0]=14
bool arm_act_finished_flag = false; // data[0]=15

// sucker
bool sucker_msg_rec_flag      = false;
bool sucker_act_finished_flag = false;

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

JRCTarget  jrc_targets;         // objects class
TargetInfo current_target_info; // Current target from kinect

/***************************NOTICE CLASS****************************/
typedef int ErrorCode;
class notice_pub_sub
{

  private:
	ros::NodeHandle       notice_handle;
	ros::Subscriber       notice_subscriber;
	ros::Publisher        notice_publisher;
	ros::SubscribeOptions notice_ops;
	ros::AsyncSpinner *   notice_spinner;
	ros::CallbackQueue    notice_callbackqueue;

	// Get from ROS parameter
	double calibration_adjust_x;
	double calibration_adjust_y;
	double calibration_adjust_z;

  public:
	boost::function<void(const id_data_msgs::ID_Data::ConstPtr &)>            notice_pub_sub_msgCallbackFun;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac;

	notice_pub_sub()
	{
		notice_pub_sub_msgCallbackFun = boost::bind(&notice_pub_sub::notice_msgCallback, this, _1);
		notice_ops = ros::SubscribeOptions::create<id_data_msgs::ID_Data>("/notice", 10, notice_pub_sub_msgCallbackFun,
		                                                                  ros::VoidPtr(), &notice_callbackqueue);
		notice_subscriber = notice_handle.subscribe(notice_ops);
		notice_spinner    = new ros::AsyncSpinner(1, &notice_callbackqueue);

		notice_publisher = notice_handle.advertise<id_data_msgs::ID_Data>("/notice", 10);
		// jaco_estop_client = notice_handle.serviceClient<wpi_jaco_msgs::EStop>(
		//     "/jaco_arm/software_estop");
		notice_handle.getParam("calibration_adjust_x", calibration_adjust_x);
		notice_handle.getParam("calibration_adjust_y", calibration_adjust_y);
		notice_handle.getParam("calibration_adjust_z", calibration_adjust_z);
		std::cout << "calibration_adjust : " << calibration_adjust_x << " " << calibration_adjust_y << " "
		          << calibration_adjust_z << std::endl;
	}

	void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data) { notice_publisher.publish(id_data); }

	void notice_display(id_data_msgs::ID_Data notice_msg, bool set)
	{

		if (set)
		{
			printf("REC Notice message,ID: %d,Data: ", notice_msg.id);
			for (char i = 0; i < 8; i++) {
				printf("%d ", notice_msg.data[i]);
				if (i == 7)
					printf("\n");
			}
		}
	}

	void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg)
	{

		id_data_msgs::ID_Data notice_message;
		notice_message.id = 0;
		for (char i = 0; i < 8; i++) notice_message.data[i] = 0;

		notice_message.id = notice_msg->id;
		for (char i = 0; i < 8; i++) notice_message.data[i] = notice_msg->data[i];

		notice_pub_sub::notice_display(notice_message, true);

		// dashgo
		if (notice_message.id == 2 && notice_message.data[0] == 8)
		{
			dashgo_act_finished_flag = true;
			std::cout << "dashgo_act_finished_flag=true" << std::endl;
		}

		// adjust dashgo

		// arm message callback
		if (notice_message.id == 4 && notice_message.data[0] == 1)
		{
			int id                                 = notice_message.data[1]; // object want to fetch
			current_target_info.header.target_id   = jrc_targets.jrc_target_info[id].header.target_id;
			current_target_info.header.target_name = jrc_targets.jrc_target_info[id].header.target_name;
			current_target_info.header.grasp_type  = jrc_targets.jrc_target_info[id].header.grasp_type;
			current_target_info.shape_info.length  = jrc_targets.jrc_target_info[id].shape_info.length;
			current_target_info.shape_info.width   = jrc_targets.jrc_target_info[id].shape_info.width;
			current_target_info.shape_info.height  = jrc_targets.jrc_target_info[id].shape_info.height;

			float x = notice_message.data[2] / 1000.0 + calibration_adjust_x; // object pose, coordinate x
			float y = notice_message.data[3] / 1000.0 + calibration_adjust_y; // object pose, coordinate y
			float z = notice_message.data[4] / 1000.0;                        // object pose, coordinate z
			// TODO
			// correction for z axis
			if (z < (DESK_HEIGHT_MIDDLE - 0.1)) // 0.6 - 0.41 = 0.19
			{
				current_target_info.header.table_type = LOW;
				// z                                     = DESK_HEIGHT_LOW + SWEEP_CENTER_TO_DESK +
				// calibration_adjust_z;
				std::cout << "---------TABLE LOW -----------" << z << std::endl;
			}
			if (z > (DESK_HEIGHT_MIDDLE - 0.1) && z < (DESK_HEIGHT_HIGH - 0.1)) // 0.19 ~ 0.49
			{
				current_target_info.header.table_type = MIDDLE;
				// z                                     = DESK_HEIGHT_MIDDLE + SWEEP_CENTER_TO_DESK +
				// calibration_adjust_z;
				std::cout << "------- TABLE MIDDLE---------" << z << std::endl;
			}
			if (z > (DESK_HEIGHT_HIGH - 0.1)) // 0.9 - 0.41 = 0.49
			{
				current_target_info.header.table_type = HIGH;
				// z                                     = DESK_HEIGHT_HIGH + SWEEP_CENTER_TO_DESK +
				// calibration_adjust_z;
				std::cout << "-------------TABLE HIGH -----------" << z << std::endl;
			}

			std::cout << "Current task information, ID:" << id << ", name:" << current_target_info.header.target_name
			          << ", grasp type:" << current_target_info.header.grasp_type << std::endl;
			std::cout << "Object pose form kinect : (x,y,z)" << x << " " << y << " " << z << std::endl;

			if (current_target_info.header.grasp_type == SWEEP && x <= 0.5 && x > 0.1 && y <= -0.40 && y > -0.80 &&
			    z >= -0.10)
			{
				sweep_pose.position.x = x;
				sweep_pose.position.y = y + 0.065;
				sweep_pose.position.z = z;
				arm_start_sweep_flag  = true;
			}
			if (current_target_info.header.grasp_type == SUCK && x <= 0.5 && x > 0.1 && y <= -0.40 && y > -0.80 &&
			    z >= -0.10)
			{
				suck_pose.position.x = x;
				suck_pose.position.y = y + 0.02;
				suck_pose.position.z = z;
				arm_start_suck_flag  = true;
			}
		}

		// stop the current task
		if (notice_message.id == 4 && notice_message.data[0] == 0)
		{
			arm_stop_sweep_flag = true;
		}

		// sucker feedback
		if (notice_message.id == 1 && notice_message.data[0] == 2)
		{
			sucker_act_finished_flag = true;
		}
		if (notice_message.id == 1 && notice_message.data[0] == 14)
		{
			sucker_msg_rec_flag = true;
		}
	}

	void notice_sub_spinner(char set)
	{
		if (set == 1)
			notice_spinner->start();
		if (set == 0)
			notice_spinner->stop();
	}
	void notice_data_clear(id_data_msgs::ID_Data *test)
	{
		test->id = 0;
		for (int i = 0; i < 8; i++) test->data[i] = 0;
	}
};

void poseInit()
{
	//   geometry_msgs::Pose temp;
	//   temp.orientation = tf::createQuaternionMsgFromRollPitchYaw(
	//       1.5, -0.01, -0.1); // grasp orientation

	// tf::Quaternion temp_q;
	/**@brief Set the quaternion using fixed axis RPY
	* @param roll Angle around X
	* @param pitch Angle around Y
	* @param yaw Angle around Z*/

	// pose before sweep or suck for each floor
	// home_pose.orientation = sweep_pose.orientation;
	// home_pose_high.orientation = sweep_pose.orientation;
	home_pose_high.position.x = 0.18;
	home_pose_high.position.y = -0.3;
	home_pose_high.position.z = DESK_HEIGHT_HIGH + 0.1;

	// home_pose_mid.orientation = sweep_pose.orientation;
	home_pose_mid.position.x = 0.18;
	home_pose_mid.position.y = -0.3;
	home_pose_mid.position.z = DESK_HEIGHT_MIDDLE + 0.2;

	// home_pose_low.orientation = sweep_pose.orientation;
	home_pose_low.position.x = 0.18;
	home_pose_low.position.y = -0.3;
	home_pose_low.position.z = DESK_HEIGHT_LOW + 0.4;

	home_pose = home_pose_high; // Default start home pose is high

	// box_pose.orientation = suck_pose.orientation;
	box_pose.position.x = 0.23;
	box_pose.position.y = -0.3;
	box_pose.position.z = 0.4;
}

void sweep();
void suck();

// Main program
int main(int argc, char **argv)
{
	ros::init(argc, argv, "jaco_moveit_control_main");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	JRCMotionPlanner motion_planner(nh);
	float            current_desk_height; // which floor the current task relates to

	// exit(0);
	notice_pub_sub        notice_test; // initial a notice class
	int                   loop_hz = 100;
	ros::Rate             loop_rate(loop_hz);
	id_data_msgs::ID_Data notice_data; // initial an empty data

	poseInit(); // initial predifined poses

	// Add collisions
	std::cout << "Add collision objects into the world (kinect and mobile base)" << std::endl;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	build_workScene                                    buildWorkScene(nh);
	handleCollisionObj(buildWorkScene);                                             // add object
	planning_scene_interface.addCollisionObjects(buildWorkScene.collision_objects); // add collision objects to world
	std::cout << "Collision setup finished" << std::endl;

	// Initial pose
	double x = 0;
	double y = 0;
	double z = 0;
	double R = 85;
	double P = 5;
	double Y = 5;
	motion_planner.cartesionPathPlanner(x, y, z, R, P, Y); // Adjust orientation
	motion_planner.setJointValueTarget(0, 1.0);

	int loop_cnt   = 0;
	int wait_count = 0;

	string pose_name = "NO DEFINE";
	while (ros::ok()) {
		loop_cnt++;
		if (loop_cnt % 100 == 0)
		{
			std::cout << "Main loop, ready to receive command from ge_test" << std::endl;
		}

		// sweep mode
		if (arm_start_sweep_flag)
		{
			double back_low_y = 0;
			std::cout << "-----------------------------------------------" << std::endl;
			std::cout << "\nStart sweep objects from shelf/desk\n" << std::endl;
			std::cout << "-----------------------------------------------" << std::endl;

			notice_test.notice_data_clear(&notice_data);
			notice_data.id      = 4;
			notice_data.data[0] = 14;
			notice_test.notice_pub_sub_pulisher(notice_data);

			// move back to home pose
			motion_planner.setJointValueTarget(0, -1.0);

			// 0. home_pose_start pose

			switch (current_target_info.header.table_type)
			{
				case LOW:
					home_pose           = home_pose_low;
					current_desk_height = DESK_HEIGHT_LOW + 0.02;
					back_low_y          = -0.05;
					// motion_planner.cartesionPathPlanner(0.05,-0.1, -0.15);
					R = 95;
					P = 5;
					Y = 2;
					std::cout << "\nhome_pose_low\n" << std::endl;
					break;

				case MIDDLE:
					home_pose           = home_pose_mid;
					current_desk_height = DESK_HEIGHT_MIDDLE;
					// motion_planner.cartesionPathPlanner(0.05, -0.1, 0);
					R = 85;
					P = 5;
					Y = 2;
					std::cout << "\nhome_pose_mid\n" << std::endl;
					break;

				case HIGH:
					home_pose           = home_pose_high;
					current_desk_height = DESK_HEIGHT_HIGH;
					// motion_planner.cartesionPathPlanner(0.05, -0.1, 0.2);
					R = 85;
					P = 5;
					Y = 2;
					std::cout << "\nhome_pose_high\n" << std::endl;
					break;
			}
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = home_pose.position.x - current_pose.position.x;
			y            = home_pose.position.y - current_pose.position.y;
			z            = home_pose.position.z - current_pose.position.z;
			cout << "Relative Cartesian distance:" << x << " " << y << " " << z << endl;
			//   motion_planner.confirmToAct(home_pose, current_pose);

			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);

			// 1. presweep
			pose_name                = "PRESWEEP";
			presweep_pose            = sweep_pose; // sweep pose is the target pose from kinect
			presweep_pose.position.y = POST_SWEEP_Y + back_low_y;
			presweep_pose.position.z = current_desk_height + current_target_info.shape_info.height + 0.15;

			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = presweep_pose.position.x - current_pose.position.x;
			y            = presweep_pose.position.y - current_pose.position.y;
			z            = presweep_pose.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, presweep_pose, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			// motion_planner.cartesionPathPlanner(x, y, z);
			motion_planner.confirmToAct();
			// motion_planner.moveLineTarget(pose1);

			// 2. move forward
			geometry_msgs::Pose pose2;
			pose2            = presweep_pose;
			pose2.position.y = sweep_pose.position.y;
			pose_name        = "FORWARD";

			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = pose2.position.x - current_pose.position.x;
			y            = pose2.position.y - current_pose.position.y;
			z            = pose2.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose2, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			// motion_planner.cartesionPathPlanner(x, y, z);
			motion_planner.confirmToAct();

			// 3. move down
			geometry_msgs::Pose pose3;
			pose3            = pose2;
			pose3.position.z = current_desk_height + SWEEP_CENTER_TO_DESK;
			pose_name        = "DOWN";
			current_pose     = motion_planner.getCurrentPoseFromDriver();
			x                = pose3.position.x - current_pose.position.x;
			y                = pose3.position.y - current_pose.position.y;
			z                = pose3.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose3, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			// motion_planner.cartesionPathPlanner(x, y, z);
			motion_planner.confirmToAct();

			// 4. move back
			geometry_msgs::Pose pose4;
			pose4            = pose3;
			pose4.position.y = POST_SWEEP_Y + back_low_y;
			pose_name        = "BACK";
			current_pose     = motion_planner.getCurrentPoseFromDriver();
			x                = pose4.position.x - current_pose.position.x;
			y                = pose4.position.y - current_pose.position.y;
			z                = pose4.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose4, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			// motion_planner.cartesionPathPlanner(x, y, z);
			motion_planner.confirmToAct();

			// 5. move to home pose
			pose_name = "HOME POSE";
			// if (home_pose.position.z == home_pose_low.position.z)
			// {
			// 	x = 0;
			// 	y = 0;
			// 	z = 0.1;
			// 	motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			// 	// motion_planner.cartesionPathPlanner(x, y, z);
			// 	motion_planner.confirmToAct();
			// }
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = home_pose.position.x - current_pose.position.x;
			y            = home_pose.position.y - current_pose.position.y;
			z            = home_pose.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, home_pose, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);

			// notice main loop that sweep task finished
			notice_test.notice_data_clear(&notice_data);
			notice_data.id      = 4;
			notice_data.data[0] = 15;
			notice_data.data[1] = current_target_info.header.target_id;
			notice_test.notice_pub_sub_pulisher(notice_data);
			arm_start_sweep_flag = false;

			cout << "Move to scan pose ..." << endl;
			motion_planner.setJointValueTarget(0, 1.0);

			std::cout << "-----------------------------------------------" << std::endl;
			std::cout << "Sweep task finished" << std::endl;
			std::cout << "-----------------------------------------------" << std::endl;
		}

		if (arm_start_suck_flag)
		{
			std::cout << "-----------------------------------------------" << std::endl;
			std::cout << "\nStart suck objects from shelf/desk\n" << std::endl;
			std::cout << "-----------------------------------------------" << std::endl;

			notice_test.notice_data_clear(&notice_data);
			notice_data.id      = 4;
			notice_data.data[0] = 14;
			notice_test.notice_pub_sub_pulisher(notice_data);

			// move to home pose (from scan pose)
			motion_planner.setJointValueTarget(0, -1.0);

			// 0. home_pose_start pose
			switch (current_target_info.header.table_type)
			{
				case LOW:
					home_pose           = home_pose_low;
					current_desk_height = DESK_HEIGHT_LOW;
					// motion_planner.cartesionPathPlanner(0.05,-0.1, -0.15);
					R = 95;
					P = 95;
					Y = 2;
					std::cout << "\nhome_pose_low\n" << std::endl;
					break;

				case MIDDLE:
					home_pose           = home_pose_mid;
					current_desk_height = DESK_HEIGHT_MIDDLE;
					// motion_planner.cartesionPathPlanner(0.05, -0.1, 0);
					R = 85;
					P = 95;
					Y = 2;
					std::cout << "\nhome_pose_mid\n" << std::endl;
					break;

				case HIGH:
					home_pose           = home_pose_high;
					current_desk_height = DESK_HEIGHT_HIGH;
					// motion_planner.cartesionPathPlanner(0.05, -0.1, 0.2);
					R = 85;
					P = 95;
					Y = 2;
					std::cout << "\nhome_pose_high\n" << std::endl;
					break;
			}
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = home_pose.position.x - current_pose.position.x;
			y            = home_pose.position.y - current_pose.position.y;
			z            = home_pose.position.z - current_pose.position.z;
			cout << "Relative Cartesian distance:" << x << " " << y << " " << z << endl;
			//   motion_planner.confirmToAct(current_pose, home_pose_start);

			motion_planner.cartesionPathPlanner(0, 0, 0, R, P, Y);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);

			// 1. pre-suck
			presuck_pose = suck_pose; // sweep pose is the target pose from kinect
			pose_name    = "PRESUCK";

			presuck_pose.position.y += PRESUCK_OFFSET;
			presuck_pose.position.z = current_desk_height + current_target_info.shape_info.height + 0.13;

			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = presuck_pose.position.x - current_pose.position.x;
			y            = presuck_pose.position.y - current_pose.position.y;
			z            = presuck_pose.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, presuck_pose, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			motion_planner.confirmToAct();

			// 2. move forward
			geometry_msgs::Pose pose2;
			pose2            = presuck_pose;
			pose2.position.y = suck_pose.position.y;
			pose_name        = "FORWARD";

			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = pose2.position.x - current_pose.position.x;
			y            = pose2.position.y - current_pose.position.y;
			z            = pose2.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose2, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			motion_planner.confirmToAct();

			// 3. suck
			notice_test.notice_data_clear(&notice_data);
			notice_data.id      = 1;
			notice_data.data[0] = 8;
			notice_test.notice_pub_sub_pulisher(notice_data);
			std::cout << "notice sucker to suck (1 8)" << std::endl;

			// data receive judge
			wait_count = 0;
			while (ros::ok()) {
				if (sucker_msg_rec_flag == true) // 1 14
				{
					sucker_msg_rec_flag = false;
					break;
				}
				wait_count++;
				if (wait_count % 100 == 0) // send msg again after waiting 1s
				{
					ROS_ERROR("Sucker haven't receive message,Retrying...");
					notice_test.notice_pub_sub_pulisher(notice_data);
				}
				notice_test.notice_sub_spinner(1);
				loop_rate.sleep();
			}

			// wait for hand to finished
			wait_count = 0;
			while (ros::ok()) {
				wait_count++;
				if (wait_count % 100 == 0)
				{
					std::cout << "Waiting for suck..." << std::endl;
				}
				if (sucker_act_finished_flag) // 1 2
				{
					sucker_act_finished_flag = false;
					break;
				}
				notice_test.notice_sub_spinner(1);
				loop_rate.sleep();
			}

			// 4. suck down
			geometry_msgs::Pose pose3;
			pose3            = pose2;
			pose3.position.z = current_desk_height + current_target_info.shape_info.height +
			                   SUCK_CENTER_TO_DESK; // 0.06:中心补偿，吸取面到中心
			pose_name    = "DOWN";
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = pose3.position.x - current_pose.position.x;
			y            = pose3.position.y - current_pose.position.y;
			z            = pose3.position.z - current_pose.position.z;

			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose3, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			motion_planner.confirmToAct();

			// 5. move up
			geometry_msgs::Pose pose4;
			pose4 = pose3;
			pose4.position.z += 0.06;
			pose_name    = "UP";
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = pose4.position.x - current_pose.position.x;
			y            = pose4.position.y - current_pose.position.y;
			z            = pose4.position.z - current_pose.position.z;

			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose4, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			motion_planner.confirmToAct();

			// 6. move back
			geometry_msgs::Pose pose5;
			pose5 = pose4;
			pose5.position.y += PRESUCK_OFFSET;
			pose_name    = "BACK";
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = pose5.position.x - current_pose.position.x;
			y            = pose5.position.y - current_pose.position.y;
			z            = pose5.position.z - current_pose.position.z;

			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, pose5, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			motion_planner.confirmToAct();

			// 6. move to box(above)
			pose_name    = "BOX POSE";
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = box_pose.position.x - current_pose.position.x;
			y            = box_pose.position.y - current_pose.position.y;
			z            = 0.0;

			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, box_pose, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);
			motion_planner.confirmToAct();

			// 7. release, suck stop
			notice_test.notice_data_clear(&notice_data);
			notice_data.id      = 1;
			notice_data.data[0] = 9;
			notice_test.notice_pub_sub_pulisher(notice_data);
			arm_start_sweep_flag = false;
			ros::Duration(1).sleep();

			// 8. move to home
			pose_name    = "SUCK HOME POSE";
			current_pose = motion_planner.getCurrentPoseFromDriver();
			x            = home_pose.position.x - current_pose.position.x;
			y            = home_pose.position.y - current_pose.position.y;
			z            = home_pose.position.z - current_pose.position.z;
			cout << x << " " << y << " " << z << endl;
			motion_planner.confirmToAct(current_pose, home_pose, pose_name);
			motion_planner.cartesionPathPlanner(x, y, z, R, P, Y);

			// notice main loop that suck task finished
			notice_test.notice_data_clear(&notice_data);
			notice_data.id      = 4;
			notice_data.data[0] = 15;
			notice_data.data[1] = current_target_info.header.target_id;
			notice_test.notice_pub_sub_pulisher(notice_data);
			arm_start_suck_flag = false;

			motion_planner.setJointValueTarget(0, 1.0);

			std::cout << "-----------------------------------------------" << std::endl;
			std::cout << "Suck task finished" << std::endl;
			std::cout << "-----------------------------------------------" << std::endl;
		}

		notice_test.notice_sub_spinner(1);
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}

/********************* functions ******************************/

void sweep() {}
void suck() {}
