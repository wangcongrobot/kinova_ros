/* Copyright SIA FreeDream.
   Author: HeKai
   Desc:   A notice class to publish and subscrib message from ROS.
*/

#ifndef NOTICE_PUB_SUB_H
#define NOTICE_PUB_SUB_H

// C++ 
#include <fstream>
#include <iomanip>
#include <iostream>

// ROS 
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// User
#include "id_data_msgs/ID_Data.h"
#include "darknet_ros_msgs/TargetPoint.h" // kinect info
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h" //using for notie event

#include "globals.h"

typedef int ErrorCode;

class notice_pub_sub {
public:

  /** \brief Constructor */
  notice_pub_sub();
  
  /** \brief Notice callback function */
  boost::function<void(const id_data_msgs::ID_Data::ConstPtr &)>
      notice_pub_sub_msgCallbackFun;
      
  /** \brief Notice publisher function */
  void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
  
  /** \brief Notice print function */
  void notice_display(id_data_msgs::ID_Data notice_msg, bool set);
  
  /** \brief Notice subscriber spinner function */
  void notice_sub_spinner(char set);
  
  /** \brief Clear the id_data */
  void notice_data_clear(id_data_msgs::ID_Data* test);
  
  // /** \brief Error process */
  // void error_deal(int error_num);
  
  // /** \brief Message confirm and waiting for act finished */
  // ErrorCode notice_pub_sub::hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
  //   bool* msg_rec_flag, bool* finished_flag, notice_pub_sub* notice_test, std::string task)
    
  /** \brief Action client */ 
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;

private:
  // // hand flags
  // bool close_hand_flag;
  // bool open_hand_flag;
  // bool hand_msg_rec_flag;
  // bool hand_act_finished_flag;
  // bool soft_close_hand_flag;
  // bool switch_suck_flag;
  // bool begin_suck_flag;
  // bool stop_suck_flag;
  
  // // main loop flags: arm control section,id=4
  // bool arm_start_fetch_flag;  // data[0]=1
  // bool arm_stop_fetch_flag;   // data[0]=0
  // bool arm_keep_fetch_flag;   // data[0]=2
  // bool arm_release_obj_flag;  // data[0]=3
  // bool arm_msg_rec_flag;      // data[0]=14
  // bool arm_act_finished_flag; // data[0]=15
  // bool use_gripper_flag;       // alvin, grasp or suck depend on target
  
  // // return error number
  // int error_num;
  
  // ROS
  ros::NodeHandle notice_handle;
  ros::Subscriber notice_subscriber;
  ros::Publisher notice_publisher;
  ros::SubscribeOptions notice_ops;
  ros::AsyncSpinner *notice_spinner;
  ros::CallbackQueue notice_callbackqueue;
  
  /** \brief Notice message callback */
  void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg);
};
  /** \brief Initilize the flags and values */
  void init_flags();

#endif // NOTICE_PUB_SUB_H
