
#ifndef NOTICE_PUB_SUB_H
#define NOTICE_PUB_SUB_H

#include "ros/callback_queue.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ros/ros.h>
#include <id_data_msgs/ID_Data.h>


class notice_pub_sub {
public:
  boost::function<void(const id_data_msgs::ID_Data::ConstPtr &)>
      notice_pub_sub_msgCallbackFun;

  notice_pub_sub();
  void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
  void notice_display(id_data_msgs::ID_Data notice_msg, bool set);
  void notice_sub_spinner(char set);

private:
  ros::NodeHandle notice_handle;
  ros::Subscriber notice_subscriber;
  ros::Publisher notice_publisher;
  ros::SubscribeOptions notice_ops;
  ros::AsyncSpinner *notice_spinner;
  ros::CallbackQueue notice_callbackqueue;
  void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg);
};

int main_MsgConform_ActFinishedWait(id_data_msgs::ID_Data *notice_data_test,
                                    bool *msg_rec_flag, bool *finished_flag,
                                    notice_pub_sub *notice_test);
void error_deal(int error_nu);
void notice_data_clear(id_data_msgs::ID_Data *test);



#endif