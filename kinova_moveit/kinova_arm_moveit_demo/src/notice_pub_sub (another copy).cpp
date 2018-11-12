/* Copyright SIA FreeDream.
   Author: HeKai
   Desc:   A notice class to publish and subscrib message from ROS.
*/

#include <notice_pub_sub.h>

notice_pub_sub::notice_pub_sub()
{
    notice_pub_sub_msgCallbackFun = boost::bind(&notice_pub_sub::notice_msgCallback, this, _1);
    notice_ops = ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
        "/notice", 50, notice_pub_sub_msgCallbackFun, ros::VoidPtr(), &notice_callbackqueue);
    notice_subscriber = notice_handle.subscribe(notice_ops);
    notice_spinner = new ros::AsyncSpinner(1, &notice_callbackqueue);

    notice_publisher = notice_handle.advertise<id_data_msgs::ID_Data>("/notice", 50);
  
  init_flags();
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

    /***** communication with hand *****/
    if (notice_message.id == 1 && notice_message.data[0] == 0) // hand open flag
    {
        open_hand_flag = true;
    }
    if (notice_message.id == 1 && notice_message.data[0] == 1) // hand close flag
    {
        close_hand_flag = true;
    }
    if (notice_message.id == 1 && notice_message.data[0] == 2) // hand finished flag
    {
        hand_act_finished_flag = true;
    }
    if (notice_message.id == 1 && notice_message.data[0] == 3) // hand close flag soft
    {
        soft_close_hand_flag = true;
    }
    if (notice_message.id == 1 && notice_message.data[0] == 4) // switch to suck mode
    {
        switch_suck_flag = true;
    }
    if (notice_message.id == 1 && notice_message.data[0] == 8) // suck up flag
    {
        begin_suck_flag = true;
    }
    if (notice_message.id == 1 && notice_message.data[0] == 9) // suck release flag
    {
        stop_suck_flag = true;
    }

    if (notice_message.id == 1 && notice_message.data[0] == 14) // hand receive flag
    {
        hand_msg_rec_flag = true;
    }


    // hand collision
    if (notice_message.id == 5 && notice_message.data[0] == 14) // hand collision flag
    {
        if (hand_collision_start_flag) {
            ac->cancelGoal();
            hand_eStop_flag = true;
            ROS_ERROR("hand_eStop_flag received");
        }
    }

    /***** joy *****/
    if (notice_message.id == 3 && notice_message.data[0] == 1 && notice_message.data[1] == 1) {
        joy_low_flag = true;
        ROS_INFO("joy_low_flag=true");
    }
    if (notice_message.id == 3 && notice_message.data[0] == 1 && notice_message.data[1] == 2) {
        joy_mid_flag = true;
    }
    if (notice_message.id == 3 && notice_message.data[0] == 1 && notice_message.data[1] == 3) {
        joy_high_flag = true;
    }

    /***** dashgo_act_finished_flag *****/
    if (notice_message.id == 2 && notice_message.data[0] == 8) {
        dashgo_act_finished_flag = true;
        ROS_WARN_STREAM("dashgo_act_finished_flag=true");
    }

    // joy distance
    if (notice_message.id == 3 && notice_message.data[0] == 27) {
        if (notice_message.data[1] > 0 && notice_message.data[1] <= 15
            && ((notice_message.data[1] + notice_message.data[1]) > 30)) {
            ROS_INFO("joy_left_flag=true");
            joy_left_flag = true;
        } else if (notice_message.data[2] > 0 && notice_message.data[2] <= 15
                   && ((notice_message.data[1] + notice_message.data[1]) > 30)) {
            joy_right_flag = true;
            ROS_INFO("joy_right_flag=true");
        }

        else {
            joy_center_flag = true;
            ROS_INFO("joy_center_flag=true");
        }

        if (notice_message.data[5] >= 15) {
            grasp_by_dashgo_flag = true;
            grasp_by_dashgo_distance = notice_message.data[5] - 10; // deep left
            ROS_INFO("grasp_by_dashgo_distance");
        }

        joy_left_distance = (double)notice_message.data[1] * 0.01;  // left
        joy_right_distance = (double)notice_message.data[2] * 0.01; // right
        joy_up_distance = (double)notice_message.data[3] * 0.01;    // up
        joy_down_distance = (double)notice_message.data[4] * 0.01;  // down
        joy_deep_distance = (double)notice_message.data[5] * 0.01;  // deep

        ROS_INFO("kinect_Y");
    }

    /***** communication with Jaco arm *****/

    if (notice_message.id == 4 && notice_message.data[0] == 1 && notice_message.data[1] == 2) {
        ROS_INFO("Receive command: grasp object");
        float x = notice_message.data[2] / 1000.0; // object pose, coordinate x
        float y = notice_message.data[3] / 1000.0; // object pose, coordinate y
        float z = notice_message.data[4] / 1000.0; // object pose, coordinate z
        ROS_INFO("receive pose form kinect %f, %f, %f", x, y, z);
        if (fabs(x) < 0.5 && y < -0.3) {
            ROS_INFO("Valid object position from kinect");
            grasp_pose.position.x = x;
            grasp_pose.position.y = y;
            grasp_pose.position.z = z + 0.02;
            // kinect_target_valid = true;
            arm_start_fetch_flag = true;
            use_gripper_flag = true; // hard obj
        } else {
            ROS_INFO("Invalid target pose, grasp task cancelled");
        }
    }
    if (notice_message.id == 4 && notice_message.data[0] == 1 && notice_message.data[1] == 8) {
        ROS_INFO("Receive command: suck object");
        float x = notice_message.data[2] / 1000.0; // object pose, coordinate x
        float y = notice_message.data[3] / 1000.0; // object pose, coordinate y
        float z = notice_message.data[4] / 1000.0; // object pose, coordinate z
        if (fabs(x) < 0.5 && y < -0.3) {
            ROS_INFO("Valid object position from kinect");
            suck_pose.position.x = x;
            suck_pose.position.y = y + 0.01; // correction
            suck_pose.position.z = z + 0.12; // correction
            // kinect_target_valid = true;
            arm_start_fetch_flag = true;
            use_gripper_flag = false;
        } else {
            ROS_INFO("Invalid target pose, suck task cancelled");
        }
    }

    if (notice_message.id == 4 && notice_message.data[0] == 0) // main loop stop arm to fetch flag
    {
        arm_stop_fetch_flag = true;
    }
    if (notice_message.id == 4 && notice_message.data[0] == 2) // main loop keep arm to fetch flag
    {
        arm_keep_fetch_flag = true;
    }
    if (notice_message.id == 4 && notice_message.data[0] == 3) // main loop let arm release joy flag
    {
        arm_release_obj_flag = true;
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

void notice_pub_sub::error_deal(int error_nu)
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

ErrorCode notice_pub_sub::hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
    bool* msg_rec_flag, bool* finished_flag, notice_pub_sub* notice_test, string task)
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
        if (wait_count % 20 == 0) // send msg again after waiting 1s
        {
            ROS_ERROR("Hand didn't receive msg, retrying...");
            notice_test->notice_pub_sub_pulisher(notice_data);
        }

        if (wait_count >= 2000) {
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
            if (task == "FETCH") ROS_INFO("Waiting for hand to grasp/suck...");
            if (task == "RELEASE") ROS_INFO("Waiting for hand to open/release...");
            if (task == "SWITCH") ROS_INFO("Waiting for hand to switch mode...");
        }
        notice_test->notice_sub_spinner(1);
        loop_rate.sleep();
    }
next:
    return error_no;
}

void notice_pub_sub::init_flags()
{
  // hand flags
  close_hand_flag = false;
  open_hand_flag = false;
  hand_msg_rec_flag = false;
  hand_act_finished_flag = false;
  soft_close_hand_flag = false;
  switch_suck_flag = false;
  begin_suck_flag = false;
  stop_suck_flag = false;
  
  // main loop flags: arm control section,id=4
  arm_start_fetch_flag = false;  // data[0]=1
  arm_stop_fetch_flag = false;   // data[0]=0
  arm_keep_fetch_flag = false;   // data[0]=2
  arm_release_obj_flag = false;  // data[0]=3
  arm_msg_rec_flag = false;      // data[0]=14
  arm_act_finished_flag = false; // data[0]=15
  use_gripper_flag = false;       // alvin, grasp or suck depend on target
  
  // return error number
  error_num = 0;
  
}


