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

ErrorCode hand_MsgConform_ActFinishedWait(id_data_msgs::ID_Data* notice_data_test,
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

// Planinig with PoseStamped targets
void moveToTarget(const geometry_msgs::PoseStamped& target)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    //   group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(1.0);
    group.setMaxVelocityScalingFactor(0.5);
    group.setNumPlanningAttempts(1);

    group.setStartStateToCurrentState();
    group.setPoseTarget(target);

    moveit::planning_interface::MoveGroup::Plan plan, temp_plan;
    int loops = 10;
    bool success = false;

    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);

    for (int i = 0; i < loops; i++) {
        bool suc = group.plan(temp_plan);
        if (suc) {
            success = true;
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time) {
                plan = temp_plan;
                best_time = current_time;
                ROS_INFO_STREAM(current_time);
            }
        }
    }
    group.execute(plan);
}

// Update--Alvin: Plan with Pose target
void moveToTarget1(const geometry_msgs::Pose& target)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    group.setMaxVelocityScalingFactor(0.5);
    group.setNumPlanningAttempts(3);
    group.setStartStateToCurrentState();
    group.setPoseTarget(target);

    ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink() << " with respect to frame  "
                                        << group.getPlanningFrame() << " with obstacles");

    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setPlanningTime(4.0);

    int loops = 10; // planing tries
    bool success = false;
    bool plan_valid = false;
    int plan_steps = 0;

    // replan until valid paln is returned
    for (int i = 0; i < loops; i++) {
        success = group.plan(my_plan);
        if (success) {
            plan_steps = evaluateMoveitPlan(my_plan);
            if (plan_steps < MAX_PLAN_STEPS) {
                ROS_INFO_STREAM("Try " << i << ": plan found in " << my_plan.planning_time_
                                       << " seconds with " << plan_steps << " steps");
                plan_valid = true;
                break;
            }
            // TODO: choose plans according to measures
        } else {
            ROS_INFO("Plan failed at try: %d", i);
        }
    }
    if (!plan_valid) {
        ROS_INFO("No valid plan found after 20 tries");
        exit(0);
    }

    // Execute the plan
    // ROS_INFO("Print n to execute the plan");
    // string pause_;
    // cin >> pause_;
    // if (pause_ == "n") {
    //     ros::Time start = ros::Time::now();
    //     group.execute(my_plan);
    //     ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    // }
    ROS_INFO("Print n to execute the plan");
    confirmToAct();

    ros::Time start = ros::Time::now();
    group.execute(my_plan);
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}

int evaluateMoveitPlan(moveit::planning_interface::MoveGroup::Plan& plan)
{
    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
    // std::vector<trajectory_msgs::JointTrajectoryPoint> points =
    // trajectory.joint_trajectory.points;
    return trajectory.joint_trajectory.points.size();
}

// Update--Alvin: Plan with Pose target
// Cong　Wang: find the shortest time trajectory
void moveToTarget(const geometry_msgs::Pose& target)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    group.setMaxVelocityScalingFactor(0.5);
    group.setNumPlanningAttempts(3);
    group.setStartStateToCurrentState();
    group.setPoseTarget(target);
    group.setPlanningTime(4.0);

    moveit::planning_interface::MoveGroup::Plan plan, temp_plan;

    ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink() << " with respect to frame  "
                                        << group.getPlanningFrame() << " with obstacles");

    bool plan_valid = false;
    int plan_steps = 0;
    int loops = 100;
    bool success = false;

    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);

    for (int i = 0; i < loops; i++) {
        bool suc = group.plan(temp_plan);
        if (suc) {
            success = true;
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time) {
                plan = temp_plan;
                best_time = current_time;
                ROS_INFO_STREAM(current_time);
            }
        }
    }
    plan_steps = evaluateMoveitPlan(plan);
    if (plan_steps < MAX_PLAN_STEPS) {
        ROS_INFO_STREAM("plan found in " << plan.planning_time_
                                << " seconds with " << plan_steps << " steps");
        plan_valid = true;
    }

    if (!plan_valid) {
        ROS_ERROR("No valid plan found after 20 tries");
        exit(0);
    }

    // Execute the plan
    // ROS_INFO("Print n to execute the plan");
    // string pause_;
    // cin >> pause_;
    // if (pause_ == "n") {
    //     ros::Time start = ros::Time::now();
    //     group.execute(my_plan);
    //     ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    // }
    ROS_INFO("Print n to execute the plan");
    confirmToAct();

    ros::Time start = ros::Time::now();
    group.execute(plan);
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}
// Plan with pre-defined postures
void moveToTarget(const std::string& target_name)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(1.0);
    group.setMaxVelocityScalingFactor(0.5);
    group.setNumPlanningAttempts(1);

    group.setStartStateToCurrentState();
    group.setNamedTarget(target_name);

    moveit::planning_interface::MoveGroup::Plan plan, temp_plan;
    int loops = 10;
    bool success = false;

    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);

    for (int i = 0; i < loops; i++) {
        bool suc = group.plan(temp_plan);
        if (suc) {
            success = true;
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time) {
                plan = temp_plan;
                best_time = current_time;
                ROS_INFO_STREAM(current_time);
            }
        }
    }
    group.execute(plan);
}

// Cartesian line plan
void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
    ROS_INFO("Begin cartesian line plan");
    geometry_msgs::Pose way_pose = start;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(way_pose); // first pose waypoint
    int num_waypoint = 6;
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
    group.setGoalPositionTolerance(0.02);    // 3cm
    group.setGoalOrientationTolerance(0.01); // 5.729576129 * 2 deg
    group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    moveit::planning_interface::MoveGroup::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    int plan_steps = evaluateMoveitPlan(cartesian_plan);
    ROS_INFO_STREAM("Line plan steps: " << plan_steps);
    if (plan_steps < MAX_PLAN_STEPS) {
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
    ROS_INFO_STREAM("Confirm start ---> goal info, press n to start plan");

    if(CONFIRM_ACT)
    {
        string pause_;
        cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Corrent state, begin to plan");
        } else {
            return;
        }
    }

}

void confirmToAct(const geometry_msgs::Pose& goal, const string& str = "NULL")
{
    cout << "\n"
         << "=================MOVE TO " + str + "=================="
         << "\n";
    ROS_INFO_STREAM("Move to target" << goal);
    ROS_INFO_STREAM("Confirm start ---> goal info, press n to start plan");
    if(CONFIRM_ACT)
    {
        string pause_;
        cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Correct state, begin to plan");
        } else {
            return;
        }
    }

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


void confirmToAct()
{
    ROS_INFO_STREAM("Confirm start and end info and press n to start plan");
    if(CONFIRM_ACT)
    {
        string pause_;
        cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Valid info, begin to plan");
        } else {
            return;
        }
    }

}


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
    Eigen::Matrix4d transformation2 = parser.Foward(qPre);
    cout << "transformation2: " << "\n" << transformation2 << endl;

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


void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ,
                              int number_point, int number_distance)
{   
    // MoveIt!
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup::Plan plan;
    group.setStartStateToCurrentState();
    std::vector<double> joint_values = group.getCurrentJointValues();
    cout << "Current pose: " << "\n" << group.getCurrentPose() << endl;
    // cout << "Current RPY: " << "\n" << group.getCurrentRPY() << endl;
    control_msgs::FollowJointTrajectoryGoal goal;
    moveit_msgs::RobotTrajectory trajectory_msg;
    
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

    for (int i = 0; i < number1; i++)
    {
        // translation
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;

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
    Eigen::Matrix4d transformation2 = parser.Foward(qPre);
    cout << "transformation2: " << "\n" << transformation2 << endl;

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


void poseInit()
{
    geometry_msgs::Pose temp;
    temp.orientation =     
        tf::createQuaternionMsgFromRollPitchYaw(1.5, -0.01, -0.1); // grasp orientation
    
    grasp_pose.orientation = temp.orientation;
    suck_pose.orientation = temp.orientation;
    move_pose.orientation = temp.orientation;
    postmove_pose.orientation = temp.orientation;
    test_pose.orientation = temp.orientation;

    test_pose.position.x = 0.2;
    test_pose.position.y = -0.25;
    test_pose.position.z = 0.3;
    
    postmove_pose.position.x = 0.2;
    postmove_pose.position.y = -0.25;
    postmove_pose.position.z = 0.55; // desk 0.35

    // rest pose
    gripper_rest_pose.orientation = grasp_pose.orientation;
    gripper_rest_pose.position.x = -0.2;
    gripper_rest_pose.position.y = -0.3;
    gripper_rest_pose.position.z = 0.4;

    // gripper place; // pre-defined
    gripper_place_pose.orientation.x = 0.248;
    gripper_place_pose.orientation.y = -0.663;
    gripper_place_pose.orientation.z = -0.145;
    gripper_place_pose.orientation.w = 0.690;
    gripper_place_pose.position.x = -0.43;
    gripper_place_pose.position.y = 0.08;
    gripper_place_pose.position.z = 0.3;

    // suck_pose.orientation
    //     = tf::createQuaternionMsgFromRollPitchYaw(1.57, -2.5, 0.0); // suck orientation
    suck_pose.orientation.x = 0.190;
    suck_pose.orientation.y = -0.639;
    suck_pose.orientation.z = 0.705;
    suck_pose.orientation.w = 0.240;

    sucker_rest_pose.orientation = suck_pose.orientation;
    sucker_rest_pose.position.x = -0.2;
    sucker_rest_pose.position.y = -0.3;
    sucker_rest_pose.position.z = 0.4;

    sucker_place_pose.orientation.x = -0.296;
    sucker_place_pose.orientation.y = -0.597;
    sucker_place_pose.orientation.z = 0.352;
    sucker_place_pose.orientation.w = 0.656;
    sucker_place_pose.position.x = -0.43;
    sucker_place_pose.position.y = 0.08;
    sucker_place_pose.position.z = 0.30;
}