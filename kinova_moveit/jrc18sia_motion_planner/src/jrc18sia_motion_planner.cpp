#include "jrc18sia_motion_planner/jrc18sia_motion_planner.h"

JRCMotionPlanner::JRCMotionPlanner(ros::NodeHandle& nh)
    : nh_(nh),
    group_name_("arm"),
    planning_id_("RRTConnectkConfigDefault"),
    planning_attempts_(1),
    position_tolerance_(0.01), // 1cm
    orientation_tolerance_(0.02), // 0.57 * 2 deg
    planning_time_(0.1), // 0.1s
    max_vel_scale_factor_(0.5), // move_group
    jump_threshold_(2.0), 
    trajectory_velocity_scaling_(TRAJECTORY_VELOCITY_SCALING), // trajectory processing
    max_plan_steps_(MAX_PLAN_STEP),
    max_cartesion_plan_steps_(MAX_CART_PLAN_STEP),
    // Get current joint state from the topic
    kinova_driver_joint_state_topic_("j2n6s300_driver/out/joint_state"),
    // Get current end effector pose from the topic
    kinova_driver_tool_pose_topic_("j2n6s300_driver/out/tool_pose"),
    joint_states_topic_("joint_states")
{
    nh_.getParam("debug_print", debug_print_);
    nh_.getParam("confirm_act", confirm_act_);
    init();
}

void JRCMotionPlanner::init()
{
    // MoveIt
    group_ = new moveit::planning_interface::MoveGroup(group_name_);
    // MoveIt configuration
    group_->setGoalPositionTolerance(position_tolerance_);
    group_->setGoalOrientationTolerance(orientation_tolerance_);
    group_->setPlannerId(planning_id_);
    group_->setPlanningTime(planning_time_);
    group_->setMaxVelocityScalingFactor(max_vel_scale_factor_);
    group_->setNumPlanningAttempts(planning_attempts_);
}


JRCMotionPlanner::~JRCMotionPlanner()
{
    delete group_;
}

std::vector<double> JRCMotionPlanner::getCurrentJointState()
{
    if (debug_print_) {
        std::cout << "Enter JRCMotionPlanner::getCurrentJointValues" << std::endl;
    }

    sensor_msgs::JointStateConstPtr joint_state
        = ros::topic::waitForMessage<sensor_msgs::JointState>(
            joint_states_topic_, nh_, ros::Duration(1.0));
    if (!joint_state) throw std::runtime_error("Joint state message capture failed");

    if (debug_print_) {
        std::size_t num = joint_state->position.size();
        std::vector<double> joint;
        for (int i = 0; i < num; i++) {
            joint.push_back(joint_state->position[i]);
            std::cout << joint[i] << std::endl;
        }
        std::cout << "Leave JRCMotionPlanner::getCurrentJointValues" << std::endl;
    }
    std::vector<double> joint_values;
    for(int i=0;i<6;i++)
    {
        joint_values.push_back(joint_state->position[i]);
    }
    return joint_values;
}

geometry_msgs::Pose JRCMotionPlanner::getCurrentPoseFromMoveit()
{
    return group_->getCurrentPose().pose;
}

geometry_msgs::Pose JRCMotionPlanner::getCurrentPoseFromDriver()
{
    if (debug_print_) {
        std::cout << "Enter JRCMotionPlanner::getCurrentPoseFromDriver" << std::endl;
    }

    geometry_msgs::PoseStampedConstPtr tool_pose
        = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
            kinova_driver_tool_pose_topic_, nh_, ros::Duration(1.0));
    if (!tool_pose) throw std::runtime_error("Current tool pose message capture failed");

    if (debug_print_) {
        std::cout << *tool_pose << std::endl;
        std::cout << "Leave JRCMotionPlanner::getCurrentPoseFromDriver" << std::endl;
    }

    return tool_pose->pose;
}

std::vector<double> JRCMotionPlanner::getCurrentJointStateFromMoveit()
{
    return group_->getCurrentJointValues();
}

std::vector<double> JRCMotionPlanner::getCurrentRPY()
{
    return group_->getCurrentRPY();
}


bool JRCMotionPlanner::executePlan(const moveit::planning_interface::MoveGroup::Plan& plan)
{
    ros::Time start_time = ros::Time::now();
    group_->execute(plan);
    std::cout << "Motion execute duration: " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

    std::cout << "\nMOVE TO TARGET SUCCESSFULLY\n" << std::endl;
}

double JRCMotionPlanner::cartesionPathPlanner(double distance_x, double distance_y,
    double distance_z, double roll, double pitch, double yaw, int number_point, int number_distance)
{
    std::cout << "cartesion path planner..." << std::endl;

    Eigen::VectorXd qPre(6); // NOT Eigen::VectorXd qPre!!! Must be qPre(6)

    std::vector<double> joint_recv = getCurrentJointState();

    qPre << joint_recv[0], joint_recv[1], joint_recv[2], joint_recv[3], joint_recv[4],
        joint_recv[5];
    if(debug_print_)
    {
        std::cout << "qPre: "
        << "\n"
        << qPre << std::endl;
    }


    double roll_rad = 0;
    double pitch_rad = 0;
    double yaw_rad = 0;

    roll_rad = (roll / 180.0 * Pi);   // X
    pitch_rad = (pitch / 180.0 * Pi); // Y
    yaw_rad = (yaw / 180.0 * Pi);     // Z

    if(debug_print_)
    {
        std::cout << "Received RPY (XYZ) angle (deg): "
                << "\n"
                << roll << " " << pitch << " " << yaw << std::endl;
    }


    tf::Quaternion end_quat_tf;
    /**@brief Set the quaternion using fixed axis RPY
    * @param roll Angle around X
    * @param pitch Angle around Y
    * @param yaw Angle around Z*/
    end_quat_tf.setRPY(roll_rad, pitch_rad, yaw_rad);

    Eigen::Matrix3d rotation_matrix_eigen;
    tf::Matrix3x3 rotation_matrix_tf;
    tf::Quaternion q_tf;

    // Current transformation, including T & R
    Eigen::Matrix4d transformation = parser_.Foward(qPre);
    // if(debug_print_)
    {
        std::cout << "current joint values: " << qPre << std::endl;
            std::cout << "current transformation1: "
              << "\n"
              << transformation << std::endl;
    }


    rotation_matrix_eigen << transformation(0, 0), transformation(0, 1), transformation(0, 2),
        transformation(1, 0), transformation(1, 1), transformation(1, 2), transformation(2, 0),
        transformation(2, 1), transformation(2, 2);

    Eigen::Quaterniond eigen_quat(rotation_matrix_eigen);
    if(debug_print_)
    {
            std::cout << "eigen Quaterniond1:"
              << "\n"
              << eigen_quat.x() << " " << eigen_quat.y() << " " << eigen_quat.z() << " "
              << eigen_quat.w() << std::endl;
    }


    // Eigen => tf
    tf::Quaternion start_quat_tf(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    if(debug_print_)
    {
            std::cout << "tf::Quaternion:"
              << "\n"
              << start_quat_tf.x() << " " << start_quat_tf.y() << " " << start_quat_tf.z() << " "
              << start_quat_tf.w() << std::endl;
    }

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double R, P, Y;
    tf::Matrix3x3 tf_rotation_matrix(start_quat_tf);
    tf_rotation_matrix.getRPY(R, P, Y);
    if(debug_print_)
    {
            std::cout << "tf RPY angle: "
              << "\n"
              << R * 180 / Pi << " " << P * 180 / Pi << " " << Y * 180 / Pi << std::endl;
    }
    control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;
    moveit_msgs::RobotTrajectory moveit_robot_traj_msg;
    follow_joint_traj_goal.trajectory.header.frame_id = "j2n6s300_base";
    follow_joint_traj_goal.trajectory.header.stamp = ros::Time::now();
    follow_joint_traj_goal.trajectory.joint_names.clear();
    moveit_robot_traj_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    moveit_robot_traj_msg.joint_trajectory.header.stamp = ros::Time::now();
    moveit_robot_traj_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++) {
        std::stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        follow_joint_traj_goal.trajectory.joint_names.push_back(jointName.str());
        moveit_robot_traj_msg.joint_trajectory.joint_names.push_back(jointName.str());
    }

    follow_joint_traj_goal.trajectory.points.clear();
    moveit_robot_traj_msg.joint_trajectory.points.clear();

    tf::Quaternion temp_quat_tf;

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(1.0);
    for (std::size_t i = 0; i < number_point; i++) {
        // translation
        transformation(0, 3) += distance_x / number_point;
        transformation(1, 3) += distance_y / number_point;
        transformation(2, 3) += distance_z / number_point;
        // rotation
        temp_quat_tf = start_quat_tf.slerp(end_quat_tf, (1.0 / (float)number_point) * (i + 1));
        tf::Matrix3x3 temp_rotation_matrix_tf(temp_quat_tf);
        temp_rotation_matrix_tf.getRPY(R, P, Y);
        if (debug_print_) {
            std::cout << "Current tf RPY (XYZ) angle 2: "
                      << "\n"
                      << R * 180 / Pi << " " << P * 180 / Pi << " " << Y * 180 / Pi << std::endl;
        }

        Eigen::Quaterniond temp_eigen_target_q(
            temp_quat_tf.w(), temp_quat_tf.x(), temp_quat_tf.y(), temp_quat_tf.z());
        rotation_matrix_eigen = temp_eigen_target_q.toRotationMatrix();

        for (int m = 0; m < 3; m++) {
            for (int n = 0; n < 3; n++) {
                // new T
                transformation(m, n) = rotation_matrix_eigen(m, n);
            }
        }
        // IK from transformation()
        Eigen::VectorXd q(6);
        q = parser_.Inverse(transformation, qPre);
        // std::cout << "=========" << std::endl;
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_ERROR("Planning Timeout!");
            break;
        }
        if (q(0) > 10) continue;

        // printf("loops:%d",i);
        for (std::size_t j = 0; j < number_distance; j++) {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++) {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number_distance * j);
                point.velocities.push_back(0.0);
                point.accelerations.push_back(0.0);
            }
            point.time_from_start = ros::Duration();
            follow_joint_traj_goal.trajectory.points.push_back(point);
            moveit_robot_traj_msg.joint_trajectory.points.push_back(point);
        }
        // printf("\n\njoint values : %d\n",i);
        // std::cout << q  << "\n" << std::endl;
        qPre = q;
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_ERROR("Planning Timeout!!");
            break;
        }
        if (debug_print_) {
            std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s"
                      << std::endl;
        }
    }
    // if(debug_print_)
    {       std::cout << "target joint values: " << qPre << std::endl;
            std::cout << "target transformation: "
              << "\n"
              << transformation << std::endl;
    }
    std::cout << "Total planning time is : " << (ros::Time::now() - start_time).toSec() << "s"
              << std::endl;

    double result = (double)moveit_robot_traj_msg.joint_trajectory.points.size() / (double)(number_point * number_distance);
    // std::cout << "result = " << result << ", number_point = " << number_point << ", numbe_distance = " << number_distance << std::endl;
    // std::cout << (double)moveit_robot_traj_msg.joint_trajectory.points.size() << " " << (double)(number_point * number_distance) << std::endl;
    std::cout << "trajectory points number: "
              << moveit_robot_traj_msg.joint_trajectory.points.size() 
              << ", " << result * 100 << "%" << std::endl;
    if(result == 0.0)
    {
        ROS_ERROR("compute cartesion path is ERROR!");
    }
    else if(result == 1.0)
    {
        std::cout << "compute cartesion path successfully! : 100 %" << std::endl;
    }
    else if(result > 0.0 && result < 1.0)
    {
        ROS_ERROR_STREAM("compute cartesion path : " << result * 100 << "%");
    }
    //    addTimeToTraj(&moveit_robot_traj_msg,TRAJECTORY_VELOCITY_SCALING);

    robot_trajectory::RobotTrajectory rt(
        group_->getCurrentState()->getRobotModel(), group_->getName());
    rt.setRobotTrajectoryMsg(*group_->getCurrentState(), moveit_robot_traj_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
    if(!IptpSuccess)
    {
        ROS_ERROR("Computed time stamped FAILED");
    }
    rt.getRobotTrajectoryMsg(moveit_robot_traj_msg);

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = moveit_robot_traj_msg;
    confirmToAct();
    executePlan(plan);
    // executeTrajectory(moveit_robot_traj_msg.joint_trajectory);
}

double JRCMotionPlanner::cartesionPathPlanner(
    double distance_x, double distance_y, double distance_z, int number_point, int number_distance)
{
    std::cout << "cartesion path planner..." << std::endl;

    Eigen::VectorXd qPre(6); // NOT Eigen::VectorXd qPre!!! Must be qPre(6)

    std::vector<double> joint_recv = getCurrentJointState();

    qPre << joint_recv[0], joint_recv[1], joint_recv[2], joint_recv[3], joint_recv[4],
        joint_recv[5];
    if(debug_print_)
    {
            std::cout << "qPre: "
              << "\n"
              << qPre << std::endl;
    }


    // Current transformation, including T & R
    Eigen::Matrix4d transformation = parser_.Foward(qPre);
    // if(debug_print_)
    {   
        std::cout << "current joint values: " << qPre << std::endl;
            std::cout << "current transformation1: "
              << "\n"
              << transformation << std::endl;
    }


    // FollowJointTrajectoryActionGoal
    control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;
    moveit_msgs::RobotTrajectory moveit_robot_traj_msg;

    follow_joint_traj_goal.trajectory.header.frame_id = "j2n6s300_base";
    follow_joint_traj_goal.trajectory.header.stamp = ros::Time::now();
    follow_joint_traj_goal.trajectory.joint_names.clear();
    moveit_robot_traj_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    moveit_robot_traj_msg.joint_trajectory.header.stamp = ros::Time::now();
    moveit_robot_traj_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++) {
        std::stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        follow_joint_traj_goal.trajectory.joint_names.push_back(jointName.str());
        moveit_robot_traj_msg.joint_trajectory.joint_names.push_back(jointName.str());
    }

    follow_joint_traj_goal.trajectory.points.clear();
    moveit_robot_traj_msg.joint_trajectory.points.clear();

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(1.0);
    for (std::size_t i = 0; i < number_point; i++) {
        // translation
        transformation(0, 3) += distance_x / number_point;
        transformation(1, 3) += distance_y / number_point;
        transformation(2, 3) += distance_z / number_point;

        // IK from transformation()
        Eigen::VectorXd q(6);
        q = parser_.Inverse(transformation, qPre);
        //        std::cout << "=========" << std::endl;
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_ERROR("Planning Timeout!");
            break;
        }
        if (q(0) > 10) continue;

        // printf("loops:%d",i);
        for (std::size_t j = 0; j < number_distance; j++) {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++) {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number_distance * j);
                point.velocities.push_back(0.0);
                point.accelerations.push_back(0.0);
            }
            point.time_from_start = ros::Duration();
            follow_joint_traj_goal.trajectory.points.push_back(point);
            moveit_robot_traj_msg.joint_trajectory.points.push_back(point);
        }
        // printf("\n\njoint values : %d\n",i);
        // std::cout << q  << "\n" << std::endl;
        qPre = q;
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_ERROR("Planning Timeout!");
            break;
        }
        if (debug_print_) {
            std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s"
                      << std::endl;
        }
    }
    // if(debug_print_)
    {
        std::cout << "target joint values: " << qPre << std::endl;
            std::cout << "target transformation: "
              << "\n"
              << transformation << std::endl;
    }
    std::cout << "Total planning time is : " << (ros::Time::now() - start_time).toSec() << "s"
              << std::endl;

    double result = (double)moveit_robot_traj_msg.joint_trajectory.points.size() / (double)(number_point * number_distance);
    std::cout << "trajectory points number: "
              << moveit_robot_traj_msg.joint_trajectory.points.size() 
              << ", " << result * 100 << "%"
              << std::endl;
    if(result == 0.0)
    {
        ROS_ERROR("compute cartesion path is ERROR!");
    }
    else if(result == 1.0)
    {
        std::cout << "compute cartesion path successfully! : 100 %" << std::endl;
    }
    else if(result > 0.0 && result < 1.0)
    {
        ROS_ERROR_STREAM("compute cartesion path : " << result * 100 << "%");
    }
    //    addTimeToTraj(&moveit_robot_traj_msg, TRAJECTORY_VELOCITY_SCALING);
    robot_trajectory::RobotTrajectory rt(
        group_->getCurrentState()->getRobotModel(), group_->getName());
    rt.setRobotTrajectoryMsg(*group_->getCurrentState(), moveit_robot_traj_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
    if(!IptpSuccess)
    {
        ROS_ERROR("Computed time stamped FAILED");
    }
    rt.getRobotTrajectoryMsg(moveit_robot_traj_msg);

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = moveit_robot_traj_msg;
    confirmToAct();
    executePlan(plan);
    // executeTrajectory(moveit_robot_traj_msg.joint_trajectory);
}

double JRCMotionPlanner::cartesionPathPlanner(
    double distance_x, double distance_y, double distance_z)
{
    int x = distance_x * 1000;
    int y = distance_y * 1000;
    int z = distance_z * 1000;
    // std::cout << x << " " << y << " " << z << std::endl;
    int max = abs(x);
    if(max < abs(y)) 
    {
        max = abs(y);
        // std::cout << max << std::endl;
    }
    if(max < abs(z)) 
    {
        max = abs(z);
        // std::cout << max << std::endl;
    }
    if(max == 0) max=100;

    std::cout << "num_step : " << max << std::endl;

    return cartesionPathPlanner(distance_x, distance_y, distance_z, max, 1);
}

double JRCMotionPlanner::cartesionPathPlanner(double distance_x, double distance_y, double distance_z,
                                double roll, double pitch, double yaw)
{
    int x = distance_x * 1000;
    int y = distance_y * 1000;
    int z = distance_z * 1000;
    // std::cout << x << " " << y << " " << z << std::endl;
    int max = abs(x);
    if(max < abs(y)) 
    {
        max = abs(y);
        // std::cout << max << std::endl;
    }
    if(max < abs(z)) 
    {
        max = abs(z);
        // std::cout << max << std::endl;
    }
    if(max == 0) max=100;

    std::cout << "num_step : " << max << std::endl;

    return cartesionPathPlanner(distance_x, distance_y, distance_z, roll, pitch, yaw, max, 1);
}

void JRCMotionPlanner::confirmToAct()
{
    std::cout << "Confirm start and end info and press n to start plan" << std::endl;
    if (confirm_act_) 
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_ || "N" == pause_) {
            std::cout << "Valid info, begin to move" << std::endl;
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const std::string& str)
{
    std::cout << "\n"
              << "=================MOVE TO " + str + "=================="
              << "\n";
    ROS_INFO_STREAM("Move from: " << start << "to " << goal);
    std::cout << "Confirm start ---> goal info, press n to start plan" << std::endl;
    if (confirm_act_) {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            std::cout << "Corrent state, begin to plan" << std::endl;
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
    ROS_INFO_STREAM("Move from: " << start << "to " << goal);
    std::cout << "Confirm start ---> goal info, press n to start plan" << std::endl;
    if (confirm_act_) {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            std::cout << "Corrent state, begin to plan" << std::endl;
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(const geometry_msgs::Pose& goal, const std::string& str)
{
    std::cout << "\n"
              << "=================MOVE TO " + str + "=================="
              << "\n";
    ROS_INFO_STREAM("Move to target" << goal);
    std::cout << "Confirm start ---> goal info, press n to start plan" << std::endl;
    if (confirm_act_) {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            std::cout << "Correct state, begin to plan" << std::endl;
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(const geometry_msgs::Pose& goal)
{
    ROS_INFO_STREAM("Move to target" << goal);
    std::cout << "Confirm start ---> goal info, press n to start plan" << std::endl;
    if (confirm_act_) {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            std::cout << "Correct state, begin to plan" << std::endl;
        } else {
            return;
        }
    }
}

// Plan with pre-defined postures
void JRCMotionPlanner::moveToTargetNamed(const std::string& target_name)
{
    group_->setStartStateToCurrentState();
    group_->setNamedTarget(target_name);

    moveit::planning_interface::MoveGroup::Plan temp_plan, best_plan;
    //    findBestTimePlan(temp_plan,best_plan);
    int loops = 100;

    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);
    ros::Time start_time = ros::Time::now();
    for (int i = 0; i < loops; i++) {
        bool suc = group_->plan(temp_plan);
        if (suc) {
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time) {
                best_plan = temp_plan;
                best_time = current_time;
                std::cout << current_time << std::endl;
            }
        }
    }
    std::cout << 
        "Motion Planning 100 times duration: " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

    bool plan_valid = false;
    int plan_steps = 0;

    plan_steps = getPlanPointNum(best_plan);
    if (plan_steps < max_plan_steps_) {
        std::cout << "Plan found in " << best_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps" << std::endl;
        plan_valid = true;
    }

    if (!plan_valid) {
        ROS_ERROR_STREAM("plan found in " << plan_steps << " steps");
        exit(0); // TODO
    }

    // Execute the plan
    confirmToAct();
    executePlan(best_plan);
}

// Find the best time trjaecotry
void JRCMotionPlanner::moveToTargetBestTime(const geometry_msgs::Pose& target)
{
    group_->setStartStateToCurrentState();
    group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroup::Plan temp_plan, best_plan;
    //    findBestTimePlan(temp_plan,best_plan);
    int loops = 100;
    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(5.0);
    for (int i = 0; i < loops; i++) {
        bool suc = group_->plan(temp_plan);
        if (suc) {
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time) {
                best_plan = temp_plan;
                best_time = current_time;
                if (debug_print_) {
                    std::cout << current_time << std::endl;
                }
            }
        }
        if ((ros::Time::now() - start_time) > timeout) {
            break;
            ROS_ERROR("No solution in 5s!");
        }
    }
    std::cout << 
        "Motion Planning 100 times duration: " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

    bool plan_valid = false;
    int plan_steps = 0;

    plan_steps = getPlanPointNum(best_plan);
    if (plan_steps < max_plan_steps_) {
        std::cout << "Plan found in " << best_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps" << std::endl;
        plan_valid = true;
    }

    if (!plan_valid) {
        ROS_ERROR_STREAM("Plan found in " << plan_steps << " steps");
        exit(0); // TODO
    }

    // Execute the plan
    confirmToAct(target);
    executePlan(best_plan);
}

void JRCMotionPlanner::moveToTargetBestTime(const geometry_msgs::PoseStamped& target)
{
    moveToTargetBestTime(target.pose);
}

// move line by MoveIt computeCartesianPath functions
void JRCMotionPlanner::moveLineTarget(const geometry_msgs::Pose& goal)
{

    std::cout << "Begin cartesian line plan by MoveIt computeCartesianPath ..." << std::endl;

    geometry_msgs::Pose start_pose = getCurrentPoseFromMoveit();
    geometry_msgs::Pose way_pose = start_pose;
    ROS_INFO_STREAM("start_pose" << start_pose);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(way_pose); // first pose waypoint
    int num_waypoint = 20;
    float delta_x = (goal.position.x - start_pose.position.x) / (num_waypoint - 1);
    float delta_y = (goal.position.y - start_pose.position.y) / (num_waypoint - 1);
    float delta_z = (goal.position.z - start_pose.position.z) / (num_waypoint - 1);

    // interplotate between current pose and target pose
    for (int i = 0; i < num_waypoint - 1; i++) {
        way_pose.position.x += delta_x;
        way_pose.position.y += delta_y;
        way_pose.position.z += delta_z;
        waypoints.push_back(way_pose);
        // ROS_INFO_STREAM(way_pose);
    }

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = jump_threshold_; // TODO
    const double eef_step = 0.01;
    ros::Time start_time = ros::Time::now();
    double fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction == 1.0) {
        std::cout << "computeCartesionPath Successfully" << std::endl;
    } else if (fraction == -1.0) {
        ROS_ERROR("computeCartesionPath ERROR!");
    } else {
        ROS_ERROR_STREAM("computeCartesionPath : " << fraction * 100 << " %");
    }

    std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s"
              << std::endl;

    moveit::planning_interface::MoveGroup::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    int plan_steps = getPlanPointNum(cartesian_plan);
    std::cout << "Line plan steps: " << plan_steps << std::endl;
    if (plan_steps < max_cartesion_plan_steps_ && plan_steps > 5) {
        std::cout << "Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps" << std::endl;
        confirmToAct(start_pose, goal);
        executePlan(cartesian_plan);
    } else {
        ROS_ERROR_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                          << plan_steps << " steps");
        exit(0); // TODO
    }
}

std::size_t JRCMotionPlanner::getPlanPointNum(
    const moveit::planning_interface::MoveGroup::Plan& plan)
{
    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
    return trajectory.joint_trajectory.points.size();
}

void JRCMotionPlanner::moveLineTarget(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
    std::cout << "Begin cartesian line plan by MoveIt computeCartesianPath ..." << std::endl;

    geometry_msgs::Pose start_pose = start;
    geometry_msgs::Pose way_pose = start_pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(way_pose); // first pose waypoint
    int num_waypoint = 20;
    float delta_x = (goal.position.x - start_pose.position.x) / (num_waypoint - 1);
    float delta_y = (goal.position.y - start_pose.position.y) / (num_waypoint - 1);
    float delta_z = (goal.position.z - start_pose.position.z) / (num_waypoint - 1);

    // interplotate between current pose and target pose
    for (int i = 0; i < num_waypoint - 1; i++) {
        way_pose.position.x += delta_x;
        way_pose.position.y += delta_y;
        way_pose.position.z += delta_z;
        waypoints.push_back(way_pose);
    }

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = jump_threshold_; // TODO
    const double eef_step = 0.01;
    ros::Time start_time = ros::Time::now();
    double fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction == 1.0) {
        std::cout << "computeCartesionPath Successfully" << std::endl;
    } else if (fraction == -1.0) {
        ROS_ERROR("computeCartesionPath ERROR!");
    } else {
        ROS_ERROR_STREAM("computeCartesionPath : " << fraction*100 << " %");
    }

    std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s"
              << std::endl;

    moveit::planning_interface::MoveGroup::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    int plan_steps = getPlanPointNum(cartesian_plan);
    std::cout << "Line plan steps: " << plan_steps << std::endl;
    if (plan_steps < max_cartesion_plan_steps_ && plan_steps > 5) {
        std::cout << "Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps" << std::endl;
        confirmToAct(start_pose,goal);
        executePlan(cartesian_plan);
    } else {
        ROS_ERROR_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps" );
        exit(0); // TODO
    }
}

void JRCMotionPlanner::setJointValueTarget(const std::vector<double> joint_values)
{
    group_->setJointValueTarget(joint_values);
    confirmToAct();
    group_->move();
}

void JRCMotionPlanner::setJointValueTarget(const int joint_index, const double joint_values)
{
    std::vector<double> current_joint_values = getCurrentJointStateFromMoveit();
    current_joint_values[joint_index] += joint_values;
    setJointValueTarget(current_joint_values);
}