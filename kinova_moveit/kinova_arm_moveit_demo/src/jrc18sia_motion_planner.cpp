#include "jrc18sia_motion_planner.h"

JRCMotionPlanner::JRCMotionPlanner(ros::NodeHandle &nh)
    : nh_(nh),
      group_name_("arm"),
      // Get current joint state from the topic
      kinova_driver_joint_state_topic_("j2n6s300_driver/out/joint_state"),
      // Get current end effector pose from the topic
      kinova_driver_tool_pose_topic_("j2n6s300_driver/out/tool_pose"),
      joint_states_topic_("joint_states")
{
    getParameters();
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

bool JRCMotionPlanner::getParameters()
{

    /***Get parameters from jrc18sia_motion_planner_parameters.yaml***/

    if (!nh_.hasParam("position_tolerance"))
    {
      ROS_ERROR("Configuration parameter `position_tolerance` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("jrc8sia_motion_planner/position_tolerance", position_tolerance_);
    // There is something wrong with this param
    position_tolerance_ = 0.01;
    if(debug_print_)
    {
        std::cout << "position_tolerance_: " << position_tolerance_ << std::endl;
    }

    if (!nh_.hasParam("orientation_tolerance"))
    {
      ROS_ERROR("Configuration parameter `orientation_tolerance` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("orientation_tolerance", orientation_tolerance_);

    std::cout << "orientation_tolerance_: " << orientation_tolerance_ << std::endl;

    if (!nh_.hasParam("planning_time"))
    {
      ROS_ERROR("Configuration parameter `planning_time` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("planning_time", planning_time_);
    if(debug_print_)
    {
        std::cout << "planning_time_: " << planning_time_ << std::endl;
    }

    if (!nh_.hasParam("max_vel_scale_factor"))
    {
      ROS_ERROR("Configuration parameter `max_vel_scale_factor` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("max_vel_scale_factor", max_vel_scale_factor_);
    if(debug_print_)
    {
        std::cout << "max_vel_scale_factor_: " << max_vel_scale_factor_ << std::endl;
    }

    if (!nh_.hasParam("planning_attempts"))
    {
      ROS_ERROR("Configuration parameter `planning_attempts` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("planning_attempts", planning_attempts_);
    if(debug_print_)
    {
        std::cout << "planning_attempts_: " << planning_attempts_ << std::endl;
    }

    if (!nh_.hasParam("planning_id"))
    {
      ROS_ERROR("Configuration parameter `planning_id` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("planning_id", planning_id_);
    if(debug_print_)
    {
        std::cout << "planning_id_: " << planning_id_ << std::endl;
    }

    if (!nh_.hasParam("jump_threshold"))
    {
      ROS_ERROR("Configuration parameter `jump_threshold` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("jump_threshold", jump_threshold_);
    if(debug_print_)
    {
        std::cout << "jump_threshold_: " << jump_threshold_ << std::endl;
    }

    if (!nh_.hasParam("trajectory_velocity_scaling"))
    {
      ROS_ERROR("Configuration parameter `trajectory_velocity_scaling` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("trajectory_velocity_scaling", trajectory_velocity_scaling_);
    if(debug_print_)
    {
        std::cout << "trajectory_velocity_scaling_: " << trajectory_velocity_scaling_ << std::endl;
    }

    if (!nh_.hasParam("max_plan_steps"))
    {
      ROS_ERROR("Configuration parameter `max_plan_steps` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("max_plan_steps", max_plan_steps_);
    if(debug_print_)
    {
        std::cout << "max_plan_steps_: " << max_plan_steps_ << std::endl;
    }

    if (!nh_.hasParam("max_cartesion_plan_steps"))
    {
      ROS_ERROR("Configuration parameter `max_cartesion_plan_steps` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("max_cartesion_plan_steps", max_cartesion_plan_steps_);
    if(debug_print_)
    {
        std::cout << "max_cartesion_plan_steps_: " << max_cartesion_plan_steps_ << std::endl;
    }

    if (!nh_.hasParam("debug_print"))
    {
      ROS_ERROR("Configuration parameter `debug_print` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("debug_print", debug_print_);
    if(debug_print_)
    {
        std::cout << "debug_print_: " << debug_print_ << std::endl;
    }

    if (!nh_.hasParam("confirm_act"))
    {
      ROS_ERROR("Configuration parameter `confirm_act` missing from rosparam server. Did you load your configuration yaml file?");
      return false;
    }
    nh_.getParam("confirm_act", confirm_act_);
    if(debug_print_)
    {
        std::cout << "confirm_act_: " << confirm_act_ << std::endl;
    }

}

JRCMotionPlanner::~JRCMotionPlanner()
{
    delete group_;
}

std::vector<double> JRCMotionPlanner::getCurrentJointState()
{
    if(debug_print_)
    {
        ROS_INFO("Enter JRCMotionPlanner::getCurrentJointValues");
    }

    sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_topic_, nh_, ros::Duration(1.0));
    if(!joint_state) throw std::runtime_error("Joint state message capture failed");


    if(debug_print_)
    {
        std::size_t num = joint_state->position.size();
        std::vector<double> joint;
        for(int i=0;i<num;i++)
        {
            joint.push_back(joint_state->position[i]);
            std::cout << joint[i] << std::endl;
        }
        ROS_INFO("Leave JRCMotionPlanner::getCurrentJointValues");
    }

    return joint_state->position;
}

Eigen::Matrix4d JRCMotionPlanner::getCurrentPoseFromCustomFK()
{
    //    return parser_.Foward(q_current_);
}

geometry_msgs::PoseStamped JRCMotionPlanner::getCurrentPoseFromMoveit()
{
    return group_->getCurrentPose();
}

geometry_msgs::Pose JRCMotionPlanner::getCurrentPoseFromDriver()
{
    if(debug_print_)
    {
        ROS_INFO("Enter JRCMotionPlanner::getCurrentPoseFromDriver");
    }

    geometry_msgs::PoseStampedConstPtr tool_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(kinova_driver_tool_pose_topic_, nh_, ros::Duration(1.0));
    if(!tool_pose) throw std::runtime_error("Current tool pose message capture failed");

    if(debug_print_)
    {
        ROS_INFO_STREAM(*tool_pose);
        ROS_INFO("Leave JRCMotionPlanner::getCurrentPoseFromDriver");
    }

    return tool_pose->pose;
}

std::vector<double> JRCMotionPlanner::getCurrentRPY()
{
    return group_->getCurrentRPY();
}

void JRCMotionPlanner::findBestTimePlan(moveit::planning_interface::MoveGroup::Plan &temp_plan,
                                        moveit::planning_interface::MoveGroup::Plan &best_plan)
{
    int loops = 100;
    bool success = false;

    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);
    ros::Time start_time = ros::Time::now();
    for (int i = 0; i < loops; i++) {
        bool suc = group_->plan(temp_plan);
        if (suc) {
            success = true;
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time) {
                best_plan = temp_plan;
                best_time = current_time;
                ROS_INFO_STREAM(current_time);
            }
        }
    }
    ROS_INFO_STREAM("Motion Planning 100 times duration: " << (ros::Time::now() - start_time).toSec() << "s");

}


void JRCMotionPlanner::addTimeToTraj(moveit_msgs::RobotTrajectory *robot_traj_msg, const double trajectory_velocity_scaling)
{
    // trajectory process, add time to the trajectory
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // trajectory processing
    robot_trajectory::RobotTrajectory robot_traj(group_->getCurrentState()->getRobotModel(), group_->getName());
    robot_traj.setRobotTrajectoryMsg(*group_->getCurrentState(), *robot_traj_msg);
    bool IptpSuccess = false;

    IptpSuccess = iptp.computeTimeStamps(robot_traj,trajectory_velocity_scaling);
    ROS_INFO("Computed time stamped %s", IptpSuccess ? "SUCCED" : "FAILED");
    robot_traj.getRobotTrajectoryMsg(*robot_traj_msg);
}

bool JRCMotionPlanner::executePlan(const moveit::planning_interface::MoveGroup::Plan &plan)
{
    ros::Time start_time = ros::Time::now();
    group_->execute(plan);
    ROS_INFO_STREAM("Motion execute duration: " << (ros::Time::now() - start_time).toSec() << "s");

    ROS_INFO_ONCE("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");
}

double JRCMotionPlanner::cartesionPathPlanner(double distance_x, double distance_y, double distance_z,
                                             double roll,      double pitch,     double yaw,
                                             int number_point, int number_distance)
{
    ROS_INFO("cartesion path planner...");

    Eigen::VectorXd qPre(6); // NOT Eigen::VectorXd qPre!!! Must be qPre(6)

    std::vector<double> joint_recv = getCurrentJointState();

    qPre << joint_recv[0], joint_recv[1], joint_recv[2], joint_recv[3], joint_recv[4], joint_recv[5];
    std::cout << "qPre: " << "\n" << qPre << std::endl;

    double roll_rad = 0;
    double pitch_rad = 0;
    double yaw_rad = 0;

    roll_rad = (roll / 180.0 * Pi);   // X
    pitch_rad = (pitch / 180.0 * Pi); // Y
    yaw_rad = (yaw / 180.0 * Pi);     // Z

    std::cout << "Received RPY (XYZ) angle (deg): " << "\n"
              << roll << " "
              << pitch << " "
              << yaw << std::endl;

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
    std::cout << "transformation1: " << "\n"
              << transformation << std::endl;

    rotation_matrix_eigen << transformation(0,0), transformation(0,1), transformation(0,2),
                             transformation(1,0), transformation(1,1), transformation(1,2),
                             transformation(2,0), transformation(2,1), transformation(2,2);

    Eigen::Quaterniond eigen_quat(rotation_matrix_eigen);
    std::cout << "eigen Quaterniond1:" << "\n"
         << eigen_quat.x() << " "
         << eigen_quat.y() << " "
         << eigen_quat.z() << " "
         << eigen_quat.w() << std::endl;

    // Eigen => tf
    tf::Quaternion start_quat_tf(eigen_quat.x(),
                                 eigen_quat.y(),
                                 eigen_quat.z(),
                                 eigen_quat.w());
    std::cout << "tf::Quaternion:" << "\n"
             << start_quat_tf.x() << " "
             << start_quat_tf.y() << " "
             << start_quat_tf.z() << " "
             << start_quat_tf.w() << std::endl;

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double R, P, Y;
    tf::Matrix3x3 tf_rotation_matrix(start_quat_tf);
    tf_rotation_matrix.getRPY(R, P, Y);
    std::cout << "tf RPY angle: " << "\n"
             << R * 180 / Pi << " "
             << P * 180 / Pi << " "
             << Y * 180 / Pi << std::endl;
    control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;
    moveit_msgs::RobotTrajectory moveit_robot_traj_msg;
    follow_joint_traj_goal.trajectory.header.frame_id = "j2n6s300_base";
    follow_joint_traj_goal.trajectory.header.stamp = ros::Time::now();
    follow_joint_traj_goal.trajectory.joint_names.clear();
    moveit_robot_traj_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    moveit_robot_traj_msg.joint_trajectory.header.stamp = ros::Time::now();
    moveit_robot_traj_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++)
    {
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
    for (std::size_t i = 0; i < number_point; i++)
    {
        // translation
        transformation(0, 3) += distance_x / number_point;
        transformation(1, 3) += distance_y / number_point;
        transformation(2, 3) += distance_z / number_point;
        // rotation
        temp_quat_tf = start_quat_tf.slerp(end_quat_tf,
                                           (1.0 / (float)number_point) * (i+1));
        tf::Matrix3x3 temp_rotation_matrix_tf(temp_quat_tf);
        temp_rotation_matrix_tf.getRPY(R, P, Y);
        if(debug_print_)
        {
            std::cout << "Current tf RPY (XYZ) angle 2: " << "\n"
                      << R * 180 / Pi << " "
                      << P * 180 / Pi << " "
                      << Y * 180 / Pi << std::endl;
        }


        Eigen::Quaterniond temp_eigen_target_q(temp_quat_tf.w(),
                                              temp_quat_tf.x(),
                                              temp_quat_tf.y(),
                                              temp_quat_tf.z());
        rotation_matrix_eigen = temp_eigen_target_q.toRotationMatrix();

        for(int m=0; m<3; m++)
        {
            for(int n=0; n<3; n++)
            {
                // new T
                transformation(m,n) = rotation_matrix_eigen(m,n);
            }
        }
        // IK from transformation()
        Eigen::VectorXd q(6);
        q = parser_.Inverse(transformation, qPre);
        // std::cout << "=========" << std::endl;
        if((ros::Time::now() - start_time)>timeout)
        {
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
        if((ros::Time::now() - start_time)>timeout)
        {
            ROS_ERROR("Planning Timeout!!");
            break;
        }
        if(debug_print_)
        {
            std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
        }

    }
    std::cout << "Total planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
    std::cout << "trajectory points number: " << moveit_robot_traj_msg.joint_trajectory.points.size() << std::endl;

//    addTimeToTraj(&moveit_robot_traj_msg,TRAJECTORY_VELOCITY_SCALING);

    robot_trajectory::RobotTrajectory rt(group_->getCurrentState()->getRobotModel(), group_->getName());
    rt.setRobotTrajectoryMsg(*group_->getCurrentState(), moveit_robot_traj_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt,trajectory_velocity_scaling_);
    ROS_INFO("Computed time stamped %s", IptpSuccess ? "SUCCED" : "FAILED");
    rt.getRobotTrajectoryMsg(moveit_robot_traj_msg);

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = moveit_robot_traj_msg;
    confirmToAct();
    executePlan(plan);
}

double JRCMotionPlanner::cartesionPathPlanner(double distance_x, double distance_y, double distance_z,
                                             int number_point, int number_distance)
{
    ROS_INFO("cartesion path planner...");

    Eigen::VectorXd qPre(6); // NOT Eigen::VectorXd qPre!!! Must be qPre(6)

    std::vector<double> joint_recv = getCurrentJointState();

    qPre << joint_recv[0], joint_recv[1], joint_recv[2], joint_recv[3], joint_recv[4], joint_recv[5];
    std::cout << "qPre: " << "\n" << qPre << std::endl;

    // Current transformation, including T & R
    Eigen::Matrix4d transformation = parser_.Foward(qPre);
    std::cout << "transformation1: " << "\n"
              << transformation << std::endl;

    // FollowJointTrajectoryActionGoal
    control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;
    moveit_msgs::RobotTrajectory moveit_robot_traj_msg;

    follow_joint_traj_goal.trajectory.header.frame_id = "j2n6s300_base";
    follow_joint_traj_goal.trajectory.header.stamp = ros::Time::now();
    follow_joint_traj_goal.trajectory.joint_names.clear();
    moveit_robot_traj_msg.joint_trajectory.header.frame_id = "j2n6s300_base";
    moveit_robot_traj_msg.joint_trajectory.header.stamp = ros::Time::now();
    moveit_robot_traj_msg.joint_trajectory.joint_names.clear();

    for (int k = 0; k < 6; k++)
    {
        std::stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        follow_joint_traj_goal.trajectory.joint_names.push_back(jointName.str());
        moveit_robot_traj_msg.joint_trajectory.joint_names.push_back(jointName.str());
    }

    follow_joint_traj_goal.trajectory.points.clear();
    moveit_robot_traj_msg.joint_trajectory.points.clear();

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(1.0);
    for (std::size_t i = 0; i < number_point; i++)
    {
        // translation
        transformation(0, 3) += distance_x / number_point;
        transformation(1, 3) += distance_y / number_point;
        transformation(2, 3) += distance_z / number_point;

        // IK from transformation()
        Eigen::VectorXd q(6);
        q = parser_.Inverse(transformation, qPre);
//        std::cout << "=========" << std::endl;
        if((ros::Time::now() - start_time)>timeout)
        {
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
        if((ros::Time::now() - start_time)>timeout)
        {
            ROS_ERROR("Planning Timeout!");
            break;
        }
        if(debug_print_)
        {
            std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
        }

    }
    std::cout << "Total planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
    std::cout << "trajectory points number: " << moveit_robot_traj_msg.joint_trajectory.points.size() << std::endl;

//    addTimeToTraj(&moveit_robot_traj_msg, TRAJECTORY_VELOCITY_SCALING);
    robot_trajectory::RobotTrajectory rt(group_->getCurrentState()->getRobotModel(), group_->getName());
    rt.setRobotTrajectoryMsg(*group_->getCurrentState(), moveit_robot_traj_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool IptpSuccess = false;
    IptpSuccess = iptp.computeTimeStamps(rt,trajectory_velocity_scaling_);
    ROS_INFO("Computed time stamped %s", IptpSuccess ? "SUCCED" : "FAILED");
    rt.getRobotTrajectoryMsg(moveit_robot_traj_msg);

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = moveit_robot_traj_msg;
    confirmToAct();
    executePlan(plan);
}

double JRCMotionPlanner::cartesionPathPlanner(double distance_x, double distance_y, double distance_z)
{
    int x = distance_x * 1000;
    int y = distance_y * 1000;
    int z = distance_z * 1000;
    int max = std::max(x, y);
    max = std::max(max, z);
    int num_step = max;

    return cartesionPathPlanner(distance_x, distance_y, distance_z, num_step, 2);
}

void JRCMotionPlanner::confirmToAct()
{
    ROS_INFO_STREAM("Confirm start and end info and press n to start plan");
    if(confirm_act_)
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_ || "N" == pause_) {
            ROS_INFO_STREAM("Valid info, begin to move");
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
    ROS_INFO_STREAM("Confirm start ---> goal info, press n to start plan");
    if(confirm_act_)
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Corrent state, begin to plan");
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
{
    ROS_INFO_STREAM("Move from: " << start << "to " << goal);
    ROS_INFO_STREAM("Confirm start ---> goal info, press n to start plan");
    if(confirm_act_)
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Corrent state, begin to plan");
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
    ROS_INFO_STREAM("Confirm start ---> goal info, press n to start plan");
    if(confirm_act_)
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Correct state, begin to plan");
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(const geometry_msgs::Pose &goal)
{
//    std::cout << "\n"
//         << "=================MOVE TO " + str + "=================="
//         << "\n";
    ROS_INFO_STREAM("Move to target" << goal);
    ROS_INFO_STREAM("Confirm start ---> goal info, press n to start plan");
    if(confirm_act_)
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            ROS_INFO_STREAM("Correct state, begin to plan");
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

    moveit::planning_interface::MoveGroup::Plan temp_plan,best_plan;
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
                ROS_INFO_STREAM(current_time);
            }
        }
    }
    ROS_INFO_STREAM("Motion Planning 100 times duration: " << (ros::Time::now() - start_time).toSec() << "s");

    bool plan_valid = false;
    int plan_steps = 0;

    plan_steps = getPlanPointNum(best_plan);
    if (plan_steps < max_plan_steps_) {
        ROS_INFO_STREAM("Plan found in " << best_plan.planning_time_
                               << " seconds with " << plan_steps << " steps");
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
void JRCMotionPlanner::moveToTargetBestTime(const geometry_msgs::Pose &target)
{
    group_->setStartStateToCurrentState();
    group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroup::Plan temp_plan,best_plan;
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
                if(debug_print_)
                {
                    ROS_INFO_STREAM(current_time);
                }

            }
        }
        if((ros::Time::now() - start_time)>timeout)
        {
            break;
            ROS_ERROR("No solution in 5s!");
        }
    }
    ROS_INFO_STREAM("Motion Planning 100 times duration: " << (ros::Time::now() - start_time).toSec() << "s");

    bool plan_valid = false;
    int plan_steps = 0;

    plan_steps = getPlanPointNum(best_plan);
    if (plan_steps < max_plan_steps_) {
        ROS_INFO_STREAM("Plan found in " << best_plan.planning_time_
                               << " seconds with " << plan_steps << " steps");
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

void JRCMotionPlanner::moveToTargetBestTime(const geometry_msgs::PoseStamped &target)
{
    moveToTargetBestTime(target.pose);
}

// move line by MoveIt computeCartesianPath functions
void JRCMotionPlanner::moveLineTarget(const geometry_msgs::Pose &goal)
{

    ROS_INFO("Begin cartesian line plan by MoveIt computeCartesianPath ...");

    geometry_msgs::PoseStamped start_stamped =  getCurrentPoseFromMoveit();
    geometry_msgs::Pose start_pose = start_stamped.pose;
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
    double fraction = group_->computeCartesianPath(waypoints,
                                                   eef_step,
                                                   jump_threshold,
                                                   trajectory);
    if(fraction == 1.0){
        ROS_INFO("computeCartesionPath Successfully");
    }
    else if(fraction == -1.0){
        ROS_ERROR("computeCartesionPath ERROR!");
    }
    else {
        ROS_ERROR_STREAM("computeCartesionPath : " << fraction*100 << " %");
    }

    std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

    moveit::planning_interface::MoveGroup::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    int plan_steps = getPlanPointNum(cartesian_plan);
    ROS_INFO_STREAM("Line plan steps: " << plan_steps);
    if (plan_steps < max_cartesion_plan_steps_) {
        ROS_INFO_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps");
        confirmToAct(start_pose,goal);
        executePlan(cartesian_plan);
    } else {
        ROS_ERROR_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps");
        exit(0); // TODO
    }
}

void JRCMotionPlanner::moveLineTarget(const geometry_msgs::PoseStamped &goal)
{
    moveLineTarget(goal.pose);
}

std::size_t JRCMotionPlanner::getPlanPointNum(const moveit::planning_interface::MoveGroup::Plan &plan)
{
    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
    return trajectory.joint_trajectory.points.size();
}

void JRCMotionPlanner::moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
    ROS_INFO("Begin cartesian line plan by MoveIt computeCartesianPath ...");

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
    double fraction = group_->computeCartesianPath(waypoints,
                                                   eef_step,
                                                   jump_threshold,
                                                   trajectory);
    if(fraction == 1.0){
        ROS_INFO("computeCartesionPath Successfully");
    }
    else if(fraction == -1.0){
        ROS_ERROR("computeCartesionPath ERROR!");
    }
    else {
        ROS_ERROR_STREAM("computeCartesionPath : " << fraction*100 << " %");
    }

    std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

    moveit::planning_interface::MoveGroup::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    int plan_steps = getPlanPointNum(cartesian_plan);
    ROS_INFO_STREAM("Line plan steps: " << plan_steps);
    if (plan_steps < max_cartesion_plan_steps_) {
        ROS_INFO_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps");
        confirmToAct(start_pose,goal);
        executePlan(cartesian_plan);
    } else {
        ROS_ERROR_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps");
        exit(0); // TODO
    }
}
