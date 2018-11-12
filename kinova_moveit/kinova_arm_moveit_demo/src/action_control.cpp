

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include "parser.h"


#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
// #include <actionlib/client/SimpleActionClient.h>
#include <actionlib/client/simple_action_client.h>  

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h> 
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Std C++ headers
#include <map>
#include <string>
#include <vector>

// custom headers
#include "darknet_ros_msgs/TargetPoint.h" // kinect info
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h" //using for notie event
#include <add_scene_objects.h>    // handle scene obstacles
#include <pick_place.h>
#include "parser.h"

std::vector<double> current_joint_values;
using namespace std;

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

void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ,int number_point, int number_distance);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_control");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Subscriber sub_joint_values = nh.subscribe("/j2n6s300_driver/out/joint_state", 100, currentJointValuesCallback);
    ros::Rate rate(10);

    


    moveit::planning_interface::MoveGroup group("arm");

    std::vector<double> joint_values;
    joint_values = group.getCurrentJointValues();
    geometry_msgs::PoseStamped current_pose;
    current_pose = group.getCurrentPose();
    for (size_t i = 0; i < 6; i++)
    {
        cout << joint_values[i] << endl;
    }
    cout << current_pose << endl;
    
    while(ros::ok())
    {
        for(size_t i=0; i<6; i++)
        {
            cout << current_joint_values[i] << endl;
        }
        cout << endl;
        sleep(5.0);
        ros::spinOnce();
        rate.sleep();
    }
    // ros::spinOnce();
    // moveLineFromCurrentState(0, 0, 0.1, 5,2);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ros::spinOnce();


    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  ac("/j2n6s300/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");  
    ac.waitForServer();  
    ROS_INFO("Action server started, sending trajectory.");  

    // moveit::planning_interface::MoveGroup group("arm");
    // std::vector<double> joint_values = group.getCurrentJointValues();
    control_msgs::FollowJointTrajectoryGoal goal;

    Eigen::VectorXd qPre(6);
    // ros::spinOnce();
    for(size_t i=0; i<6; i++)
    {
        cout << current_joint_values[i] << endl;
    }
    cout << endl;
    // sleep(5.0);
    
    qPre << current_joint_values[0], current_joint_values[1], 
            current_joint_values[2], current_joint_values[3], 
            current_joint_values[4], current_joint_values[5];
    
    // qPre << joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4], joint_values[5];
    cout << "qPre: " << "\n" << qPre << endl;
    Parser parser;
    Eigen::Matrix4d transformation = parser.Foward(qPre);
    cout << "transformation: " << "\n" << transformation << endl;
    goal.trajectory.header.frame_id = "j2n6s300_base";
    goal.trajectory.header.stamp = ros::Time::now();

    goal.trajectory.joint_names.clear();
    for (int k = 0; k < 6; k++)
    {
        stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
    }

    goal.trajectory.points.clear();

    int number1 = 5, number2 = 2;
    double distanceX = 0;
    double distanceY = 0;
    double distanceZ = 0.1;
    // int number1 = number_point, number2 = number_distance;
    cout << "number1: " << number1 << endl;
    cout << "number2: " << number2 << endl;
    for (int i = 0; i < number1; i++)
    {
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;
        Eigen::VectorXd q(6);
        q = parser.Inverse(transformation, qPre);
        cout << "=========" << endl;
        if (q(0) > 10) continue;

        printf("loops:%d",i);
        for (int j = 0; j < number2; j++)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++)
            {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number2 * j);
                //point.velocities.push_back(0.01); // 0.1  
                //point.accelerations.push_back(0.01); 
            }
            point.time_from_start = ros::Duration(5);
            goal.trajectory.points.push_back(point);
        }
        printf("\n\njoint values : %d\n",i);
        std::cout << q << std::endl;
        qPre = q;
    }
    ROS_INFO("\n\n--------------\n\n");
    // ac.sendGoal(goal);
    // ac.waitForResult(ros::Duration(10.0));
    ROS_INFO("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");

    return 0;
}




void moveLineFromCurrentState(double distanceX, double distanceY, double distanceZ,int number_point, int number_distance)
{
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  ac("/j2n6s300/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");  
    ac.waitForServer();  
    ROS_INFO("Action server started, sending trajectory.");  

    // moveit::planning_interface::MoveGroup group("arm");
    // std::vector<double> joint_values = group.getCurrentJointValues();
    control_msgs::FollowJointTrajectoryGoal goal;

    Eigen::VectorXd qPre(6);
    // ros::spinOnce();
    for(size_t i=0; i<6; i++)
    {
        cout << current_joint_values[i] << endl;
    }
    cout << endl;
    // sleep(5.0);
    
    qPre << current_joint_values[0], current_joint_values[1], 
            current_joint_values[2], current_joint_values[3], 
            current_joint_values[4], current_joint_values[5];
    
    // qPre << joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4], joint_values[5];
    cout << "qPre: " << "\n" << qPre << endl;
    Parser parser;
    Eigen::Matrix4d transformation = parser.Foward(qPre);
    cout << "transformation: " << "\n" << transformation << endl;
    goal.trajectory.header.frame_id = "j2n6s300_base";
    goal.trajectory.header.stamp = ros::Time::now();

    goal.trajectory.joint_names.clear();
    for (int k = 0; k < 6; k++)
    {
        stringstream jointName;
        jointName << "j2n6s300_joint_" << (k + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
    }

    goal.trajectory.points.clear();

    int number1 = number_point, number2 = number_distance;
    cout << "number1: " << number1 << endl;
    cout << "number2: " << number2 << endl;
    for (int i = 0; i < number1; i++)
    {
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;
        Eigen::VectorXd q(6);
        q = parser.Inverse(transformation, qPre);
        cout << "=========" << endl;
        if (q(0) > 10) continue;

        printf("loops:%d",i);
        for (int j = 0; j < number2; j++)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            for (int k = 0; k < 6; k++)
            {
                point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number2 * j);
                //point.velocities.push_back(0.01); // 0.1  
                //point.accelerations.push_back(0.01); 
            }
            point.time_from_start = ros::Duration(5);
            goal.trajectory.points.push_back(point);
        }
        printf("\n\njoint values : %d\n",i);
        std::cout << q << std::endl;
        qPre = q;
    }
    ROS_INFO("\n\n--------------\n\n");
    // ac.sendGoal(goal);
    // ac.waitForResult(ros::Duration(10.0));
    ROS_INFO("\n\nMOVE TO TARGET SUCCESSFULLY\n\n");

}
