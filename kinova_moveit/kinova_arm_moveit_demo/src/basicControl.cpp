#include <ros/ros.h>
#include <ros/console.h>

#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
//moveit
//move_group.h file uses move_group_interface.h header file actually.
// #include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
//want to cite PickPlace class of pick_place.h
#include <pick_place.h>
#include <visualization_msgs/Marker.h>
using namespace kinova;

const double FINGER_MAX = 6400;

const char *hollowPrism="package://kinova_description/meshes/hollowPrism.STL";
std::string pause_;

tf::Quaternion EulerZYZ2Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}

class build_workScene
{
public:
    build_workScene(ros::NodeHandle &nh);
    void add_boxModel(const char *name,float length, float width, float height, geometry_msgs::Pose prePose);
    void add_meshModel(const char *name,const char* path, geometry_msgs::Pose prePose);
    void clear_WorkScene(const char *objName);

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher pub ;
    ros::Publisher pub_aco_;
    ros::Publisher pub_planning_scene_diff_;

    moveit_msgs::CollisionObject co_;
    geometry_msgs::PoseStamped can_pose_;
    //work scene
    moveit_msgs::AttachedCollisionObject aco_;
    moveit_msgs::PlanningScene planning_scene_msg_;
};

build_workScene::build_workScene(ros::NodeHandle &nh):nh_(nh)
{
    pub = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Duration(1.0).sleep();
}

void build_workScene::add_boxModel(const char *name,float length, float width, float height, geometry_msgs::Pose prePose)
{
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();

    co_.id = name;
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub.publish(co_);

    // add table
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = length;//table:2.4*2.4*0.03
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;
    co_.primitive_poses[0].position.x =prePose.position.x;//x=0,y=0,z=-0.03/2.0
    co_.primitive_poses[0].position.y =prePose.position.y; 0.0;
    co_.primitive_poses[0].position.z =prePose.position.z;
    
    co_.primitive_poses[0].orientation.x =prePose.orientation.x;
    co_.primitive_poses[0].orientation.y =prePose.orientation.y;
    co_.primitive_poses[0].orientation.z =prePose.orientation.z;
    co_.primitive_poses[0].orientation.w =prePose.orientation.w;
    pub.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();    
}

void build_workScene::add_meshModel(const char *name,const char* path, geometry_msgs::Pose prePose)
{  
    co_.id = name;
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub.publish(co_);

    //add hollowPrism
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    shapes::Mesh* hollowPrism_shape = shapes::createMeshFromResource(path);
    shapes::ShapeMsg hollowPrism_mesh_msg;
    shapes::constructMsgFromShape(hollowPrism_shape, hollowPrism_mesh_msg);
    shape_msgs::Mesh hollowPrism_mesh = boost::get<shape_msgs::Mesh>(hollowPrism_mesh_msg);

    co_.meshes.push_back(hollowPrism_mesh);
    co_.mesh_poses.push_back(prePose);
    can_pose_.pose.position.x = co_.primitive_poses[0].position.x;
    can_pose_.pose.position.y = co_.primitive_poses[0].position.y;
    can_pose_.pose.position.z = co_.primitive_poses[0].position.z;
    
    pub.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_); 
}

void build_workScene::clear_WorkScene(const char *objName)
{
    co_.id = objName;
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub.publish(co_);

    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
}

moveit::planning_interface::MoveGroup::Plan evaluate_plan(moveit::planning_interface::MoveGroup &group)
{
    bool replan = true;
    int count = 0;
    moveit::planning_interface::MoveItErrorCode result_;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    while (replan == true && ros::ok())
    {
        // reset flag for replan
        count = 0;
        result_ = false;

        // try to find a success plan.
        double plan_time;
        while (result_ == false && count < 5)
        {
            count++;
            plan_time = 10+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result_ = group.plan(my_plan);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result_ == true)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();
            if (pause_ == "r" || pause_ == "R" )
            {
                replan = true;
            }
            else
            {
                replan = false;
            }
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result_ == true)
    {
        if (pause_ == "e" || pause_ == "E")
        {
            group.execute(my_plan);
        }
        return my_plan;
    }
   
    ros::WallDuration(1.0).sleep();
}

bool gripper_action(bool robot_connected_, 
                    moveit::planning_interface::MoveGroup* gripper_group_, 
                    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_,
                    double finger_turn)
{
    if(robot_connected_ == false)
    {
        if (finger_turn>0.5*FINGER_MAX)
        {
          gripper_group_->setNamedTarget("Close");
        }
        else
        {
          gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

class marker_visualize
{
public:
    marker_visualize(ros::NodeHandle &nh);
private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub;
    visualization_msgs::Marker points;
    visualization_msgs::Marker line_strip;

public:
    void add_markerPoints(int marker_id, geometry_msgs::Pose &pose_);
    void add_markerLinestrip(int marker_id, geometry_msgs::Pose &pose_);
    void clear_AllMarker();
};

marker_visualize::marker_visualize(ros::NodeHandle &nh):n_(nh)
{
    marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    
    points.header.frame_id = "root"; 
    points.header.stamp = ros::Time::now();
    points.ns = "marker_points";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.scale.x = 0.01;
    points.scale.y = 0.01;
    points.scale.z = 0.01;    
    //points color is green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    
    line_strip.header.frame_id = "root";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns =  "marker_line";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    //Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
}

void marker_visualize::add_markerPoints(int marker_id, geometry_msgs::Pose &pose_)
{
    points.id = marker_id;
    geometry_msgs::Point p;
    p.x = pose_.position.x;
    p.y = pose_.position.y;
    p.z = pose_.position.z;
    points.points.push_back(p);
    marker_pub.publish(points);
    ros::Duration(0.1).sleep();
}

void marker_visualize::add_markerLinestrip(int marker_id, geometry_msgs::Pose &pose_)
{
    line_strip.id = marker_id;
    geometry_msgs::Point p;
    p.x = pose_.position.x;
    p.y = pose_.position.y;
    p.z = pose_.position.z;
    line_strip.points.push_back(p);
    marker_pub.publish(line_strip);
    ros::Duration(0.1).sleep();
}

void marker_visualize::clear_AllMarker()
{
    visualization_msgs::Marker unknown_marker;
    unknown_marker.id = 0;
    unknown_marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(unknown_marker);
    ros::Duration(0.1).sleep();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //build work scene --------------------------------start--------------------------------------------------------->
    // primitive sphere
    ros::NodeHandle n;
    ROS_INFO("Insert scene objects in workspace");
    build_workScene buildWorkScene(n);
    geometry_msgs::Pose tablePose, prismPose, hollowPrismPose;
    
    tablePose.position.x=0;
    tablePose.position.y=0;
    tablePose.position.z=-0.03/2.0;
    buildWorkScene.add_boxModel("table", 2.4, 2.4, 0.03, tablePose);
    
    prismPose.position.x = 0.1;
    prismPose.position.y = -0.65;
    prismPose.position.z = 0.1;
    // buildWorkScene.add_boxModel("prism", 0.05, 0.05, 0.2, prismPose);

    hollowPrismPose.position.x = 0.4;
    hollowPrismPose.position.y = -0.3;
    hollowPrismPose.position.z = 0.0;
    hollowPrismPose.orientation.w = 1.0;
    // buildWorkScene.add_meshModel("hollowPrism", hollowPrism, hollowPrismPose);
    ROS_INFO_STREAM("Input re to remove all object or n to next:");

    std::cin >> pause_;

    ros::NodeHandle node_handle_marker;
    marker_visualize visualizeTrajectory(node_handle_marker);
    if(pause_ == "re" || pause_ == "RE")
    {
        buildWorkScene.clear_WorkScene("table");
        // buildWorkScene.clear_WorkScene("prism");
        // buildWorkScene.clear_WorkScene("hollowPrism");
        ROS_INFO("Objects have been removed!");
    }

    //move group c++ interface 
    kinova::PickPlace pick_place(node_handle);
    moveit::planning_interface::MoveGroup* group_; 

    group_ = new moveit::planning_interface::MoveGroup("arm");
    std::string robot_type_="j2n6s300";
    // can moveit handle a manual defined end-effector frame instead of frames parsed from urdf
    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    moveit::planning_interface::MoveGroup::Plan my_plan;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    
    group_->clearPathConstraints();
    group_->setNamedTarget("Home");
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Home pose reached ...");
    // std::cin >> pause_;
    
    tf::Quaternion q;
    geometry_msgs::PoseStamped start_pose_;
    // define start pose before grasp
    start_pose_.header.frame_id = "root";
    start_pose_.header.stamp = ros::Time::now();
    start_pose_.pose.position.x = 0.1;
    start_pose_.pose.position.y = -0.5;
    start_pose_.pose.position.z = 0.5;

    // q = EulerZYZ2Quaternion(-M_PI/4, M_PI/2, M_PI);
    q = EulerZYZ2Quaternion(0.0, M_PI/2, 0.0);
    start_pose_.pose.orientation.x = q.x();
    start_pose_.pose.orientation.y = q.y();
    start_pose_.pose.orientation.z = q.z();
    start_pose_.pose.orientation.w = q.w();

    // start_pose_.pose.orientation.x = 0.01;
    // start_pose_.pose.orientation.y = 0.71;
    // start_pose_.pose.orientation.z = 0.01;
    // start_pose_.pose.orientation.w = 0.71;

    ROS_INFO_STREAM("Input any char and Enter to move to predifined start pose (0.1, -0.5, 0.5) ...");
    std::cin >> pause_;
    group_->setPoseTarget(start_pose_);
    my_plan= evaluate_plan(*group_);
    
    // ROS_INFO("Display tracjectory!");
    // display_trajectory.trajectory_start = my_plan.start_state_;
    // display_trajectory.trajectory.push_back(my_plan.trajectory_);
    // display_publisher.publish(display_trajectory);

    // //ROS_INFO_STREAM(my_plan.start_state_);
    // //ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory);
    
    // int joint_name_size, plan_point_size;
    // joint_name_size = my_plan.trajectory_.joint_trajectory.joint_names.size();
    // plan_point_size = my_plan.trajectory_.joint_trajectory.points.size();
    // std::cout <<"joint_name size:"<< joint_name_size <<std::endl;
    // std::cout <<"points  size:"<< plan_point_size <<std::endl;
    // for(int i=0; i< joint_name_size;i++)
    // {
    //     ROS_INFO("joint name:%s", my_plan.trajectory_.joint_trajectory.joint_names[i].c_str());
    // }
    // ROS_INFO_STREAM("Last point of Plan:"<<my_plan.trajectory_.joint_trajectory.points[plan_point_size-1]);

    // for(int i=0;i<plan_point_size;i++)
    // {    
    //     std::vector<double> joint_value(6,0.0);
    //     geometry_msgs::Pose pose_;
    //     for(int j=0;j<6;j++)
    //     {
    //         joint_value[j]=my_plan.trajectory_.joint_trajectory.points[i].positions[j];
    //         if(j==5)
    //         {
    //             pick_place.getForK(joint_value, pose_);
    //             int point_id=0;
    //             int line_id=1;
    //             visualizeTrajectory.add_markerPoints(point_id, pose_);
    //             visualizeTrajectory.add_markerLinestrip(line_id, pose_);
    //             ROS_INFO("Planed Point:%f %f %f",pose_.position.x, pose_.position.y, pose_.position.z);
    //         }
    //     }
    // }
    
    // /* Sleep to give Rviz time to visualize the plan. */
    // sleep(5.0);
    // ROS_INFO("Forward Kinematic model compute test");
    // std::vector<double> forKJointValue(6,0.0); 
    // geometry_msgs::Pose joint_pose;
    // forKJointValue[0]= -2.479;
    // forKJointValue[1]=  3.738;
    // forKJointValue[2]=  1.829;
    // forKJointValue[3]= -0.252;
    // forKJointValue[4]= -0.415;
    // forKJointValue[5]=  0.440;
    // ROS_INFO("Joint value input:");
    // for(int i=0;i<forKJointValue.size();i++)
    // {
    //     std::cout<< forKJointValue[i]<<" ";
    //     if(i == forKJointValue.size()-1)
    //         std::cout<<std::endl;
    // }
    // pick_place.getForK(forKJointValue, joint_pose);
    
    // ROS_INFO("Inverse Kinematic model compute test");
    // std::vector<double> invKJointValue;  
    // // start_pose_.pose.position.x = 0.3;
    // // start_pose_.pose.position.y = -0.3;
    // // start_pose_.pose.position.z = 0.3;
    // ROS_INFO_STREAM("Target pose:"<<start_pose_.pose);
    // pick_place.getInvK(start_pose_.pose, invKJointValue);   
    // ROS_INFO("Set joint of InvK Compute");
    // group_->setJointValueTarget(invKJointValue);
    // evaluate_plan(*group_);
    // ros::Duration(5).sleep();    
    // //gripper control
    // pick_place.gripper_action(0);
    // pick_place.gripper_action(.75*FINGER_MAX);

    // //Cartesian Path
    // ROS_INFO("Cartesian Path TEST");
    // geometry_msgs::Pose start_pose2 = start_pose_.pose;
    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(start_pose2);

    // geometry_msgs::Pose target_pose3 = start_pose2;

    // target_pose3.position.z += 0.2;
    // waypoints.push_back(target_pose3);  // up

    // target_pose3.position.x -= 0.3;
    // waypoints.push_back(target_pose3);  // left

    // target_pose3.position.z -= 0.2;
    // waypoints.push_back(target_pose3);  // down and right

    // group_->setMaxVelocityScalingFactor(0.2);
    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // joint_name_size = trajectory.joint_trajectory.joint_names.size();
    // plan_point_size = trajectory.joint_trajectory.points.size();
    // std::cout <<"joint_name size:"<< joint_name_size <<std::endl;
    // std::cout <<"points  size:"<< plan_point_size <<std::endl;
    // for(int i=0; i< joint_name_size;i++)
    // {
    //     ROS_INFO("joint name:%s", trajectory.joint_trajectory.joint_names[i].c_str());
    // }
    // ROS_INFO_STREAM("Last point of Plan:"<<trajectory.joint_trajectory.points[plan_point_size-1]);

    // my_plan.trajectory_=trajectory;
    // group_->execute(my_plan);
    // ros::Duration(5).sleep();

    // for(int i=0;i<plan_point_size;i++)
    // {    
    //     std::vector<double> joint_value(6,0.0);
    //     geometry_msgs::Pose pose_;
    //     for(int j=0;j<6;j++)
    //     {
    //         joint_value[j]=trajectory.joint_trajectory.points[i].positions[j];
    //         if(j==5)
    //         {
    //             pick_place.getForK(joint_value, pose_);
    //             int point_id=0;
    //             int line_id=1;
    //             visualizeTrajectory.add_markerPoints(point_id, pose_);
    //             visualizeTrajectory.add_markerLinestrip(line_id, pose_);
    //             ROS_INFO("Planed Point:%f %f %f",pose_.position.x, pose_.position.y, pose_.position.z);
    //         }
    //     }
    // }

    //grasp test

   
    // ROS_INFO_STREAM("Input any char and Enter to move to grasp Start pose ...");
    // std::cin >> pause_;
    // start_pose_.pose.position.x = 0.1;
    // start_pose_.pose.position.y = -0.62;
    // start_pose_.pose.position.z = 0.1;
    // group_->setPoseTarget(start_pose_);
    // my_plan= evaluate_plan(*group_);
    // pick_place.gripper_action(.5*FINGER_MAX);
    // ros::Duration(2).sleep();
    // group_->attachObject("prism");


    // start_pose_.pose.position.x = 0.5;
    // start_pose_.pose.position.y = -0.5;
    // start_pose_.pose.position.z = 0.1;
    // ROS_INFO_STREAM("Input any char and Enter to move to grasp End pose ...");
    // std::cin >> pause_;
    // group_->setPoseTarget(start_pose_);
    // my_plan= evaluate_plan(*group_);
    // ros::Duration(5).sleep();
    // group_->detachObject("prism");

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    // std::vector<std::string> object_ids;
    // object_ids.push_back("prism");
    // planning_scene_interface.removeCollisionObjects(object_ids);
    // buildWorkScene.clear_WorkScene("prism");
    // ros::Duration(0.1).sleep();

    ros::shutdown();
    return 0;

}