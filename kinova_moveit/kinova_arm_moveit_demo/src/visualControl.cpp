/*
 * Receive target point from kinect and move arm to the target
 * and grasp
*/
#include "darknet_ros_msgs/TargetPoint.h"
#include "darknet_ros_msgs/TargetPoints.h"
#include "id_data_msgs/ID_Data.h"
#include "id_data_msgs/ID_Data.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <add_scene_objects.h> // handle scene obstacles
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Std C++ headers
#include <map>
#include <string>
#include <vector>
// test hk
#include <pick_place.h>

#include "id_data_msgs/ID_Data.h" //using for notie event
#include "ros/callback_queue.h"

using namespace std;
#define PREGRASP_OFFSET 0.20;

string robot_name = "j2n6s300";
geometry_msgs::PoseStamped goal_pose;
darknet_ros_msgs::TargetPoint canPoint;
bool receive_msg_flag = false;

void kinectCallback(const darknet_ros_msgs::TargetPoints::ConstPtr &msg) {
  //  TODO
  if (receive_msg_flag == false) {
    for (int i = 0; msg->target_points.size(); i++) {
      if (msg->target_points[i].Class == "can") {
        canPoint = msg->target_points[i];
        break;
      }
    }
    goal_pose.header.frame_id = "root";
    goal_pose.pose.position.x = canPoint.camera_x;
    goal_pose.pose.position.y = canPoint.camera_y;
    goal_pose.pose.position.z = canPoint.camera_z + 0.03;
    goal_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);
    receive_msg_flag = true;

    ROS_INFO_STREAM("Object postion: " << canPoint.camera_x << ", "
                                       << canPoint.camera_y << ", "
                                       << canPoint.camera_z);
  } else
    return;
}

class notice_pub_sub {
public:
  boost::function<void(const id_data_msgs::ID_Data::ConstPtr &)>
      notice_pub_sub_msgCallbackFun;

  notice_pub_sub();
  void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
  void notice_display(id_data_msgs::ID_Data notice_msg, bool set);
  void notice_sub_spinner(char set);
  void notice_data_clear(id_data_msgs::ID_Data *test);

private:
  ros::NodeHandle notice_handle;
  ros::Subscriber notice_subscriber;
  ros::Publisher notice_publisher;
  ros::SubscribeOptions notice_ops;
  ros::AsyncSpinner *notice_spinner;
  ros::CallbackQueue notice_callbackqueue;
  void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg);
};

notice_pub_sub::notice_pub_sub() {
  notice_pub_sub_msgCallbackFun =
      boost::bind(&notice_pub_sub::notice_msgCallback, this, _1);
  notice_ops = ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
      "/notice", 50, notice_pub_sub_msgCallbackFun, ros::VoidPtr(),
      &notice_callbackqueue);
  notice_subscriber = notice_handle.subscribe(notice_ops);
  notice_spinner = new ros::AsyncSpinner(1, &notice_callbackqueue);

  notice_publisher =
      notice_handle.advertise<id_data_msgs::ID_Data>("/notice", 50);
}

void notice_pub_sub::notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data) {
  notice_publisher.publish(id_data);
}

void notice_pub_sub::notice_display(id_data_msgs::ID_Data notice_msg,
                                    bool set) {

  if (set) {
    printf("REC Notice message,ID: %d,Data: ", notice_msg.id);
    for (char i = 0; i < 8; i++) {
      printf("%d ", notice_msg.data[i]);
      if (i == 7)
        printf("\n");
    }
  }
}
void notice_pub_sub::notice_msgCallback(
    const id_data_msgs::ID_Data::ConstPtr &notice_msg) {

  id_data_msgs::ID_Data notice_message;
  notice_message.id = 0;
  for (char i = 0; i < 8; i++)
    notice_message.data[i] = 0;

  notice_message.id = notice_msg->id;
  for (char i = 0; i < 8; i++)
    notice_message.data[i] = notice_msg->data[i];

  notice_pub_sub::notice_display(notice_message, true);

  //     //1,navigation section
  //     if(notice_message.id==2 && notice_message.data[0]==14)//msg received
  //     flag
  //     {
  //         nav_msg_rec_flag=true;
  //     }
  //     if(notice_message.id==2 && notice_message.data[0]==15)//nav finished
  //     flag
  //     {
  //         nav_finished_flag=true;
  //     }
  //     if(notice_message.id==2 && notice_message.data[0]==16)//nav finished
  //     flag
  //     {
  //         set_ontime++;
  //         ROS_WARN("ON TIME No.%d",set_ontime);
  //     }
  //     if(notice_message.id==2 && notice_message.data[0]==8)
  //     {
  //         command_move_finished_flag=true;
  //         ROS_INFO("Received dashgo command move finished flag");
  //     }

  //     //2,kinect scan section
  //     if(notice_message.id==3 && notice_message.data[0]==14)//kinect msg
  //     received flag
  //     {
  //         kinect_msg_rec_flag=true;
  //     }
  //     if(notice_message.id==3 && notice_message.data[0]==15)//kinect scan
  //     finished flag
  //     {
  //         kinect_scan_finished_flag=true;
  //     }
  //     if(notice_message.id==3 && notice_message.data[0]==13)//kinect scan
  //     failed
  //     {
  // //        id_data_msgs::ID_Data id_data;
  // //        id_data.id=3;
  // //        for(int count=0;count<8;count++) id_data.data[count]=0;
  // //        id_data.data[0]=1;
  // //        notice_publisher.publish(id_data);

  //         row[obj_num]=row[obj_cnt-1];
  //         column[obj_num]=column[obj_cnt-1];
  //         obj_num++;
  //         kinect_reset_flag=true;
  //         ROS_WARN("Received missing current column flag.Save data
  //         No.%d,Row:%d,Col%d",obj_cnt,row[obj_cnt-1],column[obj_cnt-1]);
  //         kinect_scan_finished_flag=true;

  //     }
  //     //3,arm control section
  //     if(notice_message.id==4 && notice_message.data[0]==14)//arm control msg
  //     received flag
  //     {
  //         arm_msg_rec_flag=true;
  //     }
  //     if(notice_message.id==4 && notice_message.data[0]==15)//arm fetch
  //     finished flag
  //     {
  //         arm_act_finished_flag=true;
  //     }
}

void notice_pub_sub::notice_sub_spinner(char set) {
  if (set == 1)
    notice_spinner->start();
  if (set == 0)
    notice_spinner->stop();
}

void notice_pub_sub::notice_data_clear(id_data_msgs::ID_Data *test)
{
    test->id=0;
    for(int i=0;i<8;i++) test->data[i]=0;
}

// get end effector state
// void getCurrentState(const geometry_msgs::PoseStampedConstPtr &msg) {
//   ROS_INFO("Get current state from kinova driver");
//   current_pose_ = *msg;
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualControl");
  notice_pub_sub notice_test;
  id_data_msgs::ID_Data notice_data_pub;

  // dfault target value
  goal_pose.header.frame_id = "root";
  // goal_pose.pose.position.x = -0.2;
  // goal_pose.pose.position.y = -0.4;
  // goal_pose.pose.position.z = 0.5;
  goal_pose.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(1.57, -1.0, 0.0);

  if (argc < 4) {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun Jaco moveit plan_arm_ik  x y z  r p y");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of "
             "/arm_tool_link expressed in world frame");
    ROS_INFO("Plan with default target point ");

  } else {
    ROS_INFO("Plan with manually defined target point ");
    goal_pose.pose.position.x = atof(argv[1]);
    goal_pose.pose.position.y = atof(argv[2]);
    goal_pose.pose.position.z = atof(argv[3]);
  }

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // test hk
  kinova::PickPlace pick_place(nh);

  ros::Subscriber subKinect =
      nh.subscribe("/darknet_ros/target_points", 100, kinectCallback);
  ros::spinOnce();

  // if (atof(argv[2]) > -0.15) {
  //   ROS_INFO("Wrong posiiton, y value should be less then -0.15");
  //   return EXIT_FAILURE;
  // }
  ROS_INFO("Check object pose returned form kinect, print n to next");
  string pause_;
  cin >> pause_;
  if ("n" == pause_) {
    ROS_INFO("Validate target pose, proceed to next");
  } else {
    return EXIT_FAILURE;
  }

  // the end-effector frame is declared in moveit config, can not be changed
  // group_arm.setEndEffectorLink("j2n6s300_link_6");

  // add collision objects to robot working space
  ROS_INFO("Insert scene objects in workspace");
  build_workScene buildWorkScene(nh);
  geometry_msgs::Pose tablePose, kinectBasePose, kinectBodyPose;

  tablePose.position.x = 0;
  tablePose.position.y = 0;
  tablePose.position.z = 0.1; // avoid collision between arm and surface
  buildWorkScene.add_boxModel("table", 1.5, 1.5, 0.01, tablePose);

  // add kinect stick obstacle
  kinectBasePose.position.x = 0.35;
  kinectBasePose.position.y = 0;
  kinectBasePose.position.z = 0.4;
  tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 1.57);
  // tf::Quaternion qkinect = tf::createQuaternionFromRPY(0, 1.57, 0);
  // ROS_INFO_STREAM("Kinect obs pose: " << qkinect);
  kinectBasePose.orientation.x = qkinect[0];
  kinectBasePose.orientation.y = qkinect[1];
  kinectBasePose.orientation.z = qkinect[2];
  kinectBasePose.orientation.w = qkinect[3];
  buildWorkScene.add_boxModel("kinectBase", 0.6, 0.03, 0.02, kinectBasePose);

  // add kinect body obstacle
  kinectBodyPose.position.x = 0.35;  // 0.35 VS 0.3
  kinectBodyPose.position.y = -0.05; //-0.05 VS 0
  kinectBodyPose.position.z = 0.75;  // 0.75
  tf::Quaternion qbody = tf::createQuaternionFromRPY(0, 0, -1.57);
  // tf::Quaternion qbody = tf::createQuaternionFromRPY(0, -0.26, 0);
  // ROS_INFO_STREAM("Kinect obs pose: " << qbody);
  kinectBodyPose.orientation.x = qbody[0];
  kinectBodyPose.orientation.y = qbody[1];
  kinectBodyPose.orientation.z = qbody[2];
  kinectBodyPose.orientation.w = qbody[3];
  buildWorkScene.add_boxModel("kinectBody", 0.1, 0.3, 0.1, kinectBodyPose);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // add collision objects to world
  planning_scene_interface.addCollisionObjects(
      buildWorkScene.collision_objects);

  ROS_INFO("Collision setup finished");

  // select group of joints
  geometry_msgs::PoseStamped pregrasp_pose = goal_pose;
  pregrasp_pose.pose.position.y += PREGRASP_OFFSET;
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroup group_arm("arm");
  ROS_INFO_STREAM("Planning to move " << group_arm.getEndEffectorLink()
                                      << " to a target pose expressed in "
                                      << group_arm.getPlanningFrame()
                                      << " with obstacles");
  // const robot_state::JointModelGroup *joint_model_group =
  //     group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // choose your preferred planner, the following planner works bad
  // group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPoseReferenceFrame("world");
  group_arm.setPoseTarget(pregrasp_pose);
  ROS_INFO_STREAM("Object postion: " << pregrasp_pose.pose.position.x << ", "
                                     << pregrasp_pose.pose.position.y << ", "
                                     << pregrasp_pose.pose.position.z);
  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(0.6);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  // set maximum time to find a plan
  group_arm.setPlanningTime(5.0);
  bool success = group_arm.plan(my_plan);

  if (!success)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  /**************************stretch stage*******************/
  sleep(3);

  // get current pose
  // tf::TransformListener listener;
  // while (ros::ok()) {
  //   listener.transformPoint(robot_name + "_link_base", ee_pose,
  //   current_pose);

  //   ROS_INFO(
  //       "current pose: (%.2f, %.2f. %.2f) -----> base_link: (%.3f, %.3f,
  //       "
  //       "%.3f) at time %.2f",
  //       ee_pose.point.x, ee_pose.point.y, ee_pose.point.z,
  //       current_pose.point.x, current_pose.point.y, current_pose.point.z,
  //       current_pose.header.stamp.toSec());
  // } catch (tf::TransformException &ex) {
  //   ROS_ERROR("Received an exception trying to transform a point from "
  //             "\"ee_frame\" to \"base_link\": %s",
  //             ex.what());
  //   tf::StampedTransform transform;
  //   try {
  //     listener.lookupTransform("/root", "/j2n6s300_link_6", ros::Time(0),
  //                              transform);
  //   } catch (tf::TransformException ex) {
  //     ROS_ERROR("%s", ex.what());
  //     ros::Duration(1.0).sleep();
  //   }

  //   if (abs(transform.getOrigin().x() - pregrasp_pose.pose.position.x) < 0.2)
  //   {
  //     current_pose.position.x = transform.getOrigin().x();
  //     current_pose.position.y = transform.getOrigin().y();
  //     current_pose.position.z = transform.getOrigin().z();

  //     tf::Quaternion q = transform.getRotation();
  //     current_pose.orientation.x = q.x();
  //     current_pose.orientation.y = q.y();
  //     current_pose.orientation.z = q.z();
  //     current_pose.orientation.w = q.w();

  //     // end effector point
  //     pick_place.get_current_pose();

  //     break;
  //   }
  // }

  geometry_msgs::Pose current_pose;
  current_pose = pick_place.get_ee_pose();
  current_pose.orientation = pregrasp_pose.pose.orientation; // target pose

  ROS_INFO("CURRENT POSE: (%.2f, %.2f. %.2f) -----> PREGRASP POSE: (%.3f, "
           "%.3f,%.3f) at time %.2f",
           current_pose.position.x, current_pose.position.y,
           current_pose.position.z, pregrasp_pose.pose.position.x,
           pregrasp_pose.pose.position.y, pregrasp_pose.pose.position.z,
           pregrasp_pose.header.stamp.toSec());

  string pause;
  ROS_INFO_STREAM("Print n to excute the straight line plan");
  cin >> pause;
  if ("n" == pause) {
    ROS_INFO("Begin cartesian line plan");
  } else {
    return EXIT_FAILURE;
  }

  // Cartesian Path
  geometry_msgs::Pose way_pose = current_pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(way_pose); // first pose waypoint
  int num_waypoint = 5;
  float delta_x = (goal_pose.pose.position.x - current_pose.position.x) /
                  (num_waypoint - 1);
  float delta_y = (goal_pose.pose.position.y - current_pose.position.y) /
                  (num_waypoint - 1);
  float delta_z = (goal_pose.pose.position.z - current_pose.position.z) /
                  (num_waypoint - 1);

  // interplotate between current pose and target pose
  for (int i = 0; i < num_waypoint - 1; i++) {
    way_pose.position.x += delta_x;
    way_pose.position.y += delta_y;
    way_pose.position.z += delta_z;
    waypoints.push_back(way_pose);
  }

  group_arm.setMaxVelocityScalingFactor(0.2);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = group_arm.computeCartesianPath(waypoints, eef_step,
                                                   jump_threshold, trajectory);
  // ROS_INFO_NAMED("tutorial",
  //                "Visualizing plan (Cartesian path) (%.2f%% acheived)",
  //                fraction * 100.0);

  // joint_name_size = trajectory.joint_trajectory.joint_names.size();
  // plan_point_size = trajectory.joint_trajectory.points.size();
  // std::cout << "joint_name size:" << joint_name_size << std::endl;
  // std::cout << "points  size:" << plan_point_size << std::endl;
  // for (int i = 0; i < joint_name_size; i++) {
  //   ROS_INFO("joint name:%s",
  //            trajectory.joint_trajectory.joint_names[i].c_str());
  // }
  // ROS_INFO_STREAM("Last point of Plan:"
  //                 << trajectory.joint_trajectory.points[plan_point_size -
  //                 1]);

  moveit::planning_interface::MoveGroup::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  group_arm.execute(cartesian_plan);

  // GRASP TEST HK
  ROS_INFO("Begin grasp demo, press n to next:");
  cin >> pause;

  if ("n" == pause) {
    ROS_INFO("Begin grasp demo");
    
  } else {
    return EXIT_FAILURE;
  }


  notice_test.notice_data_clear(&notice_data_pub);
  notice_data_pub.id = 1;
  notice_data_pub.data[0] = 1;
  notice_test.notice_pub_sub_pulisher(notice_data_pub);

  ros::Duration(5).sleep();

  spinner.stop();

  return EXIT_SUCCESS;
}
