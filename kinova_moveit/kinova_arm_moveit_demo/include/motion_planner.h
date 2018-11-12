/* Copyright SIA FreeDream.
   Author: Cong Wang
   Desc:   Motion planner class to achieve the motion plan
*/

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>


class MotionPlanner
{
public:
    /** \brief Constructor */
    MotionPlanner(ros::NodeHandle& nh);

    /** \brief Destructor */
    ~MotionPlanner();

    double cartesion_path_planner(const double &distance_x, const double &distance_y, const double &distance_z, 
                             const double &theta_x, const double &theta_y, const double &theta_z,
                             const int num_interpolation_cart, const double num_interpolation_joint);

    double cartesion_path_planner(const double &distance_x, const double &distance_y, const double &distance_z, 
                             const int num_interpolation_cart, const double num_interpolation_joint);

    double cartesion_path_planner(const double &distance_x, const double &distance_y, const double &distance_z, 
                             const double &theta_x, const double &theta_y, const double &theta_z,
                             const int num_interpolation_cart, const double num_interpolation_joint);

    bool move_to_target();

    void confirm_to_act();
    void 

private:
    ros::NodeHandle nh_;

    std::string group = "arm";
    moveit::move_group_interface::move_group group("arm");
};


#endif