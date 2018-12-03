/***********************************
*
*
* Utilis functions
*
*
************************************/

#ifndef FUCTIONS_H
#define FUCTIONS_H

// Std C++ headers
#include <iostream>
#include <map>
#include <string>
#include <vector>

// ros head files
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>


// custom headers

#include "id_data_msgs/ID_Data.h" //using for notie event
#include <add_scene_objects.h>    // handle scene obstacles


using namespace std;

// function declaration
void handleCollisionObj(build_workScene& buildWorkScene);

#endif // FUNCTIONS_H
