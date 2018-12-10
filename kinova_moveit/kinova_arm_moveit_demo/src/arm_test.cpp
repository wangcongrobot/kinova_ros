/********************************************************
*
*
* Test arm driver
*
*
**********************************************************/

// Std C++ headers
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// custom headers
#include <jrc18sia_motion_planner/jrc18sia_motion_planner.h>


using namespace std;


// Main program
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinova_arm_test");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	JRCMotionPlanner    motion_planner(nh);


	std::cout << "Adjust kinove arm orientation to verify the dirver works normal; " << std::endl;


	// Initial pose
	double x = 0;
	double y = 0;
	double z = 0;
	double R = 85;
	double P = 5;
	double Y = 5;
	motion_planner.cartesionPathPlanner(x, y, z, R, P, Y); // Adjust orientation

	ros::shutdown();
	return 0;
}

