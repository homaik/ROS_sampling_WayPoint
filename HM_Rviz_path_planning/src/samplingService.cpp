/*
HPC_LAB Sampling Service
	Sample the only 4 way Point(distance = maybe 1.0m)
*/

//ros basic file
#include <ros/ros.h>
#include <iostream>
//#include <nav_msgs/GetPlan.h>

//Visualization of Rviz header file
//#include <HM_Rviz_path_planning/HM_MapRviz.h>
#include <HM_WayPoint/HM_waypoint_server.h>


//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

bool samplingWaypoint(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res){


	ROS_INFO("Start Sampling WayPoint Function!!!");

	WPS::WP_nav W(0);

	W.receiveWaypointforMovebase_ver2(req, res);

	
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "sampling_server");

	ROS_INFO("Server");

	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("samplingWaypoint", samplingWaypoint);
	ROS_INFO("Ready to sampling WayPoint");

	ros::spin();

	return 0;

}