/*
HPC_LAB 
Run Node of waypoint server

It must run after run ROS navigation node
It uses Ros navigation node

*/

//basic ros header file
#include <ros/ros.h>

//Header files
//#include <HM_Rviz_path_planning/HM_visualization.h>
#include <HM_WayPoint/HM_waypoint_server.h>

//Message header file(it have to write related dependency file name in CMakeList and package)
#include <nav_msgs/OccupancyGrid.h>
//For visualization of shape in Rviz
#include <visualization_msgs/MarkerArray.h>
//For location with position, orientation, Covariance
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//For location(position, orientation)
#include <geometry_msgs/PoseStamped.h>
//For visualization of Color in Rviz
#include <std_msgs/ColorRGBA.h>
//For location(position)
#include <geometry_msgs/PointStamped.h>
//For move_base_msgs
#include <move_base_msgs/MoveBaseAction.h>



//Action header files
#include <actionlib/client/simple_action_client.h>


#include <tf/tf.h>


#include <iostream>
#include <stdlib.h>


//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int nMsg_MarkerArray = 0;

//Select Function
char choose(bool existWayPoint);


int main(int argc, char **argv)
{

	ros::init(argc, argv, "Waypoint_nav_ver2");

	ROS_INFO("HI");
	//ROS_INFO("HI : %d", TEST);

	ros::NodeHandle nh;

	WPS::WP_nav W(3);

	char choice = 'q';
	bool existWayPoint = false;

	
	//Start receiveWaypoint for Movebase
	W.receiveWaypointforMovebase_ver2();


	return 0;
}



char choose(bool existWayPoint)
{
	char choice = 'q';

	if (!existWayPoint)
	{
		std::cout << "|-------------------------------|" << std::endl;
		std::cout << "|PRESSE A KEY:" << std::endl;
		std::cout << "|'w': Start to pick WayPoint within 10" << std::endl;
		std::cout << "|'q': Quit " << std::endl;
		std::cout << "|-------------------------------|" << std::endl;
	}
	else
	{
		std::cout << "|-------------------------------|" << std::endl;
		std::cout << "|PRESSE A KEY:" << std::endl;
		std::cout << "|'c': Go navigation of Cycle path" << std::endl;
		std::cout << "|'j': Go navigation of Just path" << std::endl;
		std::cout << "|'q': Quit " << std::endl;
		std::cout << "|-------------------------------|" << std::endl;
	}

	std::cin >> choice;

	return choice;
}
