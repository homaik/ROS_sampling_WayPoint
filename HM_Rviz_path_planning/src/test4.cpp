/*
HPC_LAB visualization Node in RVIZ
visualize way_Point and initial Pose and simple goals in Rviz

*/

//ros basic file
#include <ros/ros.h>

//Visualization of Rviz header file
#include <HM_Rviz_path_planning/HM_MapRviz.h>


//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "test4_node");

	ROS_INFO("HI");
	ROS_INFO("HI : %d", TEST);

	ros::NodeHandle nh;
	//std::string MAP_FILENAME = "/home/turtle/HPC1.yaml";

	HMMapRviz F(MAP_FILENAME);

	//ros::Subscriber MapSub = nh.subscribe("map", 10, )

	//ros::spin();

	//nav_msg::OccupancyGridPTR
	try
	{
		while (nh.ok())
		{
			F.publish_messages();
			ros::spinOnce();
		}
	}
	catch (std::runtime_error &e)
	{
		ROS_ERROR("Exception: %s", e.what());
		return -1;
	}
}