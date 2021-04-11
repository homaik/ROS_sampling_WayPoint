/*
HPC_LAB visualization Node in RVIZ
visualize way_Point and initial Pose and simple goals in Rviz

*/

//ros basic file
#include <ros/ros.h>

//Visualization of Rviz header file
//#include <HM_Rviz_path_planning/HM_MapRviz.h>
#include <HM_WayPoint/HM_waypoint_server.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>

//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void receivedfromRVIZ_Callback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{

	ROS_INFO("Received /movebase_fake");
	MoveBaseClient MC("samplingWayPoint", true);

	WPS::WP_nav W(0);

	while (!MC.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	//ros::Rate RRR(10000);

	//여기다가 이제 목표 지점 받는 Node

	//if(MC.)
	//MC.cancelGoal();

	ROS_INFO("Pick the Goal position");

	//W.getAMCL_Goalpose();

	move_base_msgs::MoveBaseGoal temp_goal;

	temp_goal.target_pose.header = _msg->header;
	temp_goal.target_pose.pose = _msg->pose;

	MC.sendGoal(temp_goal);

	if (MC.getState() == actionlib::SimpleClientGoalState::ACTIVE)
		ROS_INFO("~ing");

	if (MC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Complete!! + and start to receive wayPoint");

	MC.sendGoal(temp_goal);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "samplingAction_client");

	ros::NodeHandle nh;

	ROS_INFO("I'm Action Client");

	

	ros::Subscriber goal_fromRviz = nh.subscribe("/move_base_simple/goalfake", 3, &receivedfromRVIZ_Callback);

	//MoveBaseClient ac("move_base", true);

	ros::spin();
}