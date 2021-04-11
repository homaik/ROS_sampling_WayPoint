#include <ros/ros.h>

//Header files
#include <HM_Rviz_path_planning/HM_visualization.h>

//Message files
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>

//Action
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <stdlib.h>

//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

#define MARKERMAXNUM 10

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int nMsg_MarkerArray = 0;


void pointArrayCallback(const visualization_msgs::MarkerArrayPtr &_msg)
{

	if (_msg->markers.size() > nMsg_MarkerArray)
	{
		//기존보다 추가되었으니 시작
		MoveBaseClient ac("move_base", true);
		for (int i = nMsg_MarkerArray; i < _msg->markers.size(); i++)
		{

			ROS_INFO("pointArrayCallback : %d'th Received Message", i);

			//wait for the action server to come up
			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}
			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = _msg->markers[i].pose.position.x;
			goal.target_pose.pose.position.y = _msg->markers[i].pose.position.y;
			goal.target_pose.pose.position.z = _msg->markers[i].pose.position.z;

			goal.target_pose.pose.orientation.x = _msg->markers[i].pose.orientation.x;
			goal.target_pose.pose.orientation.y = _msg->markers[i].pose.orientation.y;
			goal.target_pose.pose.orientation.z = _msg->markers[i].pose.orientation.z;

			goal.target_pose.pose.orientation.w = _msg->markers[i].pose.orientation.w;

			ROS_INFO("Sending  %d'th goal...", i);

			ac.sendGoal(goal);

			while (!ac.waitForResult(ros::Duration(2.0)))
			{
				if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
					ROS_INFO("Processing..");
			}

			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("GOOD");
			else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
				ROS_INFO("LOST");
		}

		nMsg_MarkerArray = _msg->markers.size();
	}
	else
	{
		//아닐때
		ROS_INFO("pointArrayCallback : Sry....");
	}
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "Waypoint_nav");

	ROS_INFO("HI");
	ROS_INFO("HI : %d", TEST);

	ros::NodeHandle nh;
	//std::string MAP_FILENAME = "/home/turtle/HPC1.yaml";

	ros::Subscriber pointArray_BYRVIZ = nh.subscribe("/PointArray", 5, pointArrayCallback);
	
 try{
	while(nh.ok()){

		//메시지 받아서 action에다가 쏘기

		//F.publish_messages();

		//pointArray_BYRVIZ.

		ros::spinOnce();
	}
    }
catch (std::runtime_error& e){
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
 
}
