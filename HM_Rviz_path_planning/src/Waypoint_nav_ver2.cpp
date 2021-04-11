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

#include <tf/tf.h>
//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

#define MARKERMAXNUM 10

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int nMsg_MarkerArray = 0;

int nWay = 0;
int Current_nWay = 0;


char choose(bool existWayPoint);

class WP_nav{
	//Make the handle
	ros::NodeHandle nh;

	//receivedPoint
	ros::Subscriber receivedPointArray_BYRVIZ;

	//MoveBaseClient
	//MoveBaseClient ac;
	
	//MoveBaseClient ac("move_base", true);
	HM_rviz_Marker test[MARKERMAXNUM] =
	{HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0),
	 HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0)};


public:
	WP_nav(int a){
		ROS_INFO("Make_WP_nav");
	}

	void start_sub_receivedPointArray(){
		receivedPointArray_BYRVIZ = nh.subscribe("/PointArray", 5, &WP_nav::receivedPointArray_Callback, this);
	}

	void finish_sub_receivedPointArray(){
		receivedPointArray_BYRVIZ.shutdown();
	}

	void receivedPointArray_Callback(const visualization_msgs::MarkerArrayPtr &_msg){
		ROS_INFO("Received /PointArray");
		ROS_INFO("The number of MarkerArray is %d", (int) _msg->markers.size());
		int arrayNum = _msg->markers.size() - 1;

		test[arrayNum].set_Position(_msg->markers[arrayNum].pose.position.x, _msg->markers[arrayNum].pose.position.y);

		test[arrayNum].set_Orientation(_msg->markers[arrayNum].pose.orientation.x, _msg->markers[arrayNum].pose.orientation.y,
											_msg->markers[arrayNum].pose.orientation.z, _msg->markers[arrayNum].pose.orientation.w);


		Current_nWay = _msg->markers.size();
	}

	void sendPoint_forJustNavigation(){
		MoveBaseClient ac("move_base", true);

		ROS_INFO("Start send Point for Just Navigation, total %d waypoint", nWay);

		for(int i = 0; i < nWay; i++){
			ROS_INFO("GoTo : %d'th Waypoint", i);

			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = test[i].msg_robot_model->pose.position.x;


			goal.target_pose.pose.position.y = test[i].msg_robot_model->pose.position.y;
			goal.target_pose.pose.position.z = test[i].msg_robot_model->pose.position.z;

			goal.target_pose.pose.orientation.x = test[i].msg_robot_model->pose.orientation.x;
			goal.target_pose.pose.orientation.y = test[i].msg_robot_model->pose.orientation.y;
			goal.target_pose.pose.orientation.z = test[i].msg_robot_model->pose.orientation.z;

			goal.target_pose.pose.orientation.w = test[i].msg_robot_model->pose.orientation.w;

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

		ac.~SimpleActionClient();

	}

	void sendPoint_forCycleNavigation(){
		MoveBaseClient ac("move_base", true);

		ROS_INFO("Start send Point for Cycle Navigation, total %d waypoint", nWay);

		for(int i = 0; i < nWay; i++){
			ROS_INFO("GoTo : %d'th Waypoint", i);

			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = test[i].msg_robot_model->pose.position.x;


			goal.target_pose.pose.position.y = test[i].msg_robot_model->pose.position.y;
			goal.target_pose.pose.position.z = test[i].msg_robot_model->pose.position.z;

			goal.target_pose.pose.orientation.x = test[i].msg_robot_model->pose.orientation.x;
			goal.target_pose.pose.orientation.y = test[i].msg_robot_model->pose.orientation.y;
			goal.target_pose.pose.orientation.z = test[i].msg_robot_model->pose.orientation.z;

			goal.target_pose.pose.orientation.w = test[i].msg_robot_model->pose.orientation.w;

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

		//방향을 반대로 하는 경우를 생각해서 코딩을 해야함....


		for(int i = nWay-1; i >= 0; i--){
			ROS_INFO("GoTo : Reverse %d'th Waypoint", i);

			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = test[i].msg_robot_model->pose.position.x;


			goal.target_pose.pose.position.y = test[i].msg_robot_model->pose.position.y;
			goal.target_pose.pose.position.z = test[i].msg_robot_model->pose.position.z;

			tf::Quaternion q(test[i].msg_robot_model->pose.orientation.x, test[i].msg_robot_model->pose.orientation.y, 
						test[i].msg_robot_model->pose.orientation.z, test[i].msg_robot_model->pose.orientation.w);

			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			if(yaw <= 0) yaw = 3.14 + yaw;
			else yaw -= 3.14;

			tf::Quaternion new_q(tf::createQuaternionFromRPY(roll, pitch, yaw));


			goal.target_pose.pose.orientation.x = new_q.getX();
			goal.target_pose.pose.orientation.y = new_q.getY();
			goal.target_pose.pose.orientation.z = new_q.getZ();

			goal.target_pose.pose.orientation.w = new_q.getW();

			ROS_INFO("Sending  Reverse %d'th goal...", i);

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


		ac.~SimpleActionClient();

	}

};




int main(int argc, char **argv)
{

	ros::init(argc, argv, "Waypoint_nav_ver2");

	ROS_INFO("HI");
	ROS_INFO("HI : %d", TEST);

	ros::NodeHandle nh;
	//std::string MAP_FILENAME = "/home/turtle/HPC1.yaml";

	//ros::Subscriber pointArray_BYRVIZ = nh.subscribe("/PointArray", 5, pointArrayCallback);
	
	//ros::Subscriber receivedPointArray_BYRVIZ;

	WP_nav W(3);

	char choice = 'q';
	bool existWayPoint = false;


	do{

		choice = choose(existWayPoint);

		if(choice == 'q'){
			std::cout << "Quit..." << std::endl;
		}

		//Exception
		if(!existWayPoint){
			if(choice != 'w'){
				std::cout << "Plz.. Right parameter.." << std::endl;
				return 0;
			}
		}
		else{
			if(choice != 'c' && choice != 'j'){
				std::cout << "Plz.. Right parameter.." << std::endl;
				return 0;
			}
		}

		if(!existWayPoint && choice == 'w'){
			//Pick the waypoint
			std::cout << "How many do you pick for way_point within 10?" << std::endl;
			std::cin >> nWay;

			//구독 시작
			//nWay만큼 입력받고 구독 끄기 + 다음껄로 넘어가기(path publish) + bool

			W.start_sub_receivedPointArray();

			std::cout << "Start to pick way point" << std::endl;

			//waiting....
			while(Current_nWay < nWay){
				//std::cout << "Listen the WayPoint : " << Current_nWay <<  " th" <<std::endl;
				//ros::Duration(2.0);
				
				//while (clock() - start < delay);
				//start = clock();
				ros::spinOnce();
			}

			//shutdown sub
			W.finish_sub_receivedPointArray();
			//receivedPointArray_BYRVIZ.shutdown();

			existWayPoint = true;
			//
		}

		if(existWayPoint && choice == 'c'){
			//Cycle
			W.sendPoint_forCycleNavigation();
		}
		
		if(existWayPoint && choice == 'j'){
			//Just
			W.sendPoint_forJustNavigation();
		}

	}while(choice != 'q');


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
