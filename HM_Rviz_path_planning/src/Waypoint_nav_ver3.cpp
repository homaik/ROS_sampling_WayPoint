#include <ros/ros.h>

//Header files
//#include <HM_Rviz_path_planning/HM_visualization.h>
#include <HM_WayPoint/HM_waypoint_server.h>

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


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int nMsg_MarkerArray = 0;



char choose(bool existWayPoint);


int main(int argc, char **argv)
{

	ros::init(argc, argv, "Waypoint_nav_ver2");

	ROS_INFO("HI");
	ROS_INFO("HI : %d", TEST);

	ros::NodeHandle nh;
	//std::string MAP_FILENAME = "/home/turtle/HPC1.yaml";

	//ros::Subscriber pointArray_BYRVIZ = nh.subscribe("/PointArray", 5, pointArrayCallback);
	
	//ros::Subscriber receivedPointArray_BYRVIZ;

	WPS::WP_nav W(3);

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
			std::cin >> W.nWay;

			//구독 시작
			//nWay만큼 입력받고 구독 끄기 + 다음껄로 넘어가기(path publish) + bool

			W.start_sub_receivedPointArray();

			std::cout << "Start to pick way point" << std::endl;

			//waiting....
			while(W.Current_nWay < W.nWay){
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
			std::cout << "Congratulation!!" << std::endl;

			//TODO
			//Clear All WayPoint
			//Restart Pick WP
		}
		
		if(existWayPoint && choice == 'j'){
			//Just
			W.sendPoint_forJustNavigation();
			//TODO
			//Clear All WayPoint
			//Restart Pick WP
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
