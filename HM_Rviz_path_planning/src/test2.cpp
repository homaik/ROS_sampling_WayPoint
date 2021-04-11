#include <ros/ros.h>
#include <HM_Rviz_path_planning/HM_visualization.h>


#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <stdlib.h>



//#include "yaml-cpp/yaml.h"


//const double VIS_HEIGHT_MARKER  = 0.01;





class HMMapRviz{
protected:
 //const double ROBOT_RADIUS = 0.1;
 //const double robot_size = ROBOT_RADIUS * 2.0;
 ros::NodeHandle nh;

 //Initial pose on Rviz
 ros::Subscriber robot_pose_subscriber;

 //Set goal on Rviz
 ros::Subscriber goal_pose_subscriber;

 //Set point on Rviz
 ros::Subscriber point_pose_subscriber;

 //Sub MapServer
 ros::Subscriber MapSub;

 //Push Robot Marker to Rviz
 ros::Publisher  robot_model_publisher;
 ros::Publisher  robot_goal_publisher;
 ros::Publisher  robot_point_publisher;

 //TODO Map_data
 nav_msgs::OccupancyGridPtr  msg_map;

 //Robot Model Message
 visualization_msgs::MarkerPtr   msg_robot_model;

 //Robot Model Message
 visualization_msgs::MarkerPtr   msg_goal_robot_model;

  //Robot Model Message
 visualization_msgs::MarkerPtr   msg_point_robot_model;

 visualization_msgs::MarkerArrayPtr msg_point_Array;

 //Robot
 //Circle robot_model;
 HM_rviz_Marker M = HM_rviz_Marker(1.0, 1.0);
 HM_rviz_Marker G = HM_rviz_Marker(1.0, 1.0);
 HM_rviz_Marker P = HM_rviz_Marker(1.0, 1.0);

#define MARKERMAXNUM 5

 HM_rviz_Marker test[MARKERMAXNUM] = 
	{HM_rviz_Marker(1.0,1.0), HM_rviz_Marker(1.0,1.0), HM_rviz_Marker(1.0,1.0), HM_rviz_Marker(1.0,1.0), HM_rviz_Marker(1.0,1.0)};
	



public:
 	explicit HMMapRviz(const std::string& _map_filename){
		ROS_INFO("Loading File....");
		robot_pose_subscriber = nh.subscribe("/initialpose", 1, &HMMapRviz::Setinitialpose, this);

		goal_pose_subscriber = nh.subscribe("/move_base_simple/goal", 1, &HMMapRviz::SetGoalpose, this);

		point_pose_subscriber = nh.subscribe("/clicked_point", 1, &HMMapRviz::Setpointpose, this);

		robot_model_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/robot_model", 1);

		robot_point_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/Point", 1); 

		robot_goal_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/robot_model2", 1);
	
	
	}
	

	void Setinitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg){

		ROS_INFO("position x = %f, position y = %f", _msg->pose.pose.position.x, _msg->pose.pose.position.y);
		
		//M = HM_rviz_Marker(_msg->pose.pose.position.x, _msg->pose.pose.position.y);
		
		M.set_Position(_msg->pose.pose.position.x, _msg->pose.pose.position.y);
		
		/*
		std_msgs::ColorRGBA color;
		color.r = 0.0;  color.g = 1.0;  color.b = 0.0;  color.a = 1.0;

		visualization_msgs::MarkerPtr MarkerM(new visualization_msgs::Marker);
	
		MarkerM->header.frame_id = "map";
        	MarkerM->header.stamp = ros::Time::now();

		MarkerM->id = 0;
        	MarkerM->type = visualization_msgs::Marker::CYLINDER;
        	MarkerM->action = visualization_msgs::Marker::ADD;

		MarkerM->scale.x = MarkerM->scale.y = robot_size* 0.5f * 2.0;
        	MarkerM->scale.z = VIS_HEIGHT_MARKER;

		MarkerM->color = color;

		MarkerM->pose.position.x = _msg->pose.pose.position.x; MarkerM->pose.position.y = _msg->pose.pose.position.y; MarkerM->pose.position.z = VIS_HEIGHT_MARKER * 0.5;
		MarkerM->pose.orientation.x = 0.0;   MarkerM->pose.orientation.y = 0.0;   MarkerM->pose.orientation.z = 0.0;   MarkerM->pose.orientation.w = 1.0;
		*/


		//msg_robot_model = MarkerM;
		
		msg_robot_model = M.msg_robot_model;
	}



	void SetGoalpose(const geometry_msgs::PoseStamped::ConstPtr& _msg){

		ROS_INFO("Goal : position x = %f, position y = %f", _msg->pose.position.x, _msg->pose.position.y);
		
		
		G.set_Position(_msg->pose.position.x, _msg->pose.position.y);
		G.set_Color(1.0, 0.0, 0.0, 1.0);


		
		msg_goal_robot_model = G.msg_robot_model;
	}


	void Setpointpose(const geometry_msgs::PointStamped::ConstPtr& _msg){

	
		ROS_INFO("Point : position x = %f, position y = %f", _msg->point.x, _msg->point.y);
			
			
		P.set_Position(_msg->point.x, _msg->point.y);
		P.set_Color(0.0, 0.0, 1.0, 1.0);


			
		msg_point_robot_model = P.msg_robot_model;
	}



	void publish_messages(){
		if(msg_robot_model != NULL && robot_model_publisher.getNumSubscribers() > 0){
			//ROS_INFO("publish : position x = %f, position y = %f", msg_robot_model->pose.position.x, msg_robot_model->pose.position.y);
			robot_model_publisher.publish(msg_robot_model);
		}

		if(msg_goal_robot_model != NULL && robot_goal_publisher.getNumSubscribers() > 0){
			//ROS_INFO("publish : position x = %f, position y = %f", msg_goal_robot_model->pose.position.x, msg_goal_robot_model->pose.position.y);
			robot_goal_publisher.publish(msg_goal_robot_model);
		}

		if(msg_point_robot_model != NULL && robot_point_publisher.getNumSubscribers() > 0){
			//ROS_INFO("publish : position x = %f, position y = %f", msg_point_robot_model->pose.position.x, msg_goal_robot_model->pose.position.y);
			robot_point_publisher.publish(msg_point_robot_model);
		}
	}



};


int main(int argc, char** argv){

 ros::init(argc, argv, "test2_node");
 
 ROS_INFO("HI");
 ROS_INFO("HI : %d", TEST);

 ros::NodeHandle nh;
 std::string MAP_FILENAME = "/home/turtle/HPC1.yaml";

 HMMapRviz F(MAP_FILENAME);

 //ros::Subscriber MapSub = nh.subscribe("map", 10, )
 
 //ros::spin();
 
 //nav_msg::OccupancyGridPTR
 try{
	while(nh.ok()){
		F.publish_messages();
		ros::spinOnce();
	}
    }
catch (std::runtime_error& e){
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

}
