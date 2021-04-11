#pragma once
#include <ros/ros.h>
#include <HM_Rviz_path_planning/HM_visualization.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <stdlib.h>

#include <math.h>
#include <tf/tf.h>

//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

class HMMapRviz
{
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

	//Set goal points on Rviz
	ros::Subscriber way_pose_subscriber;

	//Set goal points on Rviz
	ros::Subscriber ways_pose_subscriber;

	//Sub MapServer
	ros::Subscriber MapSub;

	//Push Robot Marker to Rviz
	ros::Publisher robot_model_publisher;
	ros::Publisher robot_goal_publisher;
	ros::Publisher robot_point_publisher;
	ros::Publisher robot_pointArray_publisher;

	//TODO Map_data
	nav_msgs::OccupancyGridPtr msg_map;

	//Robot Model Message
	visualization_msgs::MarkerPtr msg_robot_model;

	//Robot Model Message
	visualization_msgs::MarkerPtr msg_goal_robot_model;

	//Robot Model Message
	visualization_msgs::MarkerPtr msg_point_robot_model;

	visualization_msgs::MarkerArrayPtr msg_point_Array;

	//Robot
	//Circle robot_model;
	HM_rviz_Marker M = HM_rviz_Marker(1.0, 1.0);
	HM_rviz_Marker G = HM_rviz_Marker(1.0, 1.0);
	HM_rviz_Marker P = HM_rviz_Marker(1.0, 1.0);

	int ArrayNum = 0;
	int arrayPubNum = 0;

#define MARKERMAXNUM 10

	HM_rviz_Marker test[MARKERMAXNUM] =
	{ HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0),
	 HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0) };

public:
	explicit HMMapRviz(const std::string _map_filename)
	{
		ROS_INFO("Loading File....");

		//subscriber
		robot_pose_subscriber = nh.subscribe("/initialpose", 1, &HMMapRviz::Setinitialpose, this);

		goal_pose_subscriber = nh.subscribe("/move_base_simple/goalfake", 1, &HMMapRviz::SetGoalpose, this);

		point_pose_subscriber = nh.subscribe("/clicked_point", 1, &HMMapRviz::SetpointArraypose, this);

		way_pose_subscriber = nh.subscribe("/move_base_simple/goals", 5, &HMMapRviz::SetpointArraypose_ver2, this);

		//SetpointArrayposes_ver3
		ways_pose_subscriber = nh.subscribe("/sampling/goals", 5, &HMMapRviz::SetpointArrayposes_ver3, this);


		//publisher
		robot_model_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/robot_model", 1);

		robot_point_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/Point", 1);

		robot_goal_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/robot_model2", 1);

		robot_pointArray_publisher = nh.advertise<visualization_msgs::MarkerArray>("/PointArray", 1);
	}

	void Setinitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg);

	void SetGoalpose(const geometry_msgs::PoseStamped::ConstPtr& _msg);

	void SetpointArraypose(const geometry_msgs::PointStamped::ConstPtr& _msg);

	void SetpointArraypose_ver2(const geometry_msgs::PoseStamped::ConstPtr& _msg);

	void SetpointArrayposes_ver3(const nav_msgs::Path::ConstPtr& _msg);

	void publish_messages()
	{
		if (msg_robot_model != NULL && robot_model_publisher.getNumSubscribers() > 0)
		{
			//ROS_INFO("publish : position x = %f, position y = %f", msg_robot_model->pose.position.x, msg_robot_model->pose.position.y);
			robot_model_publisher.publish(msg_robot_model);
		}

		if (msg_goal_robot_model != NULL && robot_goal_publisher.getNumSubscribers() > 0)
		{
			//ROS_INFO("publish : position x = %f, position y = %f", msg_goal_robot_model->pose.position.x, msg_goal_robot_model->pose.position.y);
			robot_goal_publisher.publish(msg_goal_robot_model);
		}

		if (msg_point_Array != NULL && robot_pointArray_publisher.getNumSubscribers() > 0 && arrayPubNum <= ArrayNum)
		{
			//ROS_INFO("publish : position x = %f, position y = %f", msg_point_robot_model->pose.position.x, msg_goal_robot_model->pose.position.y);
			//ROS_INFO("PUB");
			robot_pointArray_publisher.publish(msg_point_Array);
			arrayPubNum++;
		}
	}
};





void HMMapRviz::SetpointArraypose_ver2(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
	ArrayNum++;

	double q_x, q_y, q_z, q_w;
	q_x = _msg->pose.orientation.x;
	q_y = _msg->pose.orientation.y;
	q_z = _msg->pose.orientation.z;
	q_w = _msg->pose.orientation.w;

	tf::Quaternion q(q_x, q_y, q_z, q_w);

	tf::Quaternion q_i(q.inverse());

	tf::Matrix3x3 m(q);

	//tf::Matrix3x3 n()

	double roll, pitch, yaw;

	m.getRPY(roll, pitch, yaw);

	if (yaw <= 0) yaw = 3.14 + yaw;
	else yaw -= 3.14;

	tf::Quaternion new_q(tf::createQuaternionFromRPY(roll, pitch, yaw));

	ROS_INFO("ADD 2D_Nav_Goal %d : position x = %f, position y = %f", this->ArrayNum, (float)_msg->pose.position.x, (float)_msg->pose.position.y);
	ROS_INFO("this 2D_Nav_Goal : orientation x = %f, y = %f, z = %f, w = %f", (float)_msg->pose.orientation.x, (float)_msg->pose.orientation.y, (float)_msg->pose.orientation.z, (float)_msg->pose.orientation.w);

	ROS_INFO("RPY: %f, %f, %f", (float)roll, (float)pitch, (float)yaw);
	ROS_INFO("Inverse orientation x = %f, y = %f, z = %f, w = %f", (float)q_i.getX(), (float)q_i.getY(), (float)q_i.getZ(), (float)q_i.getW());

	ROS_INFO("new orientation x = %f, y = %f, z = %f, w = %f", (float)new_q.getX(), (float)new_q.getY(), (float)new_q.getZ(), (float)new_q.getW());


	this->test[ArrayNum].set_Position((double)_msg->pose.position.x, (double)_msg->pose.position.y);

	this->test[ArrayNum].set_Color(0.0, 0.0, 1.0 - (this->ArrayNum * 0.2), 1.0);
	test[ArrayNum].set_ID(ArrayNum);

	this->test[ArrayNum].set_Orientation(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);

	visualization_msgs::MarkerArrayPtr message(new visualization_msgs::MarkerArray);

	message->markers.resize(ArrayNum + 1);

	for (int i = 0; i <= this->ArrayNum; i++)
	{
		message->markers[i] = *(test[i].msg_robot_model);
		//ROS_INFO("MESSAGE Point %d : position x = %f, position y = %f", i, message->markers[i].pose.position.x, message->markers[i].pose.position.y);
	}

	//msg_point_Array->markers.push_back((test[ArrayNum].msg_robot_model));

	this->ArrayNum++;

	this->msg_point_Array = message;
}


void HMMapRviz::SetpointArrayposes_ver3(const nav_msgs::Path::ConstPtr& _msg)
{
	//ArrayNum = 0;

	ROS_INFO("Welcome new Plan!!");


	for (int i = 0; i < _msg->poses.size(); i++) {

		ROS_INFO("ADD 2D_Nav_Goal %d : position x = %f, position y = %f", i, (float)_msg->poses[i].pose.position.x, (float)_msg->poses[i].pose.position.y);
		//ROS_INFO("this 2D_Nav_Goal : orientation x = %f, y = %f, z = %f, w = %f", (float)_msg->pose.orientation.x, (float)_msg->pose.orientation.y, (float)_msg->pose.orientation.z, (float)_msg->pose.orientation.w);

		this->test[i].set_Position((double)_msg->poses[i].pose.position.x, (double)_msg->poses[i].pose.position.y);

		this->test[i].set_Color(0.0, 0.0, 1.0 - (i * 0.2), 1.0);
		test[i].set_ID(i);

		this->test[i].set_Orientation(_msg->poses[i].pose.orientation.x, _msg->poses[i].pose.orientation.y, _msg->poses[i].pose.orientation.z, _msg->poses[i].pose.orientation.w);

	}


	visualization_msgs::MarkerArrayPtr message(new visualization_msgs::MarkerArray);

	message->markers.resize(_msg->poses.size() + 1);


	for (int i = 0; i < _msg->poses.size(); i++)
	{
		message->markers[i] = *(test[i].msg_robot_model);
	}

	this->msg_point_Array = message;
	ArrayNum++;
}





void HMMapRviz::SetGoalpose(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{

	ROS_INFO("Goal : position x = %lf, position y = %lf", _msg->pose.position.x, _msg->pose.position.y);
	ROS_INFO("Orientation : x = %lf, y = %lf, z = %lf, w = %lf", _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);

	this->G.set_Position(_msg->pose.position.x, _msg->pose.position.y);
	this->G.set_Color(1.0, 0.0, 0.0, 1.0);

	this->msg_goal_robot_model = G.msg_robot_model;
}

void HMMapRviz::SetpointArraypose(const geometry_msgs::PointStamped::ConstPtr& _msg)
{

	ROS_INFO("ADD Point %d : position x = %f, position y = %f", ArrayNum, _msg->point.x, _msg->point.y);

	this->test[this->ArrayNum].set_Position(_msg->point.x, _msg->point.y);
	this->test[this->ArrayNum].set_Color(0.0, 0.0, 1.0 - (this->ArrayNum * 0.2), 1.0);
	this->test[this->ArrayNum].set_ID(this->ArrayNum);

	this->test[0].set_Orientation(0.0, 0.0, -0.730931, 0.682451);

	this->test[1].set_Orientation(0.0, 0.0, 0.999777, 0.021124);
	this->test[2].set_Orientation(0.0, 0.0, -0.730931, 0.682451);

	this->test[3].set_Orientation(0.0, 0.0, -0.010465, 0.999945);

	this->test[4].set_Orientation(0.0, 0.0, 0.743587, 0.668639);
	this->test[5].set_Orientation(0.0, 0.0, 0.999867, 0.016305);
	this->test[6].set_Orientation(0.0, 0.0, -0.030628, 0.999531);

	//Right
	//test[7].set_Orientation(0.0, 0.0, -0.715280, 0.698838);

	//test[8].set_Orientation(0.0, 0.0, 0.0, 1.0);

	visualization_msgs::MarkerArrayPtr message(new visualization_msgs::MarkerArray);

	message->markers.resize(ArrayNum + 1);

	for (int i = 0; i <= ArrayNum; i++)
	{
		message->markers[i] = *(test[i].msg_robot_model);
		ROS_INFO("MESSAGE Point %d : position x = %f, position y = %f", i, message->markers[i].pose.position.x, message->markers[i].pose.position.y);
	}

	//msg_point_Array->markers.push_back((test[ArrayNum].msg_robot_model));

	this->ArrayNum++;

	this->msg_point_Array = message;
}

void HMMapRviz::Setinitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg)
{

	ROS_INFO("position x = %f, position y = %f", _msg->pose.pose.position.x, _msg->pose.pose.position.y);

	ROS_INFO("Orientation : x = %lf, y = %lf, z = %lf, w = %lf", _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z, _msg->pose.pose.orientation.w);

	this->M.set_Position(_msg->pose.pose.position.x, _msg->pose.pose.position.y);

	this->msg_robot_model = this->M.msg_robot_model;
}


void quaternion_to_euler_angle(double* q_x, double* q_y, double* q_z, double* q_w) {
	double t0 = (double)2.0 * ((*q_w) * (*q_x) + (*q_y) * (*q_z));
}