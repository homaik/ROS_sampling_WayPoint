/*
HPC_LAB visualization Node in RVIZ
visualize way_Point and initial Pose and simple goals in Rviz

*/

//ros basic file
#include <ros/ros.h>

//Visualization of Rviz header file
//#include <HM_Rviz_path_planning/HM_MapRviz.h>
#include <nav_msgs/GetPlan.h>
#include <HM_WayPoint/HM_waypoint_server.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

//#include "yaml-cpp/yaml.h"

//const double VIS_HEIGHT_MARKER  = 0.01;

class samplingAction
{
protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as;

	move_base_msgs::MoveBaseActionFeedback feedback_;
	move_base_msgs::MoveBaseActionResult result_;
	std::string action_name_;

public:
	samplingAction(std::string name) : as(nh, name, boost::bind(&samplingAction::executeCB, this, _1), false), action_name_(name)
	{
		//this->as(nh, "", boost::bind())

		as.start();
	}

	void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
	{

		ROS_INFO("Received sampling Action!!");
		ros::Rate r(2);
		bool success = true;

		//도착지점을 받으면, Service 실행하면서 Path 받기
		//feedback으로 어디 위치에 있는지 말하기
		//feedback_.feedback.base_position
		//this->as.publishFeedback

		bool arrived = false;
		ros::ServiceClient c = nh.serviceClient<nav_msgs::GetPlan>("samplingWaypoint");
		WPS::WP_nav W(0);

		nav_msgs::GetPlan temp_path;
		WPS::MoveBaseClient ac("move_base", true);

		W.getAMCLpose();


		temp_path.request.start.pose = W.receivedAmclPose.pose;
		temp_path.request.start.header = W.receivedAmclPose.header;

		temp_path.request.goal.header = goal->target_pose.header;
		temp_path.request.goal.pose = goal->target_pose.pose;

		//Sampling Distance 0.5 meter
		temp_path.request.tolerance = 0.5;

		//
		while (!arrived)
		{

			c.call(temp_path);

			ROS_INFO("Action: Received Service");

			ros::Publisher temp_publisher = nh.advertise<nav_msgs::Path>("/sampling/goals", 1);

			bool publish = false;

			ROS_INFO("Action : Start to Push the Rviz!!");

			//visualization in Rviz
			while (!publish)
			{
				if (temp_publisher.getNumSubscribers() > 0)
				{
					temp_publisher.publish(temp_path.response.plan);
					ROS_INFO("new Plan Push!!");

					ros::Duration(1.0);
					publish = true;
				}
			}

			//go to wayPoint in virtual world
			//(will be changed to push topic to unmanned Car)

			ROS_INFO("Start to Push WayPoint");
			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			WPS::WP_nav Temp(0);
			Temp.getAMCLpose();


			for (int i = 0; i < temp_path.response.plan.poses.size(); i++)
			{
				
				if (as.isNewGoalAvailable())
				{
					ROS_INFO("Action: NewGoal Available");
					//break;
				}

				

				ROS_INFO("Current Pose : %lf, %lf", (float) Temp.receivedAmclPose.pose.position.x, (float) Temp.receivedAmclPose.pose.position.y);

				ROS_INFO("Distance : %lf", (float) Temp.caculateDistance(Temp.receivedAmclPose, temp_path.request.goal));
				
				ROS_INFO("Start to Push %dth WayPoint", i);

				move_base_msgs::MoveBaseGoal goal;

				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();

				goal.target_pose.pose.position = temp_path.response.plan.poses[i].pose.position;
				goal.target_pose.pose.orientation = temp_path.response.plan.poses[i].pose.orientation;
				ac.sendGoal(goal);

				while (!ac.waitForResult(ros::Duration(2.0)))
				{
					if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
						ROS_INFO("Processing..");

					/*
					if (as.isPreemptRequested() || !ros::ok())

				move_base_msgs::MoveBaseGoal goal;

				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();

				goal.target_pose.pose.position = temp_path.response.plan.poses[i].pose.position;
				goal.target_pose.pose.orientation = temp_path.response.plan.poses[i].pose.orientation;
				ac.sendGoal(goal)
					*/
				}

				if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					ROS_INFO("GOOD");
				else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
					ROS_INFO("LOST");
			}

			

			ROS_INFO("Finish and Current Pose : %lf, %lf", (float) Temp.receivedAmclPose.pose.position.x, (float) Temp.receivedAmclPose.pose.position.y);

			
			if (Temp.caculateDistance(Temp.receivedAmclPose, temp_path.request.goal) <= (temp_path.request.tolerance / 2))
			{
				ROS_INFO("Reached the goal almostly");
				arrived = true;
			}
			

			ROS_INFO("Received the Start Position Again");

			temp_path.request.start.pose = Temp.receivedAmclPose.pose;
			temp_path.request.start.header = Temp.receivedAmclPose.header;
		}

		if (success)
		{
			ROS_INFO("Finished Action");
			this->as.setSucceeded();
		}
	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "samplingAction");

	ROS_INFO("I'm Action");

	samplingAction A("samplingWayPoint");

	ros::spin();
}