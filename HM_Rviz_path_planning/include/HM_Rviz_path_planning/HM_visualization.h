/*
HPC_LAB Marker class in RVIZ


*/


//Message header file(it have to write related dependency file name in CMakeList and package)
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/ColorRGBA.h>

#include <ctime>

//MARKER HEIGHT
const double HEIGHT_MARKER = 0.01;

#define TEST 1

/* declare robot_model_publisher */

class HM_rviz_Marker
{
	//Marker Size
protected:
	const double ROBOT_RADIUS = 0.1;
	const double robot_size = ROBOT_RADIUS * 2.0;

	//Handle to subscribe or publish..
	ros::NodeHandle nh;

	//Push Robot Marker to Rviz
	ros::Publisher robot_model_publisher;

public:
	//Robot Model Message
	visualization_msgs::MarkerPtr msg_robot_model;

	//Initialize
	explicit HM_rviz_Marker(const std::string& _publisherName, double _position_x, double _position_y)
	{
		//ROS_INFO("Hi, MyGoal is x = %lf, y = %lf", _position_x, _position_y);

		robot_model_publisher = nh.advertise<visualization_msgs::Marker>(_publisherName, 1);

		std_msgs::ColorRGBA color;
		color.r = 0.0;
		color.g = 1.0;
		color.b = 0.0;
		color.a = 1.0;

		visualization_msgs::MarkerPtr MarkerM(new visualization_msgs::Marker);

		MarkerM->header.frame_id = "map";
		MarkerM->header.stamp = ros::Time::now();

		MarkerM->id = 0;
		MarkerM->type = visualization_msgs::Marker::CYLINDER;
		MarkerM->action = visualization_msgs::Marker::ADD;

		MarkerM->scale.x = MarkerM->scale.y = robot_size * 0.5f * 2.0;
		MarkerM->scale.z = HEIGHT_MARKER;

		MarkerM->color = color;

		MarkerM->pose.position.x = _position_x;
		MarkerM->pose.position.y = _position_y;
		MarkerM->pose.position.z = HEIGHT_MARKER * 0.5;

		MarkerM->pose.orientation.x = 0.0;
		MarkerM->pose.orientation.y = 0.0;
		MarkerM->pose.orientation.z = 0.0;
		MarkerM->pose.orientation.w = 1.0;

		msg_robot_model = MarkerM;
	}

	HM_rviz_Marker();

	HM_rviz_Marker(double _position_x, double _position_y)
	{
		//ROS_INFO("Hi, HM_rviz_Marker: x = %lf, y = %lf", _position_x, _position_y);

		std_msgs::ColorRGBA color;
		color.r = 0.0;
		color.g = 1.0;
		color.b = 0.0;
		color.a = 1.0;

		visualization_msgs::MarkerPtr MarkerM(new visualization_msgs::Marker);

		MarkerM->header.frame_id = "map";
		MarkerM->header.stamp = ros::Time::now();

		MarkerM->id = 0;
		MarkerM->type = visualization_msgs::Marker::CYLINDER;
		MarkerM->action = visualization_msgs::Marker::ADD;

		MarkerM->scale.x = MarkerM->scale.y = robot_size * 0.5f * 2.0;
		MarkerM->scale.z = HEIGHT_MARKER;

		MarkerM->color = color;

		MarkerM->pose.position.x = _position_x;
		MarkerM->pose.position.y = _position_y;
		MarkerM->pose.position.z = HEIGHT_MARKER * 0.5;

		MarkerM->pose.orientation.x = 0.0;
		MarkerM->pose.orientation.y = 0.0;
		MarkerM->pose.orientation.z = 0.0;
		MarkerM->pose.orientation.w = 1.0;

		msg_robot_model = MarkerM;
	}

	void set_Color(double red, double green, double blue, double a)
	{
		if (msg_robot_model != NULL)
		{
			msg_robot_model->color.r = red;
			msg_robot_model->color.g = green;
			msg_robot_model->color.b = blue;
			msg_robot_model->color.a = a;
		}
	}

	void set_Position(double x, double y)
	{

		msg_robot_model->pose.position.x = x;
		msg_robot_model->pose.position.y = y;
	}

	void set_Orientation(double x, double y, double z, double w = 1.0)
	{

		msg_robot_model->pose.orientation.x = x;
		msg_robot_model->pose.orientation.y = y;
		msg_robot_model->pose.orientation.z = z;

		msg_robot_model->pose.orientation.w = w;
	}

	void set_ID(int num)
	{
		msg_robot_model->id = num;
	}

	void publish_messages()
	{

		bool pub = false;
		float secs = 2;

		clock_t delay = secs * CLOCKS_PER_SEC;

		clock_t start = clock();

		while (!pub)
		{
			if (msg_robot_model != NULL && robot_model_publisher.getNumSubscribers() > 0)
			{
				//ROS_INFO("publish : position x = %f, position y = %f", msg_robot_model->pose.position.x, msg_robot_model->pose.position.y);
				robot_model_publisher.publish(msg_robot_model);
				pub = true;
			}
			else
			{
				ROS_INFO("Fail.. After 2Seconds Try Again...");
			}

			while (clock() - start < delay)
				;
		}
	}
};
