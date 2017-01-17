#ifndef STDR_MAPPING
#define STDR_MAPPING

#include <iostream>
#include <ctime>
#include <iostream>
#include <fstream>

#include <ros/package.h>
#include "ros/ros.h"

#include <stdr_msgs/RobotIndexedVectorMsg.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

using namespace std;

namespace jpmsc_mapping
{	
	class Mapping
	{
	private:
		sensor_msgs::LaserScan scan;
			
		ros::Subscriber subscriber;
		ros::NodeHandle n;
		ros::Publisher cmd_vel_pub;
		ros::Publisher rviz_pub;
		
		std::string laser_topic;
		std::string speeds_topic;
		std::string rviz_topic;

		float current_angle;
		float current_float_x;
		float current_float_y;
		
		float linear_speed;
		float angular_speed;
		float max_angle;

		int current_room;
		bool on_door;
		
	public:
		Mapping(int argc,char **argv);
		~Mapping(void);
		void callback(const sensor_msgs::LaserScan& msg);
		void updateCurrentPosition(const sensor_msgs::LaserScan& scan);
		void updateMapLL(const sensor_msgs::LaserScan& scan);
		void updateMapSL(const sensor_msgs::LaserScan& scan);
		void movement(const sensor_msgs::LaserScan& scan);	
		void whichRoom(const sensor_msgs::LaserScan& scan);
		void assignNodes();
		void theEnd(const sensor_msgs::LaserScan& scan);
		void theEndLL();
		void theEndSL(const sensor_msgs::LaserScan& scan);
		void moveToTargetRoom();
  	};
}

#endif
