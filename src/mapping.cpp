#define _GLIBCXX_USE_C99 1

#include "../include/utilities.h"
#include "../include/node.h"
#include "../include/room.h"
#include "../include/mapping.h"
#include "../include/persistence1d.hpp" 
#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace p1d;

#define STARTING_ANGLE -180
#define LINEAR_SPEED 0.3
#define ANGULAR_SPEED 0.2
#define STOPPING_DISTANCE 1.5

bool FEW_LASERS_MODE;

vector<Node> nodes;
vector<Room> rooms;

vector<vector< std::pair<float, float> > > lasers;

bool end;
int largest_room_id;
float largest_room_center_x;
double order_time;

bool initBlindSpot;

namespace jpmsc_mapping
{
	Mapping::Mapping(int argc, char **argv)
	{
		if(argc != 3)
		{
			ROS_ERROR("Usage : stdr_line following <robot_frame_id> <laser_frame_id>");
			exit(0);
		}

		laser_topic = string("/") + string(argv[1]) + string("/") + string(argv[2]);
		speeds_topic = string("/") + string(argv[1]) + string("/cmd_vel");
		
		subscriber = n.subscribe(laser_topic.c_str(), 1, &Mapping::callback, this);

		cmd_vel_pub = n.advertise<geometry_msgs::Twist>(speeds_topic.c_str(), 1);


		ifstream file("vars.config");
		string str;
		string delimiter = "=";
		getline(file, str);
		size_t pos;
		string var;
		string var_value;
		while((pos = str.find(delimiter)) != string::npos)
		{
			var = str.substr(0, pos);
			var_value = str.erase(0, pos + delimiter.length());
		}
		int value = atoi(var_value.c_str());
		
		if(value == 1) FEW_LASERS_MODE = false;
		else FEW_LASERS_MODE = true;

		current_angle = 0.0;
		current_float_x = 3.0;
		current_float_y = 3.0;
		
		current_room = 1;
		Room r(current_room);
		addRoom(&rooms, r);
		
		on_door = false;
		max_angle = 0.0;
		linear_speed = 0.0;
		angular_speed = 0.0;
		
		for(int i = 0; i < 4; i++)
		{
			vector< std::pair<float,float> > v;
			lasers.push_back(v);
		}
	}

	Mapping::~Mapping(void){}

  	void Mapping::callback(const sensor_msgs::LaserScan& msg)
	{
		scan = msg;
		
		if(!FEW_LASERS_MODE)
		{
			if(!end)
			{			
				updateCurrentPosition(scan);
				
				whichRoom(scan);
				
				updateMapLL(scan);
			
				movement(scan);
			}
			else
			{
				updateCurrentPosition(scan);
				
				whichRoom(scan);
				
				moveToTargetRoom();
			}
		}
		else
		{
			if(!end)
			{			
				updateCurrentPosition(scan);
				
				whichRoom(scan);
				
				updateMapSL(scan);
			
				movement(scan);
			}
			else
			{
				updateCurrentPosition(scan);
				
				whichRoom(scan);
				
				moveToTargetRoom();
			}
		}
	}
	
	void Mapping::updateCurrentPosition(const sensor_msgs::LaserScan& scan)
	{
		//cout << "Updating current position" << endl;
		
		double now = get_wall_time();
		
		double elapsed = abs(now - order_time);
		
		cout << "Elapsed: " << elapsed << endl;

		float distance_moved = elapsed * linear_speed;
		
		cout << "Distance " << distance_moved << endl;
		
		current_float_x += distance_moved;
		current_float_y += 0.0;

		cout << "Current position (float) " << current_float_x << endl;
		//cout << "Current position updated" << endl;
	}
	
	void Mapping::updateMapLL(const sensor_msgs::LaserScan& scan)
	{
		cout << "Updating Map" << endl;
		
		vector<float> chart;
		vector<float> chart_angle;
		
    	for(int i = 0; i < scan.ranges.size(); i++)
    	{
			float sensor_angle = STARTING_ANGLE + rTd(scan.angle_increment) * i;
			float distance = scan.ranges[i];
			
			if(distance > scan.range_max)
			{
				distance = scan.range_max;
			}
			
			chart.push_back(round_2dp(distance));
			chart_angle.push_back(round_2dp(sensor_angle));
		}
		
		Persistence1D p;
		p.RunPersistence(chart);
		
		vector<TPairedExtrema> extrema;
		p.GetPairedExtrema(extrema, 0.1);
		
		for(vector<TPairedExtrema>::iterator it = extrema.begin(); it != extrema.end(); it++)
		{
			int max_index = (*it).MaxIndex;
			float corner_distance = chart.at(max_index);
			float corner_angle = chart_angle.at(max_index);
			float map_x = round_2dp((cos(dTr(corner_angle)) * corner_distance) + current_float_x);
			float map_y = round_2dp((sin(dTr(corner_angle)) * corner_distance) + current_float_y);
			
			if(corner_distance < scan.range_max) // false corners
			{
				Node n(map_x, map_y, corner_angle, corner_distance, current_float_x);
				insertNode(&nodes, n, true);
			}
		}
		
		cout << "Number of corners detected " << nodes.size() << endl;
		
		assignNodes();
		
		cout << "Number of rooms " << rooms.size() << endl;
		for(int i = 0; i < rooms.size(); i++)
		{
			Room r = rooms.at(i);
			cout << "Nodes in room " << r.getId() << ": " << r.getCorners().size() << endl;
		}
		
		cout << "Map Updated" << endl;
	}

	void Mapping::updateMapSL(const sensor_msgs::LaserScan& scan)
	{
		cout << "Updating Map few lasers " << endl;
	
		for(int i = 0; i < scan.ranges.size(); i++)
    	{
			pair<float, float> pr = make_pair <float ,float> (scan.ranges[i], current_float_x);
			if(!repeatedPair(&lasers.at(i), pr))
			{
				lasers.at(i).push_back(pr);
			}
		}
		
		vector<pair<float, float> > laserm90 = lasers.at(1);
		vector<pair<float, float> > laser90 = lasers.at(3);
		
		for(int i = 0; i < laserm90.size() - 1; i++)
		{
			pair<float, float> first = laserm90.at(i);
			pair<float, float> second = laserm90.at(i + 1);
			
			Node n1(round_2dp(second.second), 3.0 - first.first, -90, first.first, second.second);
			Node n2(round_2dp(second.second), 3.0 - second.first, -90, second.first, second.second);
			
			insertNode(&nodes, n1, false);
			insertNode(&nodes, n2, false);
		}
		
		for(int i = 0; i < laser90.size() - 1; i++)
		{
			pair<float, float> first = laser90.at(i);
			pair<float, float> second = laser90.at(i + 1);
			
			Node n1(round_2dp(second.second), 3.0 + first.first, 90, first.first, second.second);
			Node n2(round_2dp(second.second), 3.0 + second.first, 90, second.first, second.second);
			
			insertNode(&nodes, n1, false);
			insertNode(&nodes, n2, false);
		}
		
		if(scan.ranges[0] < scan.range_max) // behind robot 
		{
			if(!initBlindSpot)
			{
				Node n1(round_2dp(current_float_x - scan.ranges[0]), 3.0 + scan.ranges[3], 90, scan.ranges[3], current_float_x);
				Node n2(round_2dp(current_float_x - scan.ranges[0]), 3.0 - scan.ranges[1], -90, scan.ranges[1], current_float_x);
				 
				insertNode(&nodes, n1, false);
				insertNode(&nodes, n2, false);
				
				initBlindSpot = true;
			}
		}
		
		assignNodes();
	}
	
	void Mapping::movement(const sensor_msgs::LaserScan& scan)
	{
		cout << "Moving" << endl;
		geometry_msgs::Twist cmd;
		
		cmd.linear.x = LINEAR_SPEED;
		
		for(int i = 0; i < scan.ranges.size(); i++)
		{
			float distance = scan.ranges[i];
			float sensor_angle = STARTING_ANGLE + rTd(scan.angle_increment) * i;
			
			if(sensor_angle <= 10 && sensor_angle >= -10)
			{
				if(distance < STOPPING_DISTANCE)
				{
					cmd.linear.x = 0.0;
					cout << "Reached the end " << endl;
					cmd_vel_pub.publish(cmd);
					theEnd(scan);
					break;
				}
			}
		}

		linear_speed = cmd.linear.x;

		cmd_vel_pub.publish(cmd);
		
		cout << "Moved" << endl;
		
		order_time = get_wall_time();
	}
	
	void Mapping::whichRoom(const sensor_msgs::LaserScan& scan)
	{
		cout << "Deciding which room I'm in" << endl;
		
		float min_distance_left = FLT_MAX;
		float min_distance_right = FLT_MAX;
		
		float distance_threshold = 1.5;
		
		for(int i = 0; i < scan.ranges.size(); i++)
		{
			float sensor_angle = STARTING_ANGLE + rTd(scan.angle_increment) * i;
			float distance = scan.ranges[i];
			
			if(sensor_angle >= -95 && sensor_angle <= -85) //right side
			{
				if(distance < min_distance_right)
				{
					min_distance_right = distance;
				}
			} 
			else if(sensor_angle <= 95 && sensor_angle >= 85) //left side
			{
				if(distance < min_distance_left)
				{
					min_distance_left = distance;
				}
			}
		}
		
		int val = end ? -1 : 1;
		
		if(min_distance_right < distance_threshold && min_distance_left < distance_threshold)
		{
			if(on_door) // in the passage between one room and another
			{
				cout << "Passing between room " << current_room << " and room " << current_room + val << endl;
			}
			else // leaving the currrent room
			{
				cout << "Leaving room " << current_room << endl;
				on_door = true;
			}
		}
		else
		{
			if(on_door) // entering new room
			{
				on_door = false;
				cout << "Left " << current_room << " and entered room " << current_room + val << endl;
				current_room = current_room + val;
				if(!end)
				{
					Room r(current_room);
					addRoom(&rooms, r);
				}
			}
			else // in room
			{
				cout << "I'm in room " << current_room << endl;
			}
		}
	}
	
	void Mapping::assignNodes()
	{
		for(int i = 0; i < nodes.size(); i++)
		{
			Node& n = nodes.at(i);
	
			if(!n.isAssigned() && n.getX() < current_float_x)
			{
				rooms.at(current_room - 1).addCorner(n);
				n.assign(current_room);
			}
		}
	}
	
	void Mapping::theEnd(const sensor_msgs::LaserScan& scan)
	{
		end = true;
		
		if(FEW_LASERS_MODE)
		{
			theEndSL(scan);
		}
		else
		{
			theEndLL();
		}
	}

	void Mapping::theEndLL()
	{
		cout << "Number of rooms " << rooms.size() << endl;
		
		for(int i = 0; i < nodes.size(); i++)
		{
			Node& n = nodes.at(i);
	
			if(!n.isAssigned())
			{
				rooms.at(current_room - 1).addCorner(n);
				n.assign(current_room);
			}
		}
		
		for(int i = 0; i < rooms.size(); i++)
		{
			Room* r = &rooms.at(i);
			r->calculateCenter();
			r->orderNodes(true);
			r->calculateArea();
		}
		
		Room r = getLargestRoom(&rooms);
		
		cout << "Largest Room: " << r.getId() << endl;
		cout << "Area: " << r.getArea() << endl;
		cout << "Center " << r.getCenterX() << endl;
		
		largest_room_id = r.getId();
		largest_room_center_x = r.getCenterX();
	}
	
	void Mapping::theEndSL(const sensor_msgs::LaserScan& scan)
	{
		cout << "Number of rooms " << rooms.size() << endl;
		
		/**
		 * Last Blindspot
		 * */
		Node n1(round_2dp(current_float_x + STOPPING_DISTANCE), 3.0 + scan.ranges[3], 90, scan.ranges[3], current_float_x);
		Node n2(round_2dp(current_float_x + STOPPING_DISTANCE), 3.0 - scan.ranges[1], -90, scan.ranges[1], current_float_x);
		 
		insertNode(&nodes, n1, false);
		insertNode(&nodes, n2, false);
		
		rooms.at(current_room - 1).addCorner(n1);
		n1.assign(current_room);
		
		rooms.at(current_room - 1).addCorner(n2);
		n2.assign(current_room);
		
		for(int i = 0; i < rooms.size(); i++)
		{
			Room* r = &rooms.at(i);
			r->calculateCenter();
			r->orderNodes(false);
			r->calculateArea();
			cout <<  " AREA " << r->getArea() << endl;
		}
		
		Room r = getLargestRoom(&rooms);
		
		cout << "Largest Room: " << r.getId() << endl;
		cout << "Area: " << r.getArea() << endl;
		cout << "Center " << r.getCenterX() << endl;

		largest_room_id = r.getId();
		largest_room_center_x = r.getCenterX();
	}

	void Mapping::moveToTargetRoom()
	{
		cout << "Towards Largest Room" << endl;
		cout << "Moving to Room " << largest_room_id << endl;
		
		geometry_msgs::Twist cmd;
		
		if(current_room == largest_room_id && current_float_x < largest_room_center_x)
		{
			cout << "Reached the largest room" << endl;
			cout << "My position " << current_float_x << endl;
			cmd.linear.x = 0.0;
			cmd_vel_pub.publish(cmd);
			
			for(int i = 0; i < rooms.size(); i++)
			{
				Room& r = rooms.at(i);
				cout << "Nodes in room " << r.getId() << ": " << r.getCorners().size() << " -- " << r.getArea() << r.getCenterX() << endl;
			}
			
			exit(0);
		}
		else
		{
			cmd.linear.x = -LINEAR_SPEED;
		}
		
		linear_speed = cmd.linear.x;
		
		cmd_vel_pub.publish(cmd);
		
		order_time = get_wall_time();
	}
}






