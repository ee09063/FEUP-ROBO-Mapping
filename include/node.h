#ifndef STDR_MAPPING_NODE
#define STDR_MAPPING_NODE

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;
		
#define NODE_MIN_DISTANCE 0.5
			
class Node
{
private:
	float _pos_x;
	float _pos_y;
	float _angle;
	float _distance;
	bool _assigned;
	int _room;
	
public:
	float robot_x;
	float getX() const
	{
		return _pos_x;
	}
	
	float getY() const
	{
		return _pos_y;
	}
	
	float getAngle() const
	{
		return _angle;
	}
	
	void setAngle(float angle)
	{
		_angle = angle;
	}
	
	float getDistance() const
	{
		return _distance;
	}
	
	int getRoom()
	{
		return _room;
	}
	
	bool isAssigned()
	{
		return _assigned;
	}
	
	Node(float pos_x, float pos_y, float angle, float distance, float r_x)
	{
		_pos_x = pos_x;
		_pos_y = pos_y;
		_angle = angle;
		_distance = distance;
		robot_x = r_x;
		_assigned = false;
	}
	
	void assign(int room_id)
	{
		_room = room_id;
		_assigned = true;
	}
	
	void unassign()
	{
		_room = 0;
		_assigned = false;
	}
};
	
float manhattanDistance(Node n1, Node n2)
{
	return abs(n1.getX() - n2.getX()) + abs(n1.getY() - n2.getY());
}
	
float squareDistance(Node n1, Node n2)
{
	return sqrt(pow((n1.getX() - n2.getX()), 2.0) + pow((n1.getY() - n2.getY()), 2.0));
}
	
bool nodeInVicinityLL(vector<Node>* nodes, Node n)
{
	for(int i = 0; i < nodes->size(); i++)
	{
		Node n_vec = nodes->at(i);
		if(manhattanDistance(n_vec, n) < NODE_MIN_DISTANCE)
		{
			return true;
		}
	}
	return false;
}

bool nodeInVicinitySL(vector<Node>* nodes, Node n)
{
	for(int i = 0; i < nodes->size(); i++)
	{
		Node n_vec = nodes->at(i);
		if(manhattanDistance(n_vec, n) < (NODE_MIN_DISTANCE / 5.0))
		{
			return true;
		}
	}
	return false;
}

struct less_than_key_node
{
	inline bool operator() (const Node &n1, const Node &n2)
	{
		return (n1.getAngle() < n2.getAngle());
	}
};

void insertNode(vector<Node>* nodes, Node n, bool LL)
{
	if(LL)
	{
		if(!nodeInVicinityLL(nodes, n))
		{
			nodes->push_back(n);
			sort(nodes->begin(), nodes->end(), less_than_key_node());
		}
	} 
	else
	{
		if(!nodeInVicinitySL(nodes, n))
		{
			nodes->push_back(n);
			sort(nodes->begin(), nodes->end(), less_than_key_node());
		}
	}
}

#endif
