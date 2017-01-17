#ifndef STDR_MAPPING_ROOM
#define STDR_MAPPING_ROOM

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include "float.h"

using namespace std;

struct less_than_key_corner
{
	inline bool operator() (const Node &n1, const Node &n2)
	{
		return (n1.getAngle() < n2.getAngle());
	}
};

void sortCorners(vector<Node>* corners)
{
	sort(corners->begin(), corners->end(), less_than_key_corner());
}

class Room
{
private:
	vector<Node> _corners;
	float _area;
	float _center_x;
	float _center_y;
	int _id;
	
public:
	Room(int id)
	{
		_id = id;
		_area = 0;
		_center_x = 0;
		_center_y = 0;
	}
	
	int getId() const
	{
		return _id;
	}
	
	float getArea() const
	{
		return _area;
	}
	
	float getCenterX()
	{
		return _center_x;
	}
	
	float getCenterY()
	{
		return _center_y;
	}
	
	void orderNodes(bool LL)
	{
		for(int i = 0; i < _corners.size(); i++)
		{
			Node& c = _corners.at(i);
			float delta_x = c.getX() - _center_x;
			float delta_y = c.getY() - _center_y;
			float angle = atan2(delta_y, delta_x) * 180 / 3.1415;
			if(angle < 0)
			{
				angle += 360;
			}
			c.setAngle(angle);
		}
		
		sort(_corners.begin(), _corners.end(), less_than_key_corner());
		
		/*if(LL)
		{
			return;
		}
		
		vector<Node> corners;
		float total_size = _corners.size();
		corners.push_back(_corners.at(0));
		corners.push_back(_corners.at(1));
		_corners.erase(_corners.begin());
		_corners.erase(_corners.begin());
		
		while(corners.size() != total_size)
		{
			Node last = corners.at(corners.size()-1);
			
			int index = 0;
			float min_distance = FLT_MAX;
			
			for(int i = 0; i < _corners.size(); i++)
			{
				Node n = _corners.at(i);
				float sdist = squareDistance(last, n);
				if(sdist < min_distance)
				{
					index = i;
					min_distance = sdist;
				}
			}
			Node n = _corners.at(index);
			corners.push_back(_corners.at(index));
			_corners.erase(_corners.begin()+index);
		}
		
		_corners.clear();
		_corners = corners;*/
	}
	
	void calculateArea() //http://www.mathopenref.com/coordpolygonarea.html
	{
		cout << "CALCULATION AREA OF ROOM " << _id << endl;
		for(int j = 0; j < _corners.size(); j++)
		{
			Node* n = &_corners.at(j);
			cout << n->getX() << " " << n->getY() << " " << n->getAngle() << endl;
		}
		
		for(int i = 0; i < _corners.size() - 1; i++)
		{
			Node n1 = _corners.at(i);
			Node n2 = _corners.at(i + 1);
			_area += ((n1.getX() * n2.getY()) - (n1.getY() * n2.getX()));
		}
		
		Node n2 = _corners.at(0);
		Node n1 = _corners.at(_corners.size() - 1);
		_area += ((n1.getX() * n2.getY()) - (n1.getY() * n2.getX()));
		
		_area /= 2;

		_area = abs(_area);
	}
	
	void calculateCenter()
	{
		float min_x = FLT_MAX;
		float max_x = 0;
		
		float min_y = FLT_MAX;
		float max_y = 0;
		
		for(int i = 0; i < _corners.size(); i++)
		{
			Node n = _corners.at(i);
			if(n.getX() < min_x)
			{
				min_x = n.getX();
			}
			
			if(n.getX() > max_x)
			{
				max_x = n.getX();
			}
			
			if(n.getY() < min_y)
			{
				min_y = n.getY();
			}
			
			if(n.getY() > max_y)
			{
				max_y = n.getY();
			}
		}
		
		_center_x = (max_x + min_x) / 2;
		_center_y = (max_y + min_y) / 2;
	}
	
	void addCorner(Node n)
	{
		_corners.push_back(n);
	}	
	
	vector<Node> getCorners()
	{
		return _corners;
	}
	
	void clearCorners()
	{
		_corners.clear();
	}
};

bool repeatedId(vector<Room>* rooms, Room r)
{
	for(int i = 0; i < rooms->size(); i++)
	{
		Room r_vec = rooms->at(i);
		if(r_vec.getId() == r.getId())
		{
			cout << "Room with Id " << r.getId() << " already present" << endl;
			return true; 
		}
	}
	return false;
}

struct less_than_key_room
{
	inline bool operator() (const Room &r1, const Room &r2)
	{
		return (r1.getArea() > r2.getArea());
	}
};

void addRoom(vector<Room>* rooms, Room r)
{
	if(!repeatedId(rooms, r))
	{
		rooms->push_back(r);
	}
}

void sortRoomsByArea(vector<Room>* rooms)
{
	sort(rooms->begin(), rooms->end(), less_than_key_room());
}

Room getLargestRoom(vector<Room>* rooms)
{
	sortRoomsByArea(rooms);
	
	if(rooms->size() > 0)
	{
		return rooms->at(0);
	}
	else
	{
		cout << "Vector rooms is empty and should not be" << endl;
	}
}

#endif
