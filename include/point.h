#ifndef STDR_MAPPING_POINT
#define STDR_MAPPING_POINT

class Point
{
	private:
		int _x;
		int _y;
		
	public:
		void setValues(int x, int y)
		{
			_x = x;
			_y = y;
		}
		
		int getX()
		{
			return _x;
		}
		
		int getY()
		{
			return _y;
		}
};

#endif
	
