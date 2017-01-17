#ifndef STDR_MAPPING_UTILITIES
#define STDR_MAPPING_UTILITIES

#include <cmath>
#include <vector>
#include <time.h>
#include <sys/time.h>

using namespace std;

float rTd(float radian)
{
	return 	radian * (180 / 3.1415);
}

float dTr(float degree)
{
	return degree * (3.1415 / 180);
}

int round(float f)
{
	return (int)floor(f + 0.5);
}

float round_2dp(float to_round)
{
	return roundf(to_round * 100) / 100;
}

bool repeatedPair(vector<pair<float, float > >* vec, pair<float, float > pr)
{
	if(vec->size() == 0)
	{
		return false;
	}
	
	pair<float, float> p = vec->at(vec->size() - 1);
	if(abs(pr.first - p.first) < 0.05)
	{
		return true;
	}
	return false;
}

double get_wall_time()
{
    struct timeval time;
    if (gettimeofday(&time,NULL))
    {
        return 0;
    }
    
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

#endif
