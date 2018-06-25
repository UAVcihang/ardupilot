/*
 * AC_TerrainEstimator.cpp
 *
 *  Created on: 2018-5-28
 *      Author: liwh1
 *      From PX4 terrain_estimator
 */
#include "AC_TerrainEstimator.h"

AC_TerrainEstimator::AC_TerrainEstimator(const AP_AHRS* ahrs, const RangeFinder* rangefinder):
_ahrs(ahrs),
_rangefinder(rangefinder),
_terrain_valid(false),
_distance_last(0.0f),
_time_last_distance(0),
_time_last_gps(0)
{
	_x.x = _x.y = _x.z = 0.0f;
	//memset(_x, 0, sizeof(_x));
	_u_z = 0.0f;
	_P.identity();

}


bool AC_TerrainEstimator::is_distance_valid(float distance)
{
	if(distance > 40.0f || distance < 0.00001f) {
		return false;
	}
	else {
		return true;
	}
}

void AC_TerrainEstimator::predict(float dt)
{

}



