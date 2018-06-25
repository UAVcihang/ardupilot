/*
 * control_position.cpp
 *
 *  Created on: 2018-6-1
 *      Author: liwh1
 *
 * ╦двт px4 mc_pos_control.cpp
 *  RC mode where RPT sticks control speed in corresponding directions.
 *  Centered sticks level vehicle and hold it to fixed position and altitude against wind.
 *  Centered RPT sticks hold x, y, z position steady against any wind and levels attitude.
 *	Outside center:
 *		Roll/Pitch sticks control speed over ground in left-right and forward-back directions (respectively) relative to the "front" of the vehicle.
 *		Throttle stick controls speed of ascent-descent.
 *		Yaw stick controls rate of angular rotation above the horizontal plane.
 */
#include "Copter.h"

bool Copter::position_init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok() && !ignore_checks) {
        return false;
    }
    return true;
}

void Copter::position_run(void)
{

}





