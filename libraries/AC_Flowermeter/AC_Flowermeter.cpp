/*
 * AC_Flowermeter.cpp
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AC_Flowermeter.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <uORB/topics/pwm_input.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

AC_Flowermeter::AC_Flowermeter():
	_last_timestamp(0),
	_count(0),
	_last_count(0),
	_vel_flower(0),
	_count_totally(0),
	_period(0),
	_valid(false)
{

}

void AC_Flowermeter::init()
{
	_fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
	if (_fd == -1) {
	     hal.console->printf("Unable to open PX4 flowermeter\n");
	     // set_status(RangeFinder::RangeFinder_NotConnected);
	     return;
	}
}

AC_Flowermeter::~AC_Flowermeter()
{
    if (_fd != -1) {
        close(_fd);
    }
}

// see if pwm_input is available
bool AC_Flowermeter::detect()
{
    int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    close(fd);
    return true;
}

void AC_Flowermeter::update(void)
{
	if(_fd == -1)
		return;

	struct pwm_input_s pwm;
	// uint32_t now = hal.scheduler->millis();

	if (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm) && pwm.timestamp != _last_timestamp) {
		//hal.console->printf("flower vel %d\n", pwm.error_count - _count);
		_vel_flower = (pwm.error_count - _count);
		_count = pwm.error_count;
		_period = pwm.period;
		_last_timestamp = pwm.timestamp;
		//_valid = true;
		//_last_count = _count;
		//_count_totally += _count;
	}
	/*else
		_valid = false;*/
	//hal.console->printf("flower %d\t%d\n", _count, _last_count);

	//_vel_flower = (uint16_t)((_count - _last_count) / 10);


}

#endif



