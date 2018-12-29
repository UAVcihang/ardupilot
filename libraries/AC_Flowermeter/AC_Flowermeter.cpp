/*
 * AC_Flowermeter.cpp
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */
#include <AP_HAL/AP_HAL.h>
#include "AC_Flowermeter.h"
#include "AC_Flowermeter_Pin.h"
/*
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
#include <math.h>*/

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Flowermeter::var_info[] = {
	// @Param: _TYPE
    // @DisplayName: RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM,2:AUXPIN
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, AC_Flowermeter, _type, 2),
	
    // @Param: _PIN
    // @DisplayName: Input pin number
    // @Description: Which pin to use
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("_PIN",    1, AC_Flowermeter, _pin, 54),
	
	
    // @Param: _volume
    // @DisplayName: volume
    // @Description: max volume default 10L
    // @Values: 10L
	// @User: Standard
	AP_GROUPINFO("_VOL",    2, AC_Flowermeter, _volume, 10),
    // @Param: Flowermeter velocity
    // @DisplayName: Max Flowermeter velocity
    // @Description:
    // @Values: 300-6000mL/min
    // @User: Standard
    AP_GROUPINFO("_VEL", 3, AC_Flowermeter, _flowerVel, 1.5),



    AP_GROUPEND
};

AC_Flowermeter::AC_Flowermeter()/*:
	_last_timestamp(0),
	_count(0),
	_last_count(0),
	_vel_flower(0),
	_count_totally(0),
	_period(0),
	_valid(false)*/
{
    AP_Param::setup_object_defaults(this, var_info);
	
	// init state and drivers
    //memset(state,0,sizeof(state));
	state._count = 0;
	state._last_timestamp = 0;
	state._period = 0;
	state._vel_flower = 0;
	drivers = nullptr;
	
	_empty = false;
}

void AC_Flowermeter::init()
{
	/*_fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
	if (_fd == -1) {
	     hal.console->printf("Unable to open PX4 flowermeter\n");
	     // set_status(RangeFinder::RangeFinder_NotConnected);
	     return;
	}*/
	switch(_type){
		case FL_TYPE_PIN:
			drivers = new AC_Flowermeter_Pin(*this, state);
			break;
		default:
			break;
	}
}

/*AC_Flowermeter::~AC_Flowermeter()
{
    if (_fd != -1) {
        close(_fd);
    }
}*/

// see if pwm_input is available
/*bool AC_Flowermeter::detect()
{
    int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    close(fd);
    return true;
}*/

void AC_Flowermeter::update(void)
{
/*	if(_fd == -1)
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
	}*/
	if(drivers != nullptr) {
		drivers->update();
		
		// 
		if(state._count >= PWM_COUNT_MAX) {
			_empty = true;
		}
	}

}

//#endif



