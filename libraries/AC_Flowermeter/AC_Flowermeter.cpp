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
    // @Param: Flowermeter dosage
    // @DisplayName: Max Flowermeter dosage
    // @Description:
    // @Values: L/area
    // @User: Standard
    AP_GROUPINFO("_DSGE", 3, AC_Flowermeter, _dosage, 1.2),

	// @Param: Flowermeter speed
	// @DisplayName: Max Flowermeter speed
	// @Values: L/min
	AP_GROUPINFO("_VEL", 4, AC_Flowermeter, _flowerVel, 1.5),


    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AC_Flowermeter::AC_Flowermeter()
{
    AP_Param::setup_object_defaults(this, var_info);

	// init state and drivers
    //memset(state,0,sizeof(state));
	state._count = 0;
	state._last_timestamp = 0;
	state._period = 0;
	state._vel_flower = 0;
	// drivers = nullptr;

	_empty = false;
	_dt = 0.2; // 5HZ
	_pwm_count_max = 0;
}

void AC_Flowermeter::init(float dt)
{
	_fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
	if (_fd == -1) {
	     hal.console->printf("Unable to open PX4 flowermeter\n");
	     // set_status(RangeFinder::RangeFinder_NotConnected);
	     return;
	}

	_dt = dt;
	float minspeed = MIN(PUMP_MAX_SPEED, FLOW_MAX_SPEED) / 60.0f;
	_pwm_count_max = minspeed * PWM_RATIO * _dt ;
	if(is_zero(_dosage)){
		_dosage = 0.5;// 0.5L/area
	}

}

AC_Flowermeter::~AC_Flowermeter()
{
    if (_fd != -1) {
        close(_fd);
    }
}

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
	if(_fd == -1)
		return;

	struct pwm_input_s pwm;
	// uint32_t now = hal.scheduler->millis();

	if (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm) && pwm.timestamp != state._last_timestamp) {
		//hal.console->printf("flower vel %d\n", pwm.error_count - _count);
		state._vel_flower = (pwm.error_count - state._count);
		state._count = pwm.error_count;
		state._period = pwm.period;
		state._last_timestamp = pwm.timestamp;
		//_valid = true;
		//_last_count = _count;
		//_count_totally += _count;
	}


	if(state._count >= PWM_COUNT_MAX) {
		_empty = true;
	}
	/*else
		_valid = false;*/
	//hal.console->printf("flower %d\t%d\n", _count, _last_count);

	//_vel_flower = (uint16_t)((_count - _last_count) / 10);


}

#endif



