#include <AP_HAL/AP_HAL.h>
#include "AC_Sprayer.h"

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when traveling 1m/s expressed as a percentage
    // @Units: percentage
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: percentage
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer(const AP_InertialNav* inav, AC_Flowermeter &flower) :
    _inav(inav),
	_flower(flower),
    _speed_over_min_time(0),
    _speed_under_min_time(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }
	
	_pwm_pos = 800.0f;
    // To-Do: ensure that the pump and spinner servo channels are enabled
}

void AC_Sprayer::run(const bool true_false)
{
    // return immediately if no change
    if (true_false == _flags.running) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = true_false && _enabled;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }
}

void AC_Sprayer::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);

    _flags.spraying = false;
	_pwm_pos = 800.0f;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void
AC_Sprayer::update()
{
    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !running()) {
        run(false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    const Vector3f &velocity = _inav->get_velocity();
    float ground_speed = norm(velocity.x,velocity.y);

    // get the current time
    const uint32_t now = AP_HAL::millis();

    bool should_be_spraying = _flags.spraying;
    // check our speed vs the minimum
    if (ground_speed >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                    should_be_spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    }else{
        // we are under the min speed.
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    should_be_spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }

    // if testing pump output speed as if traveling at 6m/s
    if (_flags.testing) {
        ground_speed = 600.0f;
        should_be_spraying = true;
    }
	//hal.console->printf("sprayer %.2f, refpwm %.2f\n", _pwm_pos, _flower.get_refPwm());
    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        /*float pos = ground_speed * _pump_pct_1ms;
        pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        pos = MIN(pos,10000); // clamp to range*/
		_pwm_pos += (ground_speed * _flower.get_refPwm() * 0.01 - _flower.get_flowerVel()) * _pump_pct_1ms;
		_pwm_pos = MAX(_pwm_pos, 100 *_pump_min_pct); // ensure min pump speed
		_pwm_pos = MIN(_pwm_pos,10000); // clamp to range*/
        SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, /*pos*/_pwm_pos, 0, 10000);
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
        _flags.spraying = true;
    }else{
        stop_spraying();
    }
}
