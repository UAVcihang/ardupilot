/*
 * AC_Flowermeter.h
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */

#ifndef AC_FLOWERMETER_H_
#define AC_FLOWERMETER_H_

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define PWM_RATIO  1600 //4600 // F = 98 *Q(L/Min)
#define PWM_COUNT_MAX  15800 //45500  // PWM_RATIO * 10L
#define PWM_COUNT_PER_AERA 2.4// 6.9 // PWM_RATIO / 666.6667
#define PUMP_MAX_SPEED  2.7 // 2.7L/min
#define FLOW_MAX_SPEED  4.5 // 4.5L/min


class AC_Flowermeter
{
public:
    // constructor
	AC_Flowermeter();

    // destructor
    ~AC_Flowermeter(void);

	struct FL_State {
    uint64_t _count;
    //uint64_t _last_count;
    uint64_t _last_timestamp;
    //uint64_t _count_totally;
    uint64_t _period;

    uint16_t _vel_flower;

	};

    // update state
    void update(void);
    void init(float dt);

    // get count
    uint16_t get_cur_flowerVel(){ return state._vel_flower; }
    uint64_t get_pwmCount(){ return state._count; }
    //uint64_t get_totallyCount() { return state._count_totally; }
    uint64_t last_update() { return state._last_timestamp; }
    uint64_t get_period() { return state._period; }


	float get_refPwm(void) {
		/// float temp = (float)_dosage.get() * _width * PWM_COUNT_PER_AERA * _dt;

		float temp = _flowerVel * PWM_RATIO * _dt  / 60.0f;///(float)_dosage.get() * _width * PWM_COUNT_PER_AERA * _dt;
		temp = constrain_float(temp, 0, _pwm_count_max);
		return temp;
		//return (float)_dosage * _width * PWM_COUNT_PER_AERA * _dt; // 0.1 means times = 1/fs
	}

	float get_maxFlyVel(void) {
		float minFlowspeed = MIN(PUMP_MAX_SPEED, FLOW_MAX_SPEED) / 60.0f;
		if(_width == 0) _width = 3.0f;
		if(is_zero(_dosage.get())){
		_dosage = 0.5f;
		}
		return minFlowspeed  * 666.667 / _width / _dosage;
	}


	float get_maxFlowerVel()
	{
		return MIN(PUMP_MAX_SPEED, FLOW_MAX_SPEED);
	}

	void set_flowerVel(float ratio)
	{
		_flowerVel = MIN(PUMP_MAX_SPEED, FLOW_MAX_SPEED) * ratio;
	}

	void set_width(uint16_t width) {
	_width = width;
	}

    void set_valid(bool value){
    	_valid = value;
    }

    bool get_valid(){
    	return _valid;
    }

	bool isEmpty() {
		return _empty;
	}

	AP_Int8  _type;
	AP_Int8  _pin;
	AP_Int8  _volume;

	AP_Float _dosage; // L/aera
    AP_Float _flowerVel; // L/min


    static const struct AP_Param::GroupInfo var_info[];
private:
    int _fd;
    bool _valid;
	bool _empty;
	float _dt;
	float _pwm_count_max;
	//float velocity_max;

	uint16_t _width;
	FL_State state;
	//AC_Flowermeter_Backend *drivers;
    // uint64_t _last_pulse_time_ms;
    // uint32_t _disable_time_ms;
    // uint32_t _good_sample_count;
    // float _last_sample_distance_cm;
};


#endif /* AC_FLOWERMETER_H_ */
