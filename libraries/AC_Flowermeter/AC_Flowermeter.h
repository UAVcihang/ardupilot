/*
 * AC_Flowermeter.h
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */
#pragma once

#ifndef AC_FLOWERMETER_H_
#define AC_FLOWERMETER_H_

#include <AP_Param/AP_Param.h>

#define PWM_RATIO  4600 // F = 98 *Q(L/Min)����1Lˮ������������
#define PWM_COUNT_MAX  45500  // PWM_RATIO * 10L
#define PWM_COUNT_PER_AERA 6.9 // PWM_RATIO / 666.6667
// Maximum number of RPM measurement instances available on this platform
//#define RPM_MAX_INSTANCES 2
class AC_Flowermeter_Backend;

class AC_Flowermeter
{
	friend class AC_Flowermeter_Backend;
public:
    // constructor
	AC_Flowermeter();

    // destructor
   // ~AC_Flowermeter(void);

    // Flowermeter driver types
    enum Flowermeter_Type {
        FL_TYPE_NONE    = 0,
        FL_TYPE_PX4_PWM = 1,
        FL_TYPE_PIN     = 2
    };
	
	struct FL_State {
    uint64_t _count;
    //uint64_t _last_count;
    uint64_t _last_timestamp;
    //uint64_t _count_totally;
    uint64_t _period;

    uint16_t _vel_flower;

	};
    // static detection function
    //static bool detect(void);

    // update state
    void update(void);
    void init();

    // get count
    uint64_t get_flowerVel(){ return state._vel_flower; }
    uint64_t get_pwmCount(){ return state._count; }
    //uint64_t get_totallyCount() { return state._count_totally; }
    uint64_t last_update() { return state._last_timestamp; }
    uint64_t get_period() { return state._period; }


	float get_refPwm(void) {
		return (float)_flowerVel * _width * PWM_COUNT_PER_AERA * 0.1; // 0.1 means times = 1/fs
	}
	
	void set_width(uint16_t width)
	{
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
    AP_Float _flowerVel; // L/aera
    

    static const struct AP_Param::GroupInfo var_info[];
private:
    //int _fd;
    bool _valid;
	bool _empty;
	uint16_t _width;
	FL_State state;
	AC_Flowermeter_Backend *drivers;
	
	//void detect_instance();
	//void update_instance();
    // uint64_t _last_pulse_time_ms;
    // uint32_t _disable_time_ms;
    // uint32_t _good_sample_count;
    // float _last_sample_distance_cm;
};


#endif /* AC_FLOWERMETER_H_ */
