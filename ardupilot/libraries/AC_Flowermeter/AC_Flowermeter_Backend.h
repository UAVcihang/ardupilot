/*
 * AC_Flowermeter.h
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */
#pragma once
#ifndef AC_FLOWERMETER_Backend_H_
#define AC_FLOWERMETER_Backend_H_

#include "AC_Flowermeter/AC_Flowermeter.h"
#include <AP_HAL/AP_HAL.h>

//#define PWM_RATIO  5880 // F = 98 *Q(L/Min)流过1L水输出的脉冲个数

class AC_Flowermeter_Backend
{
public:
    // constructor
	AC_Flowermeter_Backend(AC_Flowermeter &ac_flower, AC_Flowermeter::FL_State &state);

	virtual ~AC_Flowermeter_Backend(void) {}
	
	// update stae state structure. All backends must implement this.
	virtual void update() = 0;
    
    int8_t get_pin(void) const {
        /*if (state.instance > 1) {
            return -1;
        }*/
        return _ac_flower._pin.get();
    }
	
protected:
 	AC_Flowermeter &_ac_flower;
	AC_Flowermeter::FL_State &_state;
};


#endif /* AC_FLOWERMETER_Backend_H_ */
