/*
 * AC_Flowermeter.h
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */

#pragma once

#include "AC_Flowermeter.h"

#include "AC_Flowermeter_Backend.h"
#include <Filter/Filter.h>
#include <AP_Math/AP_Math.h>

class AC_Flowermeter_Pin : public AC_Flowermeter_Backend
{
public:
    // constructor
    AC_Flowermeter_Pin(AC_Flowermeter &flower, AC_Flowermeter::FL_State &state);
    static void irq_handler0(void);

    // update state
    void update(void);

private:
    static void irq_handler(uint8_t instance);

    
    uint8_t last_pin = -1;
    uint32_t last_gpio;
    struct IrqState {
        uint32_t last_pulse_us;
        uint32_t dt_period;
        uint32_t dt_sum;
        uint32_t dt_count;
    };
    static struct IrqState irq_state[2];
};
