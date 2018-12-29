/*
 * AC_Flowermeter.cpp
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */
#include <AP_HAL/AP_HAL.h>
#include "AC_Flowermeter_Pin.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <board_config.h>
#endif

#include <stdio.h>

extern const AP_HAL::HAL& hal;
AC_Flowermeter_Pin::IrqState AC_Flowermeter_Pin::irq_state;

/* 
   open the sensor in constructor
*/
AC_Flowermeter_Pin::AC_Flowermeter_Pin(AC_Flowermeter &flower, AC_Flowermeter::FL_State &state) :
	AC_Flowermeter_Backend(flower, state)
{
}

/*
  handle interrupt on an instance
 */
void AC_Flowermeter_Pin::irq_handler()
{
    uint32_t now = AP_HAL::micros();
    uint32_t dt = now - irq_state.last_pulse_us;
    irq_state.last_pulse_us = now;
    // we don't accept pulses less than 100us. Using an irq for such
    // high RPM is too inaccurate, and it is probably just bounce of
    // the signal which we should ignore
    if (dt > 100 && dt < 1000*1000) {
        irq_state.dt_period = dt;
        irq_state.dt_count++;
    }
}



void AC_Flowermeter_Pin::update(void)
{
    if (last_pin != get_pin()) {
        last_pin = get_pin();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        uint32_t gpio = 0;

#ifdef GPIO_GPIO0_INPUT
        switch (last_pin) {
        case 50:
            gpio = GPIO_GPIO0_INPUT;
            break;
        case 51:
            gpio = GPIO_GPIO1_INPUT;
            break;
        case 52:
            gpio = GPIO_GPIO2_INPUT;
            break;
        case 53:
            gpio = GPIO_GPIO3_INPUT;
            break;
        case 54:
            gpio = GPIO_GPIO4_INPUT;
            break;
        case 55:
            gpio = GPIO_GPIO5_INPUT;
            break;
        }
#endif // GPIO_GPIO5_INPUT
        
        // uninstall old handler if installed
        if (last_gpio != 0) {
            stm32_gpiosetevent(last_gpio, false, false, false, nullptr);
        }
        irq_state.dt_count = 0;
        irq_state.dt_sum = 0;

        last_gpio = gpio;

        if (gpio == 0) {
            return;
        }
        
        // install interrupt handler on rising edge of pin. This works
        // for either polarity of pulse, as all we want is the period
        stm32_gpiosetevent(gpio, true, false, false,
                           irq_handler);
#else // other HALs
        hal.gpio->attach_interrupt(last_pin, irq_handler,
                                   HAL_GPIO_INTERRUPT_RISING);
#endif
    }

	//uint32_t now = AP_HAL::micros();
	if(AP_HAL::micros() - irq_state.last_pulse_us < 1000000) // 1s
	{
		_state._period = irq_state.dt_period;
		_state._vel_flower = irq_state.dt_count - _state._count;
		_state._count = irq_state.dt_count;
		_state._last_timestamp = AP_HAL::millis();
	}
/*    if (irq_state.dt_count > 0) {
        float dt_avg;

        // disable interrupts to prevent race with irq_handler
        void *irqstate = hal.scheduler->disable_interrupts_save();
        dt_avg = irq_state.dt_sum / irq_state.dt_count;
        irq_state.dt_count = 0;
        irq_state.dt_sum = 0;
        hal.scheduler->restore_interrupts(irqstate);

        const float scaling = ap_rpm._scaling[state.instance];
        float maximum = ap_rpm._maximum[state.instance];
        float minimum = ap_rpm._minimum[state.instance];
        float quality = 0;
        float rpm = scaling * (1.0e6 / dt_avg) * 60;
        float filter_value = signal_quality_filter.get();

        state.rate_rpm = signal_quality_filter.apply(rpm);
        
        if ((maximum <= 0 || rpm <= maximum) && (rpm >= minimum)) {
            if (is_zero(filter_value)){
                quality = 0;
            } else {
                quality = 1 - constrain_float((fabsf(rpm-filter_value))/filter_value, 0.0, 1.0);
                quality = powf(quality, 2.0);
            }
            state.last_reading_ms = AP_HAL::millis();
        } else {
            quality = 0;
        }
        state.signal_quality = (0.1 * quality) + (0.9 * state.signal_quality);
    }
    
    // assume we get readings at at least 1Hz, otherwise reset quality to zero
    if (AP_HAL::millis() - state.last_reading_ms > 1000) {
        state.signal_quality = 0;
        state.rate_rpm = 0;
    }*/
}



