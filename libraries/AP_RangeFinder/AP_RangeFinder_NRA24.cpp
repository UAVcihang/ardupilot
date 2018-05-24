/*
 * AP_RangeFinder_NRA24.cpp
 *
 *  Created on: 2017-5-16
 *      Author: weihli
 */
#include "AP_RangeFinder_NRA24.h"
//#include <AP_HAL.h>
#include <ctype.h>

// extern const AP_HAL::HAL& hal;

AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_SerialManager &serial_manager/*float cut_freq*/):
	AP_RangeFinder_Backend(_ranger, instance, _state)
{
	 /*hal.uartE->begin(115200UL, 256, 256);
	 hal.uartE->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);*/
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24, 0));
    }
	 _parse_status = NRA24_IDLE;
	 linebuf_len = 0;

	 snr = 0;
	 //_dist_filted_cm.set_cutoff_frequency(0.1f, cut_freq);
}


bool AP_RangeFinder_NRA24::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    /*if (!start_reading()) {
        return false;
    }
    // give time for the sensor to process the request
    hal.scheduler->delay(50);
    uint16_t reading_cm;
    return get_reading(reading_cm);*/
	return serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24, 0) != nullptr;
}


// read - return last value measured by sensor
bool AP_RangeFinder_NRA24::get_reading(uint16_t &reading_cm)
{

	uint32_t sum = 0;
	uint16_t count = 0;
	//uint16_t dist = 0;
	//AP_HAL::UARTDriver *port = hal.uartE;

    if (uart == nullptr) {
        return false;
    }

	int16_t nbytes = uart->available();

	if(nbytes == 0) return false;

	while(nbytes-- >0) {
        uint8_t c = uart->read();
        switch(_parse_status) {
        case NRA24_IDLE:
        	if(c == 0xaa)
        		_parse_status = NRA24_GOT_START1;
        	break;

        case NRA24_GOT_START1:
        	if(c == 0xaa)
        		_parse_status = NRA24_GOT_START2;
        	else
        		_parse_status = NRA24_IDLE;
        	break;

        case NRA24_GOT_START2:
        	if(c == 0x0c)
        		_parse_status = NRA24_GOT_MSGID1;
        	else if(c == 0xaa)
        		_parse_status = NRA24_GOT_START1;
        	else
        		_parse_status = NRA24_IDLE;
        	break;

        case NRA24_GOT_MSGID1:
        	if(c == 0x07){
        		_parse_status = NRA24_GOT_MSGID2;
        		linebuf_len = 0;
        	}
        	else if(c == 0xaa)
        		_parse_status = NRA24_GOT_START1;
        	else
        		_parse_status = NRA24_IDLE;
        	break;

        case NRA24_GOT_MSGID2:
        	linebuf[linebuf_len++] = c;
        	if(linebuf_len == 8){
        		dist = (uint16_t)(linebuf[2] * 0x100 + linebuf[3]);
        		snr = linebuf[7] - 127;
        		rcs = linebuf[1] * 0.5 - 50;
        		//count++;
        		_parse_status = NRA24_GOT_DATA;
        	}
        	break;

        case NRA24_GOT_DATA:
        	if(c == 0x55)
        		_parse_status = NRA24_GOT_END1;
        	else if(c == 0xaa)
        		_parse_status = NRA24_GOT_START1;
        	else
        		_parse_status = NRA24_IDLE;
        	break;

        case NRA24_GOT_END1:
        	if(c == 0x55){
        		_parse_status = NRA24_IDLE;
        		sum += dist;
        		count++;
        	}
        	else if(c == 0xaa)
        		_parse_status = NRA24_GOT_START1;
        	else
        		_parse_status = NRA24_IDLE;
        	break;
        }

	}

	// 没有高度信息则保持上次数据
    if (count == 0) {
    	//_dist_filted_cm.reset(0);
        return false;
    }

    reading_cm = (uint16_t)(sum / count);
    /*dist = (uint16_t)(sum / count);
    reading_cm = _dist_filted_cm.apply(dist);*/
    /*if(dist == 0)
    	dist = reading_cm;
    reading_cm = _dist_filted_cm.apply(dist);*/
    // reading_cm = dist;//sum / count;
    return true;
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_NRA24::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
