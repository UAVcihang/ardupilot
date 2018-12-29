
/********************************************************************************************************************************
*头  文件： NRA24驱动文件
*作  者 ： uavcihang
*修改日期： 2018-9-6
*备   注：
**********************************************************************************************************************************/


#include "AP_RangeFinder_NRA24.h"
#include "AP_RangeFinder_LightWareSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL &hal;
/*******************************************************************************************************************************
*函数原型：AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State &_state,  AP_SerialManager &serial_manager,
	 uint8_t serial_instance):AP_RangeFinder_Backend( _state)
*函数功能：任务组
*修改日期：2018-9-6
*备   注：
********************************************************************************************************************************/
AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State &_state,  AP_SerialManager &serial_manager, uint8_t serial_instance):
	AP_RangeFinder_Backend( _state)
{

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24, serial_instance);
    if (uart != nullptr)
    {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24, serial_instance));
    }
	 _parse_status = NRA24_IDLE;
	 linebuf_len = 0;
	 snr = 0;

}

/****************************************************************************************************************
*函数原型：bool AP_RangeFinder_NRA24::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
*函数功能：任务组
*修改日期：2018-9-6
*备   注：识别传感器
*****************************************************************************************************************/
bool AP_RangeFinder_NRA24::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
	hal.console->printf("AP_RangeFinder_NRA24\n\r");

	return serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24, serial_instance) != nullptr;
}

/****************************************************************************************************************
*函数原型：bool AP_RangeFinder_NRA24::get_reading(uint16_t &reading_cm)
*函数功能：获取数据
*修改日期：2018-9-6
*备   注：read - return last value measured by sensor
*****************************************************************************************************************/

bool AP_RangeFinder_NRA24::get_reading(uint16_t &reading_cm)
{

	uint32_t sum = 0;
	uint16_t count = 0;


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
        	if(linebuf_len == 8)
        	{
        		dist = (uint16_t)(linebuf[2] * 0x100 + linebuf[3]);
        		snr = linebuf[7] - 127;
        		rcs = linebuf[1] * 0.5 - 50;

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
        	if(c == 0x55)
        	{
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

    if (count == 0)
    {

        return false;
    }
    reading_cm = dist;
    return true;
}

/****************************************************************************************************************
*函数原型：void AP_RangeFinder_NRA24::update(void)
*函数功能：NRA24函数更新
*修改日期：2018-9-6
*备   注：update the state of the sensor
*****************************************************************************************************************/

void AP_RangeFinder_NRA24::update(void)
{
    if (get_reading(state.distance_cm))
    {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200)
    {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}



/*************************************************************************************************************************
*                              file_end
*************************************************************************************************************************/
