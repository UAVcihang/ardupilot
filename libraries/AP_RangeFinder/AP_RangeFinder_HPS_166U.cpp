/************************************************************************************************************************************************************************************************
*头  文件：ArduCopter函数
*函数功能：任务组
*修改日期：2018-5-4
*备   注：连续模式精度参数precision，这个值比较小，但是非连续模式参数precision较大
*        通过设定set_flag_precision变量来选择是否引进连续模式和非连续测量模式
*************************************************************************************************************************************************************************************************/

#include "AP_RangeFinder_HPS_166U.h"
#include "AP_RangeFinder_LightWareSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
extern const AP_HAL::HAL &hal;



/**********************************************************************************************************************************************
*函数原型：AP_RangeFinder_HPS_166U::AP_RangeFinder_HPS_166U(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
		 AP_SerialManager &serial_manager):AP_RangeFinder_Backend(_ranger, instance, _state)
*函数功能：
*修改日期：2018-8-16
*备   注：
***********************************************************************************************************************************************/
AP_RangeFinder_HPS_166U::AP_RangeFinder_HPS_166U(RangeFinder::RangeFinder_State &_state,  AP_SerialManager &serial_manager, uint8_t serial_instance):
	AP_RangeFinder_Backend( _state)
{

	 uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_HPS166U, serial_instance);

//     if(_state.select_status==1)
//     {
//    	   //连续测量命令
//    	 _measure[0] = 0x0A;
//    	 _measure[1] = 0x24;
//    	 _measure[2] = 0x00;
//    	 _measure[3] = 0x00;
//    	 _measure[4] = 0x00;
//    	 _measure[5] = 0x00;
//    	 _measure[6] = 0x00;
//    	 _measure[7] = 0x00;
//    	 _measure[8] = 0x0F;
//    	 _measure[9] = 0x72;
//    	 set_flag_precision=1;
//
//     }
//     else
//     {
    	    //单次测量命令
		 _measure[0] = 0x0A;
		 _measure[1] = 0x22;
		 _measure[2] = 0x00;
		 _measure[3] = 0x00;
		 _measure[4] = 0x00;
		 _measure[5] = 0x00;
		 _measure[6] = 0x00;
		 _measure[7] = 0x00;
		 _measure[8] = 0xAE;
		 _measure[9] = 0x57;
		 set_flag_precision=0;
//     }
	 //半字节校验
	 crc_ta_4[0] = 0x0000;
	 crc_ta_4[1] = 0x1021;
	 crc_ta_4[2] = 0x2042;
	 crc_ta_4[3] = 0x3063;
	 crc_ta_4[4] = 0x4084;
	 crc_ta_4[5] = 0x50a5;
	 crc_ta_4[6] = 0x60c6;
	 crc_ta_4[7] = 0x70e7;
	 crc_ta_4[8] = 0x8108;
	 crc_ta_4[9] = 0x9129;
	 crc_ta_4[10] = 0xa14a;
	 crc_ta_4[11] = 0xb16b;
	 crc_ta_4[12] = 0xc18c;
	 crc_ta_4[13] = 0xd1ad;
	 crc_ta_4[14] = 0xe1ce;
	 crc_ta_4[15] = 0xf1ef;
	 _parse_status = HPS166_IDLE;

    if (uart != nullptr)
    {
    	uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HPS166U, serial_instance));

    }

}

/****************************************************************************************************************************
*函数原型：bool AP_RangeFinder_HPS_166U::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
*函数功能：
*修改日期：2018-8-16
*备   注：
*****************************************************************************************************************************/
bool AP_RangeFinder_HPS_166U::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
	hal.console->printf("AP_RangeFinder_HPS_166UL\n\r");

	return serial_manager.find_serial(AP_SerialManager::SerialProtocol_HPS166U, serial_instance) != nullptr;

}

/****************************************************************************************************************************
*函数原型：bool AP_RangeFinder_HPS_166U::get_reading(uint16_t &reading_cm)
*函数功能：获取传感器数据
*修改日期：2018-8-16
*备   注：返回1，获取数据成功，返回0获取失败。
*****************************************************************************************************************************/
bool AP_RangeFinder_HPS_166U::get_reading(uint16_t &reading_cm)
{
    uint8_t data[11]; //去掉校验的数据
    uint16_t checksum=0, precision = 0;
    uint8_t idx= 0;
    uint16_t dist = 0;
    int16_t nbytes = uart->available(); //获取当前串口缓冲区的数据长度
	if(nbytes == 0)
	{

		uart->write(_measure,sizeof(_measure));//写入命令sizeof(_measure)=10;
		return false;
	}

	while(nbytes-- >0)
	{

	        uint8_t c = uart->read();
	        switch(_parse_status)
	        {
	        case HPS166_IDLE:
	        	if(c == 0x0a)

	        		_parse_status = HPS166_GOT_START;
	        	break;
	        case HPS166_GOT_START:
	        	if(c == 0x0d)
	        	{

	        		_parse_status = HPS166_GOT_LENGTH;
	        		idx = 0;
	        	}
	        	else
	        		_parse_status = HPS166_IDLE;
	        	break;

	        case HPS166_GOT_LENGTH: //2-12     2-11    2-10,    2-9,    2-8,   2-7,    2-6,   2-5,    2-4,    2-3,     2-2 , 2-1
	        	data[idx++] = c;    //data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[2],data[9],data[10]

	        	if(idx == 11)
	        	{

	        		checksum = crc_cal_by_halfbyte(data, 11); //获取校验结果
	        		_parse_status = HPS166_GOT_DATA;
	        	}
	        	break;

	        case HPS166_GOT_DATA:
	        	if(c != (uint8_t)(checksum >> 8))
	        	{
	        		_parse_status = HPS166_IDLE;
	        	}
	        	else

	        		_parse_status = HPS166_GOT_CRC1;
	        	break;

	        case HPS166_GOT_CRC1:
	        	if(c != (uint8_t)(checksum & 0xff))
	        	{
        		   _parse_status = HPS166_IDLE;
	        	}

	        	else
	        	{

	        		dist = (uint16_t)((data[3] * 256 + data[4]) * 0.1);

	        		precision = (uint16_t)(data[9] * 256 + data[10]);

	        		_parse_status = HPS166_IDLE;

	        	}
	        	break;
	        }
		}

    reading_cm = dist;
    uart->write(_measure,10);//写入命令

     if(set_flag_precision)
     {

		 if (precision >100)
		 {

			 return false;
		 }

     }

    return true;
}


/****************************************************************************************************************
*函数原型：void AP_RangeFinder_HPS_166U::update(void)
*函数功能：数据更新函数
*修改日期：2018-8-16
*备   注：
*****************************************************************************************************************/

void AP_RangeFinder_HPS_166U::update(void)
{
    if (get_reading(state.distance_cm))
    {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    }
    else if (AP_HAL::millis() - last_reading_ms > 200)
    {
        set_status(RangeFinder::RangeFinder_NoData);
    }

}



/****************************************************************************************************************
*函数原型：uint16_t AP_RangeFinder_HPS_166U::CRC16_Cal(uint8_t* ptr, uint8_t len)
*函数功能：计算数据包CRC16 CCITT校验码
*修改日期：2018-8-16
*备   注：1、ptr：数据包首地址；2、len：数据包要参与计算的数据字节数；返回数据包的 CRC16 值
*****************************************************************************************************************/

uint16_t AP_RangeFinder_HPS_166U::crc_cal_by_halfbyte(uint8_t* ptr, uint8_t len)
{
	   uint16_t crc = 0xffff;

		while(len-- !=0)
		{
			uint8_t high = (uint8_t)(crc>>12);//crc / 4096;
			crc <<= 4;
			crc ^= crc_ta_4[high ^ (*ptr >> 4)];

			high = (uint8_t)(crc>>12);
			crc <<=4;
			crc ^= crc_ta_4[high ^ (*ptr & 0x0f)];
			ptr++;
		}

		return crc;
}


/*************************************************************************************************************************
*                              file_end
*************************************************************************************************************************/
