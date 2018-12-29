/*
 * AP_RangeFinder_HPS_167U.h
 *
 *  Created on: Aug 15, 2018
 *      Author: coco
 */

#ifndef LIBRARIES_AP_RANGEFINDER_AP_RANGEFINDER_HPS_167UL_H_
#define LIBRARIES_AP_RANGEFINDER_AP_RANGEFINDER_HPS_167UL_H_


#include "RangeFinder.h"
#include "RangeFinder_Backend.h"



typedef enum HPS167_Status
{
	HPS167_IDLE     = 0,
	HPS167_GOT_START = 1,
	HPS167_GOT_LENGTH,
	HPS167_GOT_DATA,
	HPS167_GOT_CRC1
}HPS167_parse_status;



class AP_RangeFinder_HPS_167UL : public AP_RangeFinder_Backend
{
public:

	//构造函数
	AP_RangeFinder_HPS_167UL(RangeFinder::RangeFinder_State &_state,  AP_SerialManager &serial_manager, uint8_t serial_instance);


    // 芯片识别函数
    static bool detect(  AP_SerialManager &serial_manager,uint8_t serial_instance);

    // 数据更新函数
    void update(void);

    //设置选择哪种模式，设置1是连续，设置0是单次
    int8_t set_flag_precision;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
	{
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }


private:
      bool get_reading(uint16_t &reading_cm);
      uint16_t crc_cal_by_halfbyte(uint8_t *ptr, uint8_t len);

      HPS167_parse_status _parse_status;
      AP_HAL::UARTDriver *uart = nullptr;
      uint32_t last_reading_ms = 0;
      uint8_t _measure[10];
      uint16_t crc_ta_4[16];

};



#endif /* LIBRARIES_AP_RANGEFINDER_AP_RANGEFINDER_HPS_167UL_H_ */
