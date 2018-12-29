#ifndef __AP_RANGEFINDER_NRA24_H__
#define __AP_RANGEFINDER_NRA24_H__
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
//#include <Filter.h>

typedef enum NRA24_Status
{
	NRA24_IDLE     = 0,
	NRA24_GOT_START1 = 1,
	NRA24_GOT_START2,
	NRA24_GOT_MSGID1,
	NRA24_GOT_MSGID2,
	NRA24_GOT_DATA,
	NRA24_GOT_END1
}NRA24_parse_status;


class AP_RangeFinder_NRA24 : public AP_RangeFinder_Backend
{
public:

    // constructor

	AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State &_state,  AP_SerialManager &serial_manager, uint8_t serial_instance);
    // destructor
    //~AP_RangeFinder_SK20(void);
    // static detection function
	static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);


    // update state
    void update(void);
protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
	{
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    // start a reading
    //static bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::UARTDriver *uart = nullptr;
    //uint16_t crc_cal_by_halfbyte(uint8_t *ptr, uint8_t len);
    uint32_t last_reading_ms = 0;
    uint8_t linebuf[8];
    uint8_t linebuf_len;

    uint8_t snr;
    uint8_t rcs;
    uint16_t dist;
    //uint8_t _measure[10];
    NRA24_parse_status _parse_status;

};


#endif /* AP_RANGEFINDER_NRA24_H_ */
