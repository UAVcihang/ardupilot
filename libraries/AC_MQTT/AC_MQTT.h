/*
 * AC_MQTT.h
 *
 *  Created on: 2017-12-28
 *      Author: liwh1
 */

#pragma once
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include "MQTTGPRSEthernet.h"
#include "MQTTClient.h"
#include "Countdown.h"
//#include "JsonInterface.h"

#define INIT_MAX_TIME  3
class AC_MQTT
{

public:

    // constructor
	AC_MQTT(const AP_AHRS *ahrs);

    // static detection function
    // static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

    void init(AP_SerialManager& _serial_manager);


    // parameter list
    // static const struct AP_Param::GroupInfo var_info[];
private:

    //AP_SerialManager &serial_manager;
    MQTTGPRSEthernet ethernet;
    MQTT::Client<MQTTGPRSEthernet, Countdown> client;
    MQTT::Message message;

    const AP_AHRS *_ahrs;
    //JSONInterface json;
    uint32_t port;
    const char *hostname;

    bool connected = false;
    bool sendflag = false;

    uint32_t lastTimeGpsReceived_ms;// last time we received GPS data
    //Tempt_State state;
    //AP_HAL::UARTDriver *uart = nullptr;
};
