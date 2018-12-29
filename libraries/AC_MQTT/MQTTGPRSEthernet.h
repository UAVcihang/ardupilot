#pragma once
#if !defined(MQTTGPRSETHERNET_H)
#define MQTTGPRSETHERNET_H

#define DEFAULT_GPRS_TIMEOUT 6000000  //6s
#define SERIAL_BUFFER_SIZE 256

#include <AP_SerialManager/AP_SerialManager.h>


/****UL865的AT指令*****/
#define ACK_OK "OK"
#define ACK_T ">"

#define AT_CGREG "AT+CGREG?"  //原来的CREG只能检测有没有sim卡，换成CGREG，检测有没有注册GPRS网络
#define ACK_CGREG "+CGREG: 0,1"

#define AT_CGATT "AT+CGATT=1"
#define AT_GPRS  "AT#GPRS=1"
#define AT_SCFG  "AT#SCFG=1,1,300,90,600,50" // 配制 socket
#define AT_SD    "AT#SD=1,0,1883,\"120.132.101.145\",0,0,0"//"AT#SD=1,0,1883,\"m2m.eclipse.org\",0,0,0"
#define AT_SSEND "AT#SSEND=1"
#define ACK_CONNECT "CONNECT"

//#include "mbed.h"

class MQTTGPRSEthernet
{
public:
	MQTTGPRSEthernet();
	~MQTTGPRSEthernet();
	void init(AP_SerialManager &serial_manager);
	bool initNet(/*const char* apn, const char* userName = "", const char* passWord = "", int timeout = DEFAULT_GPRS_TIMEOUT, bool isReconnect = false*/);
	bool connect(const char* hostname, uint32_t port, uint32_t timeout = DEFAULT_GPRS_TIMEOUT);
	int read_line(char* buffer, uint32_t timeout = DEFAULT_GPRS_TIMEOUT);
	int read(unsigned char* buffer, int len, uint32_t timeout = DEFAULT_GPRS_TIMEOUT);
	int write(unsigned char* buffer, int len, uint32_t timeout = DEFAULT_GPRS_TIMEOUT);

	//int sendHeart(void);
	bool disconnect();
	bool isOK();
private:
	//bool initNet();
	AP_HAL::UARTDriver *uart = nullptr;
	// Serial eth;
	//bool command(const char* cmd, const char* ack = "");
	bool recACK(const char* at_ack);
	void sendAT(const char* cmd);
	bool connected = false;
	//bool initialized = false;
	char* localIP;
	const char *_apn;
	const char *_passWord;
	const char *_userName;

	uint32_t status;
	uint32_t initTime;
	bool ack;
};
#endif
