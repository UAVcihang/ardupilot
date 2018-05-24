/*
 * AC_MQTT.cpp
 *
 *  Created on: 2017-12-27
 *      Author: liwh1
 */

//#include "mbed.h"
#include "AC_MQTT.h"

extern const AP_HAL::HAL& hal;

AC_MQTT::AC_MQTT(const AP_AHRS *ahrs):
_ahrs(ahrs),
client(ethernet)
{
	//client = MQTT::Client<MQTTGPRSEthernet, Countdown>(ethernet);
	message.qos = MQTT::QOS0;
	message.retained = false;
	message.dup = false;

	connected = false;

	lastTimeGpsReceived_ms = 0;
	//message.payload = (void*)buf;
	//message.payloadlen = strlen(buf) + 1;
}

void AC_MQTT::init(AP_SerialManager& _serial_manager)
{
	ethernet.init(_serial_manager);
	/// To Do
	/// init net



	/// connect host port
	/*for (int i = 0; i < INIT_MAX_TIME; i++)
	{
		if (ethernet.connect(hostname, port))
			break;
		else
			hal.console->printf("Can not connect to server!\n");
			//LOG("CAN NOT CONNECT TO SERVER!\r\n");
	}

	//// connect MQTT servers
	//char* topic = "uav-gps";
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.MQTTVersion = 3;
	data.clientID.cstring = "weihli";

	for (int i = 0; i < INIT_MAX_TIME; i++)
	{
		if (client.connect(data) != 0)
		//无法连接到MQTT服务器
			hal.console->printf("Can not connect to MQTT server!\n");
		else
			break;
	}*/
}

void AC_MQTT::update(){
	// publish topic to MQTT servers
	//char *buf;
	char buf[100];
	uint8_t heart[2];
	if(!ethernet.isOK()){
		ethernet.initNet();
		return;
		//return;
	}

	if(!connected) {
		MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
		data.MQTTVersion = 3;
		data.clientID.cstring = "cihang_uav";
		//data.username.cstring = "testuser";
		//data.password.cstring = "testpassword";
		data.keepAliveInterval = 40;
		data.willFlag = 1;
		data.will.topicName.cstring = "lwt";
		//data.will.topicName.lenstring = 3;
		data.will.message.cstring = "cihang_uav died";
		data.cleansession = 1;


		hal.console->printf("connect mqtt\n");
		// connceted MQTT servers
		if(client.connect(data) == 0){
			connected = true;
			hal.console->printf("connect mqtt success\n");
		}
		return;
	}

	// send 0xc0 0x00
	if(!sendflag){
		//ethernet.sendHeart();
		heart[0] = 0xc0;
		heart[1] = 0x00;
		ethernet.write(heart, 2, 2000);
		sendflag = true;
	}
	/*buf = json.getOutPut();
	message.payload = (void*)buf;
	message.payloadlen = strlen(buf) + 1;
	client.publish("uav-gps", message);*/
	else{
		// only publish mqtt topic when gps 3d fix
	    if (_ahrs->get_gps().last_message_time_ms() != lastTimeGpsReceived_ms) {
	        if (_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
	        	lastTimeGpsReceived_ms = _ahrs->get_gps().last_message_time_ms();

	        	hal.console->printf("mqtt publish\n");
	        	const struct Location &gpsloc = _ahrs->get_gps().location();
	        	Vector3f curr_vel;
	        	_ahrs->get_velocity_NED(curr_vel);
	        	//my_vel.length();
	        	float vel = norm(curr_vel.x, curr_vel.y);
	        	//sprintf(buf, "test");
	        	sprintf(buf, "trace=p&u=1&g=%.6f,%.6f&c=%.1f&ts=%d&s=%.2f", gpsloc.lng * 1e-6, gpsloc.lat * 1e-6, _ahrs->yaw_sensor * 1e-2, _ahrs->get_gps().time_epoch_sec(), vel);
	        	message.qos = MQTT::QOS0;
	        	message.retained = false;
	        	message.dup = false;
	        	message.payload = (void*)buf;
	        	message.payloadlen = strlen(buf) + 1;
	        	client.publish("uav-gps", message);

	        }
	    }
	    sendflag = false;
	}
}

/*#define APN         "uninet"
#define USERNAME    NULL
#define PASSWORD    NULL
#define INIT_TIMES  5            //初始化次数

#ifdef DEBUG
Serial pc2(PB_10, PB_11, 115200);
#define LOG(args...)    pc2.printf(args)
#else
#define LOG(args...)    (args)
#endif // DEBUG

#include "MQTTClient.h"
#include "Countdown.h"
#include "MQTTGPRSEthernet.h"
#include "StringHelper.h"

int arrivedcount = 0;

void messageArrived(MQTT::MessageData& md)
{
	MQTT::Message &message = md.message;
	LOG("Message arrived: qos %d, retained %d, dup %d, packetid %d\n", message.qos, message.retained, message.dup, message.id);
	LOG("Payload %.*s\n", message.payloadlen, (char*)message.payload);
	++arrivedcount;
}

int main(int argc, char* argv[])
{
	MQTTGPRSEthernet ethernet(PB_6, PB_7);
	for (int i = 0; i < INIT_TIMES; i++)
	{
		if (ethernet.initNet(APN, USERNAME, PASSWORD))
			break;
		else
			LOG("SIM CARD INIT FAILED!\r\n");
	}

	MQTT::Client<MQTTGPRSEthernet, Countdown> client = MQTT::Client<MQTTGPRSEthernet, Countdown>(ethernet);

	char* hostname = "iot.eclipse.org";
	int port = 1883;
	for (int i = 0; i < INIT_TIMES; i++)
	{
		if (ethernet.connect(hostname, port))
			break;
		else
			LOG("CAN NOT CONNECT TO SERVER!\r\n");
	}

	char* topic = "tson-topic-18669888635";
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.MQTTVersion = 3;
	data.clientID.cstring = "tson-client-18669888635";
	int rc;
	for (int i = 0; i < INIT_TIMES; i++)
	{
		if ((rc = client.connect(data)) != 0)
		//无法连接到MQTT服务器
			LOG("CAN NOT CONNECT TO MQTT SERVER!\r\n");
		else
			break;
	}

	if ((rc = client.subscribe(topic, MQTT::QOS0, messageArrived)) != 0)
		//无法订阅
		LOG("CAN NOT SUBSCRIBE THE TOPIC!\r\n");

	MQTT::Message message;

	   // QoS 0
	char buf[100];
	sprintf(buf, "Hello World!  QoS 0 message from tson\r\n");
	message.qos = MQTT::QOS0;
	message.retained = false;
	message.dup = false;
	message.payload = (void*)buf;
	message.payloadlen = strlen(buf) + 1;
	rc = client.publish(topic, message);
	while (arrivedcount == 0)
		client.yield(100000);

	// QoS 1
	sprintf(buf, "Hello World!  QoS 1 message from tson\n");
	message.qos = MQTT::QOS1;
	message.payloadlen = strlen(buf) + 1;
	rc = client.publish(topic, message);
	while (arrivedcount == 1)
		client.yield(100000);

	// QoS 2
	sprintf(buf, "Hello World!  QoS 2 message from tson\n");
	message.qos = MQTT::QOS2;
	message.payloadlen = strlen(buf) + 1;
	rc = client.publish(topic, message);
	while (arrivedcount == 2)
		client.yield(100000);

	if ((rc = client.unsubscribe(topic)) != 0)
		printf("rc from unsubscribe was %d\n", rc);

	if ((rc = client.disconnect()) != 0)
		printf("rc from disconnect was %d\n", rc);

	ethernet.disconnect();
	return 0;
}*/


