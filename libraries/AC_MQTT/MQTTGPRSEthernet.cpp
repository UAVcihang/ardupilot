/*
 * MQTTGPRSEthernet.cpp
 *
 *  Created on: 2017-12-27
 *      Author: liwh1
 */
#include "MQTTGPRSEthernet.h"
//#include "StringHelper.h"

extern const AP_HAL::HAL& hal;

MQTTGPRSEthernet::MQTTGPRSEthernet()
{
	status = 0;
	ack = true;
	// initialized = false;
}

//还需要增加一个表示已经初始化成功的变量
//后续可能要增加表示6条连接的东东，如果host，port，连接状态等
void MQTTGPRSEthernet::init(AP_SerialManager &serial_manager)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MQTT, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_MQTT, 0));
    }
}
bool MQTTGPRSEthernet::initNet()
{
	if(uart == nullptr)
		return false;

	switch(status) {
	case 0:
		initTime = AP_HAL::millis();
		status++;
		break;
	case 1:
		// wait 3 second for ul865 initial
		if((AP_HAL::millis() - initTime) > 3000)
		{
			status++;
		}
		break;

	case 2:
		if(ack) {
			// send CGREG AT command
			sendAT(AT_CGREG);
			hal.console->printf("send %s\n", AT_CGREG);
			ack = false;
			initTime = AP_HAL::millis();
		}
		else
		{
			// receive ack
			ack = recACK(ACK_CGREG);
			// receive correct ack then step into next status
			if(ack)
				status++;
			else if((AP_HAL::millis() - initTime) > 10000) {
				// 10 second no ack then send at command once more
				ack = true;
			}
		}
		break;

	case 3:
		if(ack) {
			sendAT(AT_CGATT);
			hal.console->printf("send %s\n", AT_CGATT);
			ack = false;
			initTime = AP_HAL::millis();
		}
		else {
			// receive ack
			ack = recACK(ACK_OK);
			// receive correct ack then step into next status
			if(ack)
				status++;
			else if((AP_HAL::millis() - initTime) > 10000) {
				// 10 second no ack then send at command once more
				ack = true;
			}
		}
		break;

	case 4:
		if(ack) {
			sendAT(AT_GPRS);
			hal.console->printf("send %s\n", AT_GPRS);
			ack = false;
			initTime = AP_HAL::millis();
		}
		else {
			// receive ack
			ack = recACK(ACK_OK);
			// receive correct ack then step into next status
			if(ack)
				status++;
			else if((AP_HAL::millis() - initTime) > 10000) {
				// 10 second no ack then send at command once more
				ack = true;
			}
		}
		break;

	case 5:
		if(ack) {
			sendAT(AT_SCFG);
			hal.console->printf("send %s\n", AT_SCFG);
			ack = false;
			initTime = AP_HAL::millis();
		}
		else {
			// receive ack
			ack = recACK(ACK_OK);
			// receive correct ack then step into next status
			if(ack)
				status++;
			else if((AP_HAL::millis() - initTime) > 10000) {
				// 10 second no ack then send at command once more
				ack = true;
			}
		}
		break;

	case 6:
		if(ack) {
			sendAT(AT_SD);
			hal.console->printf("send %s\n", AT_SD);
			ack = false;
			initTime = AP_HAL::millis();
		}
		else {
			// receive ack
			ack = recACK(ACK_CONNECT);
			// receive correct ack then step into next status
			if(ack){
				status++;
				connected = true;
			}
			else if((AP_HAL::millis() - initTime) > 15000) {
				// 10 second no ack then send at command once more
				ack = true;
			}
		}
		break;

	case 7:
		/*if(ack) {
			sendAT(AT_SSEND);
			hal.console->printf("send %s\n", AT_SSEND);
			ack = false;
			initTime = AP_HAL::millis();
		}
		else {
			// receive ack
			ack = recACK(ACK_T);
			// receive correct ack then step into next status
			if(ack) {
				status++;
				connected = true;
			}
			else if((AP_HAL::millis() - initTime) > 10000) {
				// 10 second no ack then send at command once more
				ack = true;
			}
		}*/
		break;

	default :
		break;
	}

	if(status >=7)
		return true;
	return false;
	/*_apn = apn;
	_userName = userName;
	_passWord = passWord;
	command("ATE0");
	// wait_ms(800);
	command("AT+CIPMUX=0");
	// wait_ms(800);
	//检查 GPRS 附着状态
	while (!command("AT+CGATT?\r\n", "+CGATT: 1"))
	{
		//LOG("GPRS NOT ATTACHED!\r\n");
		//wait_ms(1200);
		uart->write("+++");
		//eth.printf("+++");
		//wait_ms(1200);
		command("AT");
		command("AT+CIPSHUT\r\n");
	}
	//Select multiple connection
	//单链路模式
	//command("AT+CIPMUX=0");
	//wait_ms(800);
	//透传模式
	command("AT+CIPMODE=1");
	//wait_ms(800);
	command("AT+CIPCCFG=5,2,1024,1,0,1460,50");
	//wait_ms(800);
	// Set APN
	command(StringHelper::Format("AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, userName, passWord));
	//wait_ms(800);
	//LOG(StringHelper::Format("AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", apn, userName, passWord));
	uint32_t start = AP_HAL::millis();//us_ticker_read();
	do
	{
		// Brings up wireless connection
		//建立无线链路(GPRS 或者 CSD)
		bool flag = command("AT+CIICR", "OK");
		// Get local IP address
		uart->write("AT+CIFSR\r\n");
		//eth.printf("AT+CIFSR\r\n");
		char ip_addr_buf[32];

		if (read_line(ip_addr_buf) <= 0) {
			//LOG("failed to join network\r\n");
			initialized = false;
		}
		else if (StringHelper::CheckIP(ip_addr_buf))
		{
			//LOG("IP ADDRESS:");
			//LOG(ip_addr_buf);
			//LOG("\r\n");
			localIP = ip_addr_buf;
			initialized = true;
			break;
		}
		else
		{
			initialized = false;
		}
		if (flag == false && initialized == false && isReconnect == false)
		{
			//表明此时建立无线链路返回error，得不到IP地址
			//此时应使用AT+CIPSHUT关闭PDP上下文后再重新进行连接
			//防止在数据状态
			//wait_ms(1200);
			uart->write("+++");
			//eth.printf("+++");
			//wait_ms(1200);
			command("AT");
			command("AT+CIPSHUT\r\n");
			initialized = false;
			return initNet(_apn, _userName, _passWord, timeout, true);
		}

	} while (AP_HAL::millis() - start < timeout);
	//command("AT+CRPRXGET=0");
	return initialized;*/

}
/*bool MQTTGPRSEthernet::initNet()
{
	return initNet(_apn, _userName, _passWord);
}*/

bool MQTTGPRSEthernet::isOK()
{
	if(uart == nullptr){
		return false;
	}

	return connected;
}

void MQTTGPRSEthernet::sendAT(const char *cmd) {
	if(uart == nullptr)
		return;

	uart->printf("%s\r\n", cmd);
}

bool MQTTGPRSEthernet::recACK(const char* at_ack)
{
	char buffer[128];
	uint16_t nbytes;

	//hal.console->printf("recACK\n");
	if(uart == nullptr)
		return false;

	nbytes = uart->available();

	// 必须要判断串口缓存的数据量，如果超过了buffer的大小 必须限定它，不然会出错
	if(nbytes > 128) {
		nbytes = 128;
	}
	if(nbytes > 0){

		/*hal.console->printf("receive %d bytes\n", nbytes);
		hal.console->flush();*/
		for(int i=0; i<nbytes; i++){
			*(buffer+i) = uart->read();
		}


		hal.console->printf("receive ack %s\n", buffer);
		if(strstr(buffer, at_ack) != nullptr){
			return true;
		}
	}

	return false;
}
/*bool MQTTGPRSEthernet::connect(const char* hostname, uint32_t port, uint32_t timeout)
{
	uint32_t start = AP_HAL::millis();//us_ticker_read();
	do
	{
		if (initialized == false)
			initNet();
		else
			break;
	} while (AP_HAL::millis() - start < timeout);
	start = AP_HAL::millis();
	do
	{
		//AT+CIPSTART=0,”TCP”,”116.228.221.51”,”8500”
		char strPort[10];
		itoa(port, strPort, 10);
		char response[64] = { 0, };
		int connectStart = AP_HAL::millis();
		//LOG(StringHelper::Format("AT+CIPSTART=TCP,%s,%s\r\n", hostname, strPort));
		uart->write(StringHelper::Format("AT+CIPSTART=TCP,%s,%s\r\n", hostname, strPort));
		//eth.printf(StringHelper::Format("AT+CIPSTART=TCP,%s,%s\r\n", hostname, strPort));

		do
		{
			read_line(response);
			if (strstr(response, "CONNECT") != NULL)
			{
				connected = true;
				return connected;
			}
		} while (AP_HAL::millis() - connectStart < timeout);
	} while (AP_HAL::millis() - start < timeout);
	connected = false;
	return connected;

}*/
//向SIM800C发送命令
//cmd:发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串.
//ack:期待的应答结果,如果为空,则表示不需要等待应答，对于那些并不关心返回结果的直接返回true
//此函数只能对应立即进行读取的情况。
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
/*bool MQTTGPRSEthernet::command(const char* cmd, const char* ack)
{
	char response[64] = { 0, };
	if (StringHelper::EndWith(cmd, "\r\n"))
	{
		//LOG(cmd);
		uart->write(cmd);
		//eth.printf(cmd);
	}
	else if (StringHelper::EndWith(cmd, "\r"))
	{
		//LOG(StringHelper::Add(cmd, "\n"));
		uart->write(StringHelper::Add(cmd, "\n"));
		//eth.printf(StringHelper::Add(cmd, "\n"));
	}
	else
	{
		//LOG(StringHelper::Add(cmd, "\r\n"));
		uart->write(StringHelper::Add(cmd, "\r\n"));
		//eth.printf(StringHelper::Add(cmd, "\r\n"));
	}
	read_line(response);

	if (strstr(response, ack) != NULL) {
		return true;
	}
	return false;
}*/
int MQTTGPRSEthernet::read_line(char* buffer, uint32_t timeout)
{
	int bytes = 0;
	uint32_t start = AP_HAL::millis();

	while (true) {
		if (uart->available() > 0) {
			char ch = uart->read();
			if ((ch == '\n' || ch == '\r') && bytes == 0)
			{
				//此时说明以\r\n开头，在无回显的情况下经常会出现
				//此时忽略空行\r\n
				continue;
			}
			if (ch == '\n') {
				if (bytes > 0 && buffer[bytes - 1] == '\r') {
					//表明读取到\r\n，此行结束
					bytes--;
				}
				if (bytes > 0) {
					buffer[bytes] = '\0';
					return bytes;
				}
			}
			else {
				buffer[bytes] = ch;
				bytes++;
			}
		}
		else {
			if ((uint32_t)(AP_HAL::millis() - start) > timeout) {
				return bytes;
			}
		}
	}
	//此时表示读到最后一个字节还没有读到\n
	//有两种情况，一种是len不够长，另一种是恰好读完，此时数组的末尾没有\0
	return bytes;
}
int MQTTGPRSEthernet::read(unsigned char* buffer, int len, uint32_t timeout)
{
	int bytes = 0;
	uint32_t start = AP_HAL::millis();

	while (bytes < len) {
		if (uart->available() > 0) {
			//hal.console->printf("read func: ");
			char ch = uart->read();
			buffer[bytes] = ch;
			bytes++;
			hal.console->printf("read %02x\n", ch);
		}
		else {
			if ((uint32_t)(AP_HAL::millis() - start) > timeout) {
				return bytes;
			}
		}
	}

	/*bytes = uart->available();
	if(bytes>0){
		hal.console->printf("read func: ");
		for(int i=0; i<bytes; i++)
		{
			char ch = uart->read();
			buffer[i] = ch;
			hal.console->printf("%02x ", ch);
		}

	}*/
	//此时表示读到最后一个字节还没有读到\n
	//有两种情况，一种是len不够长，另一种是恰好读完，此时数组的末尾没有\0
	return bytes;
}

int MQTTGPRSEthernet::write(unsigned char* buffer, int len, uint32_t timeout)
{
	uint32_t start = AP_HAL::millis();
	while (!connected)
	{
		if ((AP_HAL::millis() - start) > timeout)
		{
			return -1;
		}
		initNet();
	}
	//hal.console->printf("mqtt write: ");
	for (int i = 0; i < len; i++)
	{
		//hal.console->printf("%02x ", buffer[i]);
		uart->write(buffer[i]);
		//eth.putc(buffer[i]);
	}
	//hal.console->printf("mqtt write %s\n", buffer);
	//uart->write(buffer, len);
	return len;
}

/*int MQTTGPRSEthernet::sendHeart(void)
{
	uart->write(0xc0);
	uart->write(0x00);
}*/

bool MQTTGPRSEthernet::disconnect()
{
	//wait_ms(1200);
	uart->write("+++");
	//eth.printf("+++");
	//wait_ms(1200);
	uart->write("AT\r\n");
	//uart->write("AT+CIPSHUP\r\n");
	//eth.printf("AT\r\n");
	//eth.printf("AT+CIPSHUT\r\n");
	connected = false;
	return true;
}

MQTTGPRSEthernet::~MQTTGPRSEthernet()
{
	if (connected)
	{
		disconnect();
	}
}



