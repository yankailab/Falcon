#pragma once

#include "falconcommon.h"
//#include <SoftwareSerial.h>
#include "variables.h"


struct CMD_STREAM
{
	uint8_t m_cmd;
	uint8_t m_payloadLen;
	uint16_t m_iByte;
	uint8_t m_pBuf[CMD_BUF_LEN];
};

class VehicleLink
{
public:
	VehicleLink(void);
	~VehicleLink(void);

	void init(void);
	void sendRFupdateAttitude(void);
	void receiveFromRF(void);

	bool receiveFromHost(void);
	void hostCommand(void);
	void sendHostHeartBeat(void);

	void vehicleCommand(void);

#ifdef LIDAR_CONTROLLER
	Serial_* m_pHostSerial;
	CMD_STREAM m_uartCMD;
	HardwareSerial* m_pUartSerial;

	bool receiveFromUART(void);
	void uartCommand(void);
	void sendUartHeartBeat(void);
#endif

#ifdef RC_TRANSMITTER
//	Serial_* m_pHostSerial;
	HardwareSerial* m_pHostSerial;
#endif

#ifdef UART_RC_BRIDGE
	Serial_* m_pHostSerial;
	CMD_STREAM m_uartCMD;
	HardwareSerial* m_pUartSerial;

	bool receiveFromUART(void);
	void uartCommand(void);
	void sendUartHeartBeat(void);
#endif

#ifdef SERIAL_RECEIVER
	SoftwareSerial* m_pHostSerial;
#endif

	HardwareSerial* m_pRFSerial;
	config_t* m_pConfig;

	uint8_t* m_pOprMode;
	byte m_numChannel;
	int16_t m_channelValues[LINK_CHANNELS];

	CMD_STREAM m_hostCMD;
	CMD_STREAM m_rfCMD;

};

