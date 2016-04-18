#include "falconcommon.h"
#include "VehicleLink.h"

class DEVICE_UART_RC_BRIDGE
{
public:
	void deviceSetup();
	void deviceLoop();

public:
	//Common classes
	VehicleLink m_VLink;
//	config_t m_config;

	//Program
	int m_counter;

	//switches
	bool m_bPrintIMU;
	bool m_bBootSuccess;
	bool m_bHostConnected;

#ifndef ATMEGA_A328
	Serial_* m_pUSBSerial;
#else
	HardwareSerial* m_pUSBSerial;
#endif

};

