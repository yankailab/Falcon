#include "falconcommon.h"
#include "IMU.h"
#include "PPMInput.h"
#include <Wire.h>
#include "LIDARLite.h"
#include "VehicleLink.h"

#define RC_CHANNEL_NUM	8				//set the number of chanels
#define PPM_ON_STATE	0				//set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN	A7				//5 for arduino pro micro,set PPM signal output pin on the arduino
#define PPM_INPUT_PIN 7

struct LIDAR_UNIT
{
	long m_distCM;
	long m_lockCM;
	unsigned char m_address;
	uint8_t	m_pinEN;

	uint8_t m_P;
	uint8_t m_I;
	uint8_t m_D;
};


class DEVICE_LIDAR_LOCKER_2560
{
public:
	void deviceSetup(config_t* pConfig);
	void deviceLoop();

	void collisionAvoid();

public:
	HardwareSerial* m_pRFSerial;
	HardwareSerial* m_pUSBSerial;

	IMU m_IMU;
	LIDARLite m_LidarLite;
	LIDAR_UNIT m_pLidar[6];
	PPMInput m_PPMInput;

	//Common classes
	VehicleLink m_VLink;
	config_t* m_pConfig;

	//Program
	int m_counter = 0;

	//switches
	bool m_bPrintIMU;
	bool m_bBootSuccess;
	bool m_bHostConnected;

	//General
	uint8_t m_opeMode;
	bool m_bMpuInterrupt;

};

