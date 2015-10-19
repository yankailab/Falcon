#include "falconcommon.h"
#include "IMU.h"
#include "PPMInput.h"
#include <Wire.h>
#include "LIDARLite.h"
#include "VehicleLink.h"



struct LIDAR_UNIT
{
	long m_distCM;
	long m_lockCM;
	long m_prevErr;
	long m_integErr;

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

	void referenceLock();
	void collisionAvoid();

public:
	HardwareSerial* m_pRFSerial;
	HardwareSerial* m_pUSBSerial;

	IMU m_IMU;
	PPMInput m_PPMInput;

	LIDARLite m_LidarLite;
	LIDAR_UNIT m_pLidar[6];
	LIDAR_UNIT* m_pLidarUP;
	LIDAR_UNIT* m_pLidarL;
	LIDAR_UNIT* m_pLidarR;

	//Common classes
	VehicleLink m_VLink;
	config_t* m_pConfig;

	//Program
	int m_counter;

	//switches
	bool m_bPrintIMU;
	bool m_bBootSuccess;
	bool m_bHostConnected;

	//General
	uint8_t m_opeMode;
	bool m_bMpuInterrupt;

};

