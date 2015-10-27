#include "falconcommon.h"
#include "IMU.h"
#include "PPMInput.h"
#include <Wire.h>
#include "LIDARLite.h"
//#include "VehicleLink.h"

struct Lidar_Setting
{
	unsigned char m_address;
	uint8_t	m_pinEN;

	long m_P;
	long m_I;
	long m_Imax;
	long m_D;
};

struct LIDAR_UNIT
{
	long m_distCM;
	long m_lockCM;
	long m_prevErr;
	long m_integErr;
	long m_diverge;

	bool m_bLocked;

	Lidar_Setting m_setting;
};

struct config_t
{
	uint8_t vers;

	int16_t PPMframeLength;
	int16_t PPMPulseLength;
	int16_t PWMLenFrom;
	int16_t PWMLenTo;
	int16_t PWMCenter;

	unsigned char m_ppmIdxRoll;
	unsigned char m_ppmIdxPitch;
	unsigned char m_ppmIdxThrottle;
	unsigned char m_ppmIdxYaw;
	unsigned char m_ppmIdxMode;

	long lidarLim[NUM_LIDAR];
	long cAvoidPWM[NUM_LIDAR];
	int16_t PWM_THR_UP_Lim;
	uint8_t cAvoidALT_PPMIdx;
	uint8_t cAvoidROLL_PPMIdx;

	Lidar_Setting lidar[NUM_LIDAR];
	long m_errLim;
	long m_lidarRangeMax;
	long m_lidarRangeMin;
	long m_rollDSpeed; //difference in CM
	long m_altDSpeed;
	long m_deadZone;
	float m_divergeFactor;
	float m_pwmFactor;
	long m_inputDtime;
};


class DEVICE_LIDAR_LOCKER_2560
{
public:
	void deviceSetup(void);
	void deviceLoop(void);
	void setDefaultParameters(void);
	void serialPrint(void);

	void decideRollLidar();
	void referenceLock();
	void collisionAvoid();

public:
	HardwareSerial* m_pRFSerial;
	HardwareSerial* m_pUSBSerial;

	IMU m_IMU;
	PPMInput m_PPMInput;

	LIDARLite m_LidarLite;
	LIDAR_UNIT m_pLidar[NUM_LIDAR];
	LIDAR_UNIT* m_pLidarUP;
	LIDAR_UNIT* m_pLidarL;
	LIDAR_UNIT* m_pLidarR;
	LIDAR_UNIT* m_pLidarF;
	LIDAR_UNIT* m_pLidarB;
	LIDAR_UNIT* m_pLidarRoll;
	LIDAR_UNIT* m_pLidarPitch;

	//Common classes
//	VehicleLink m_VLink;
	config_t m_config;

	//Program
	int m_counter;

	//switches
	bool m_bPrintOut;
	bool m_bBootSuccess;
	bool m_bHostConnected;

	//General
	int8_t m_opeMode;
	bool m_bMpuInterrupt;

	unsigned long m_timeNow;
	unsigned long m_timePrev;
	unsigned long m_dTime;


};

