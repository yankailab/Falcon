#include "falconcommon.h"
#include "IMU.h"
#include "PPMInput.h"
#include <Wire.h>
#include "LIDARLite.h"
//#include "VehicleLink.h"


#define HISTORY_BUF 10
#define LIDAR_CALIB_SAMPLE 20

#define LIDAR_MODE 2
//#define LIDAR_CONTINUOUS
#define LIDAR_INTERVAL 0xc0
//#define LIDAR_STARTUPCALIB

#define PWM_CENTER_HIGH 1650
#define PWM_CENTER_LOW 1350

#define NUM_LIDAR 4


struct Lidar_Setting
{
	unsigned char m_address;
	uint8_t	m_pinEN;

	long m_P;
	long m_I;
	long m_Imax;
	long m_D;
	long m_criticalRegion;
	int  m_dSpeed;
	long m_offset;
	long m_cAvoidPWM;
	long m_PWMcenter;
};

struct LIDAR_UNIT
{
	long m_distCM;
	long m_prevDistCM;
	long m_lockCM;
	long m_prevErr;
	long m_integErr;
	long m_diverge;

	long m_pHistory[HISTORY_BUF];
	uint8_t m_iHistory;

	Lidar_Setting m_setting;
};

struct config_t
{
	uint8_t vers;

	int16_t PPMframeLength;
	int16_t PPMPulseLength;
	int16_t PWMLenFrom;	//Output range in controlled mode
	int16_t PWMLenTo;
	int16_t PWMRange;	//Output range in through mode

	unsigned char m_ppmIdxRoll;
	unsigned char m_ppmIdxPitch;
	unsigned char m_ppmIdxThrottle;
	unsigned char m_ppmIdxYaw;
	unsigned char m_ppmIdxControlMode;
	unsigned char m_ppmIdxFlightMode;

	long lidarLim[NUM_LIDAR];
	uint8_t cAvoidALT_PPMIdx;
	uint8_t cAvoidROLL_PPMIdx;

	Lidar_Setting lidar[NUM_LIDAR];
	long m_errLim;
	long m_lidarRangeMax;
	long m_lidarRefRangeMax;
	long m_lidarRefRangeMin;
	long m_lidarRefChangeDist;
	long m_deadZone;
	float m_divergeFactor;
	float m_pwmFactor;
	long m_inputDtime;

	uint8_t m_filterWindow;
	uint8_t m_medianIdx;
	float	m_dTdist;

	uint8_t m_LidarIdxUP;
	uint8_t m_LidarIdxDOWN;
	uint8_t m_LidarIdxL;
	uint8_t m_LidarIdxR;
	uint8_t m_LidarIdxF;
	uint8_t m_LidarIdxB;
};


class DEVICE_LIDAR_LOCKER_2560
{
public:
	void deviceSetup(void);
	void deviceLoop(void);
	void setDefaultParameters(void);
	void serialPrint(void);
	void resetLidar(void);

	LIDAR_UNIT* decideLidar(LIDAR_UNIT* pCurrent, LIDAR_UNIT* pLidar1, LIDAR_UNIT* pLidar2);
	void updateLidar(LIDAR_UNIT* pLidar, float factor, float dTdist);
	void updateRefLockPWM(LIDAR_UNIT* pLidar, uint16_t* pPWM, uint8_t ppmIdx);
	void updateStickInput(LIDAR_UNIT* pLidar, int16_t PWM);
	long medianFilter(LIDAR_UNIT* pLidar);
	void lidarCalibration(LIDAR_UNIT* pLidar);
	void checkLockCondition(LIDAR_UNIT* pLidar);

	void collisionAvoid(LIDAR_UNIT* pLidar, uint16_t* pPWM, uint8_t ppmIdx, int16_t dPwmHigh, int16_t dPwmLow);

public:
	HardwareSerial* m_pRFSerial;
	HardwareSerial* m_pUSBSerial;

	IMU m_IMU;
	PPMInput m_PPMInput;

	//Lidar
	LIDARLite m_LidarLite;
	LIDAR_UNIT m_pLidar[NUM_LIDAR];
	LIDAR_UNIT* m_pLidarUP;
	LIDAR_UNIT* m_pLidarDOWN;
	LIDAR_UNIT* m_pLidarL;
	LIDAR_UNIT* m_pLidarR;
	LIDAR_UNIT* m_pLidarF;
	LIDAR_UNIT* m_pLidarB;
	LIDAR_UNIT* m_pLidarRoll;
	LIDAR_UNIT* m_pLidarPitch;
	LIDAR_UNIT* m_pLidarAlt;

	//PWM
	uint16_t* m_pPWMOutPitch;
	uint16_t* m_pPWMOutRoll;
	uint16_t* m_pPWMOutThrottle;

	uint16_t* m_pPWMInPitch;
	uint16_t* m_pPWMInRoll;
	uint16_t* m_pPWMInThrottle;

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
	uint8_t m_opeMode;
	bool m_bMpuInterrupt;

	unsigned long m_timeNow;
	unsigned long m_timePrev;
	unsigned long m_dTime;


};

