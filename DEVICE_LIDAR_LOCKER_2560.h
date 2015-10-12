
HardwareSerial* g_pUSBSerial;

#include "IMU.h"
#include <Wire.h>
#include "LIDARLite.h"

//IMU
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}
IMU g_IMU;

//Lidar
int sensorPins[] = { A0, A1, A2 }; // Array of pins connected to the sensor Power Enable lines
unsigned char addresses[] = { 0x66, 0x68, 0x64 };
LIDARLite gLidar;



volatile uint8_t val;
volatile unsigned long timeNow;
volatile unsigned long timeOld;
volatile unsigned int ppmIdx;
volatile unsigned int pulseLength;

struct LIDAR_UNIT
{
	long m_distCM;
	uint8_t	m_pinPWM;
	uint8_t m_pinTrigger;

	uint8_t m_ppmChannel;
	uint8_t m_P;
	uint8_t m_D;

	int m_counter;
	int m_priority;


};

LIDAR_UNIT g_LidarUP;
LIDAR_UNIT g_LidarL;
LIDAR_UNIT g_LidarR;
LIDAR_UNIT g_LidarDOWN;

uint16_t g_inputPPM[RC_CHANNEL_NUM];

void ppmInt()
{
	timeOld = timeNow;
	timeNow = micros();
	pulseLength = timeNow - timeOld;

	if (pulseLength >= 2500)
	{
		ppmIdx = 0;
	}
	else
	{
//		g_inputPPM[ppmIdx] = pulseLength; 
		g_ppm[ppmIdx] = pulseLength;
/*		if (BIT_ON(g_bPPMthrough, ppmIdx))
		{
			if (ppmIdx == g_ppmTHROTTLE)
			{
				if (g_opeMode == OPE_UP_COLLISION_AVOID)
				{
					if (pulseLength >= config.PWM_THR_UP_Lim)
					{
						pulseLength = config.PWM_THR_UP_Lim;
					}
				}
			}

			g_ppm[ppmIdx] = pulseLength;
		}
*/		ppmIdx++;
	}
}



void deviceSetup()
{

#ifdef USB_DEBUG
	// wait for Leonardo enumeration, others continue immediately
	while (!(*g_pUSBSerial));
#endif

	if (*g_pUSBSerial)
	{
		//To host side (Android device etc.)
		g_pUSBSerial->begin(115200);
		g_bHostConnected = true;
		g_pUSBSerial->println(F("FALCON_ON"));
	}

	//Pin setup
//	pinMode(BATT_SENSE, INPUT);
//	pinMode(BUZZER_PIN, OUTPUT);

	//Vehicle Link
/*	g_VLink.m_pConfig = &config;
	g_VLink.init();
	g_VLink.m_pOprMode = &g_opeMode;
	g_VLink.m_pRFSerial = g_pRFSerial;
	g_VLink.m_pHostSerial = g_pUSBSerial;

	g_VLink.m_channelValues[config.yawChannel.ppmIdx] = config.PWMCenter;
	g_VLink.m_channelValues[config.controlChannel[PITCH].ppmIdx] = config.controlChannel[PITCH].center;
	g_VLink.m_channelValues[config.controlChannel[ROLL].ppmIdx] = config.controlChannel[ROLL].center;
	g_VLink.m_channelValues[config.throttleChannel.ppmIdx] = config.PWMLenFrom;
	g_VLink.m_channelValues[config.buttonChannel[0].ppmIdx] = config.buttonChannel[0].modePPM[0];
	*/

	//IMU Init
	if (g_IMU.init() != true)
	{
		g_pUSBSerial->println(F("IMU_FAIL"));
		g_bBootSuccess = false;
		return;
	}
	// enable Arduino interrupt detection
	attachInterrupt(6, dmpDataReady, RISING);

	// if programming failed, don't try to do anything
	if (!g_IMU.m_dmpReady)
	{
		g_pUSBSerial->println(F("DMP_FAIL"));
		g_bBootSuccess = false;
		return;
	}

	//LidarLite Setup
	gLidar.begin();
	gLidar.changeAddressMultiPwrEn(3, sensorPins, addresses, false);


	//Init PPM generator
	PPM_init(config);
	//Enable Arduino interrupt detection
//	attachInterrupt(4, ppmInt, RISING);
	//	attachInterrupt(3, ppmInt, RISING);
	timeNow = 0;
	timeOld = 0;
	ppmIdx = 0;

	g_pUSBSerial->println(F("FALCON_START"));
}



void deviceLoop()
{
	int i;

	if (!g_bBootSuccess)
	{
		g_pUSBSerial->println(F("FALCON_FAIL"));
		return;
	}

		if (mpuInterrupt || (g_IMU.m_fifoCount >= g_IMU.m_packetSize))
		{
			// reset interrupt flag and get INT_STATUS byte
			mpuInterrupt = false;
			g_IMU.update();
//			g_Controller.updateAttitude(PITCH, g_IMU.m_ypr[2]);
//			g_Controller.updateAttitude(ROLL, g_IMU.m_ypr[1]);

/*			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 980, 2025);
			}
			*/
			//			printf("MPU Updated\n");
		}

	// slow rate actions
	switch (g_counter)
	{
	case 1:
		//			g_VLink.receiveFromRF();
/*		if (g_heartBeatCounter++ >= 10)
		{
			g_VLink.sendHostHeartBeat();
			g_heartBeatCounter = 0;
		}
*/		break;
	case 2:
/*		if (g_opeMode != OPE_RC_BRIDGE)
		{
			g_VLink.receiveFromHost();
		}
*/		break;

	case 3:
//		g_UI.draw(g_opeMode, (int16_t*)g_ppm);	//g_VLink.m_channelValues
		break;

	case 4:
//		g_Controller.updateYaw();
		break;

	case 5:
//		g_Controller.updateThrottle();

		if (g_bPrintIMU)
		{
			/*
			g_pUSBSerial->print("ypr\t");
			g_pUSBSerial->print(g_VLink.m_channelValues[config.controlChannel[YAW].ppmIdx]);//(g_Controller.m_ypr[0]);
			g_pUSBSerial->print("\t");
			g_pUSBSerial->print(g_VLink.m_channelValues[config.controlChannel[PITCH].ppmIdx]);//(g_Controller.m_ypr[1]);
			g_pUSBSerial->print("\t");
			g_pUSBSerial->print(g_VLink.m_channelValues[config.controlChannel[ROLL].ppmIdx]);//(g_Controller.m_ypr[2]);
			g_pUSBSerial->print("\tpre:\t");
			g_pUSBSerial->print(g_FSLP.m_pressure);
			g_pUSBSerial->print("\tpos:\t");
			g_pUSBSerial->print(g_FSLP.m_position);
			g_pUSBSerial->print("\tThr:\t");
			g_pUSBSerial->println(g_VLink.m_channelValues[config.throttleChannel.ppmIdx]);//(g_Controller.m_throttle);
			*/

			g_pUSBSerial->print("Sensor 0x66: ");
			g_pUSBSerial->print(gLidar.distance(true, true, 0x66));
			g_pUSBSerial->print(", Sensor 0x68: ");
			g_pUSBSerial->print(gLidar.distance(true, true, 0x68));
			g_pUSBSerial->print(", Sensor 0x64: ");
			g_pUSBSerial->print(gLidar.distance(true, true, 0x64));

			g_pUSBSerial->print("   ypr\t");
			g_pUSBSerial->print(g_IMU.m_ypr[0]);
			g_pUSBSerial->print("\t");
			g_pUSBSerial->print(g_IMU.m_ypr[1]);
			g_pUSBSerial->print("\t");
			g_pUSBSerial->println(g_IMU.m_ypr[2]);
/*			g_pUSBSerial->print("\tpre:\t");
			g_pUSBSerial->print(g_FSLP.m_pressure);
			g_pUSBSerial->print("\tpos:\t");
			g_pUSBSerial->print(g_FSLP.m_position);
			g_pUSBSerial->print("\tThr:\t");
			g_pUSBSerial->println(g_Controller.m_throttle);
*/
		}

		g_counter = 0;
		break;

	default:
		break;
	}
	g_counter++;

}
