
#define LD_TIMEOUT 60000

#define SWITCH_LOW 1250
#define SWITCH_MID 1650

volatile uint8_t val;
volatile unsigned long timeNow;
volatile unsigned long timeOld;
volatile unsigned int ppmIdx;
volatile unsigned int pulseLength;

Serial_* g_pUSBSerial;

struct LIDAR_UNIT
{
	long m_distCM;
	uint8_t	m_pinPWM;
	uint8_t m_pinTrigger;

	uint8_t m_ppmChannel;
	uint8_t m_P;
	uint8_t m_D;


};

LIDAR_UNIT g_LidarUP;
LIDAR_UNIT g_LidarL;
LIDAR_UNIT g_LidarR;
LIDAR_UNIT g_LidarDOWN;

//Yaw, Pitch, Roll
float g_attitude[3];

//Switches
/*
Channel 6: Up distance
Channel 7: Left/Right distance
*/
uint16_t g_inputPPMSwitch;
uint16_t g_inputPPM[RC_CHANNEL_NUM];
uint16_t* g_pPPMSwitch;

uint8_t g_ppmROLL;
uint8_t g_ppmPITCH;
uint8_t g_ppmTHROTTLE;

//8 channels currently
uint16_t g_bPPMthrough;	


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
		g_inputPPM[ppmIdx] = pulseLength;
		if (BIT_ON(g_bPPMthrough,ppmIdx))
		{
			g_ppm[ppmIdx] = pulseLength;
		}
		ppmIdx++;
	}
}

void resetLidar(int pinTrigger)
{
	digitalWrite(pinTrigger, LOW);
	delayMicroseconds(100);
	digitalWrite(pinTrigger, HIGH);
}

void deviceSetup()
{
	int i;

	if (*g_pUSBSerial)
	{
		//To host side (Android device etc.)
		g_pUSBSerial->begin(115200);
		g_bHostConnected = true;
		g_pUSBSerial->println(F("FALCON_LIDAR"));
	}

	g_pRFSerial->begin(115200);

	//Pin setup
	pinMode(PPM_INPUT_PIN, INPUT);
	pinMode(PPM_OUTPUT_PIN, OUTPUT);

	//Init PPM generator
	PPM_init(config);

	//Enable Arduino interrupt detection
	attachInterrupt(4, ppmInt, RISING);
	timeNow = 0;
	timeOld = 0;
	ppmIdx = 0;

	//Lidar
	g_LidarL.m_pinPWM = 21;
	g_LidarUP.m_pinPWM = 20;
	g_LidarR.m_pinPWM = 19;
	g_LidarDOWN.m_pinPWM = 18;

	g_LidarL.m_pinTrigger = 15;
	g_LidarUP.m_pinTrigger = 14;
	g_LidarR.m_pinTrigger = 16;
	g_LidarDOWN.m_pinTrigger = 10;
	g_LidarDOWN.m_distCM = 0;

	pinMode(g_LidarL.m_pinPWM, INPUT);
	pinMode(g_LidarL.m_pinTrigger, OUTPUT);
	pinMode(g_LidarUP.m_pinPWM, INPUT);
	pinMode(g_LidarUP.m_pinTrigger, OUTPUT);
	pinMode(g_LidarR.m_pinPWM, INPUT);
	pinMode(g_LidarR.m_pinTrigger, OUTPUT);
	pinMode(g_LidarDOWN.m_pinPWM, INPUT);
	pinMode(g_LidarDOWN.m_pinTrigger, OUTPUT);

	//PPM input/output
	g_bPPMthrough = 0xffff; //through mode for all channels, 00011111
	g_pPPMSwitch = &g_inputPPM[7];
	g_inputPPMSwitch = 0;
	g_opeMode = OPE_PPM_THROUGH;

	g_ppmROLL = config.controlChannel[ROLL].ppmIdx;
	g_ppmPITCH = config.controlChannel[PITCH].ppmIdx;
	g_ppmTHROTTLE = config.throttleChannel.ppmIdx;


}


void deviceLoop()
{
	int i;
	long distCM;


	//Update main switch status
	if (*g_pPPMSwitch < SWITCH_LOW)
	{
		if (g_inputPPMSwitch >= SWITCH_LOW)
		{
			//All Lidars OFF
			digitalWrite(g_LidarUP.m_pinTrigger, LOW);
			digitalWrite(g_LidarL.m_pinTrigger, LOW);
			digitalWrite(g_LidarR.m_pinTrigger, LOW);
			digitalWrite(g_LidarDOWN.m_pinTrigger, LOW);

#ifdef USB_DEBUG
			g_pUSBSerial->println("ALL_LIDARS_OFF");
			g_pUSBSerial->println("PASS_THROUGHT_MODE");
#endif
		}

		g_opeMode = OPE_PPM_THROUGH;
		g_bPPMthrough = 0xffff;
	}
	else if (*g_pPPMSwitch < SWITCH_MID)
	{
		if (g_inputPPMSwitch <= SWITCH_LOW)
		{
			//All Lidars ON
			digitalWrite(g_LidarUP.m_pinTrigger, HIGH);
			digitalWrite(g_LidarL.m_pinTrigger, HIGH);
			digitalWrite(g_LidarR.m_pinTrigger, HIGH);
			digitalWrite(g_LidarDOWN.m_pinTrigger, HIGH);

#ifdef USB_DEBUG
			g_pUSBSerial->println("ALL_LIDARS_ON");
			g_pUSBSerial->println("COLLISION_AVOID_MODE");
#endif
		}

		g_opeMode = OPE_COLLISION_AVOID;
		g_bPPMthrough = 0xff1f;
	}
	else
	{
		if (g_inputPPMSwitch <= SWITCH_MID)
		{
			//Save to flash
			EEPROM_writeAnything(0, config);

#ifdef USB_DEBUG
			g_pUSBSerial->println("PARAMETER_SAVED_TO_FLASH");
#endif
		}
	}
	g_inputPPMSwitch = *g_pPPMSwitch;



	//Update controller setting input
	//*0.001*4000 ,up to 40m
	config.lidarLim[g_ppmTHROTTLE] = abs(g_inputPPM[5]-1000)*1;	
	config.lidarLim[g_ppmROLL] = abs(g_inputPPM[6]-1000)*1;

	//Collision avoid mode
	if (g_opeMode == OPE_COLLISION_AVOID)
	{
		// Update Lidar sensors, pulsedIn in microseconds, 10 usec = 1cm
		g_LidarL.m_distCM = pulseIn(g_LidarL.m_pinPWM, HIGH, LD_TIMEOUT)*0.1;
		if (g_LidarL.m_distCM == 0)resetLidar(g_LidarL.m_pinTrigger);

		g_LidarUP.m_distCM = pulseIn(g_LidarUP.m_pinPWM, HIGH, LD_TIMEOUT)*0.1;
		if (g_LidarUP.m_distCM == 0)resetLidar(g_LidarUP.m_pinTrigger);

		g_LidarR.m_distCM = pulseIn(g_LidarR.m_pinPWM, HIGH, LD_TIMEOUT)*0.1;
		if (g_LidarR.m_distCM == 0)resetLidar(g_LidarR.m_pinTrigger);


		//Update Attitude, TODO: change to using IMU
//		g_attitude[PITCH] = cos(abs(g_ppm[config.controlChannel[PITCH].ppmIdx] - 1500)*0.002);
/*		g_attitude[ROLL] = cos(abs(g_ppm[config.controlChannel[ROLL].ppmIdx] - 1500)*0.002);
		pLidarL->m_distCM *= g_attitude[ROLL];
		pLidarR->m_distCM *= g_attitude[ROLL]; */
//		pLidarUP->m_distCM *= g_attitude[PITCH];

		//TODO solve the conflict of L and R

		// Left
		distCM = g_LidarL.m_distCM - config.lidarLim[g_ppmROLL];
		if (distCM < 0)
		{
			SETBIT_OFF(g_bPPMthrough, g_ppmROLL);
			g_ppm[g_ppmROLL] = 1500 + config.cAvoidPWM[g_ppmROLL];
		}
		else
		{
			SETBIT_ON(g_bPPMthrough, g_ppmROLL);
		}

/*		// Right
		distCM = g_LidarR.m_distCM - config.lidarLim[g_ppmROLL];
		if (distCM < 0)
		{
			SETBIT_OFF(g_bPPMthrough, g_ppmROLL);
			g_ppm[g_ppmROLL] = 1500 - config.cAvoidPWM[g_ppmROLL];
		}
		else
		{
			SETBIT_ON(g_bPPMthrough, g_ppmROLL);
		}
*/
		// Up
		distCM = g_LidarUP.m_distCM - config.lidarLim[g_ppmTHROTTLE];
		if (distCM < 0)
		{
			SETBIT_OFF(g_bPPMthrough, g_ppmTHROTTLE);
			g_ppm[g_ppmTHROTTLE] = 1500 - config.cAvoidPWM[g_ppmTHROTTLE];
		}
		else
		{
			SETBIT_ON(g_bPPMthrough, g_ppmTHROTTLE);
		}

	}

	// slow rate actions
	switch (g_counter)
	{
	case 1:
		//Recognize USB if connected
		if (!g_bHostConnected)
		{
			if (Serial)
			{
				g_pUSBSerial = &Serial;
				g_bHostConnected = true;
			}
		}
		break;

	case 2:

		break;

	case 3:
#ifdef USB_DEBUG
		if (g_bHostConnected)
		{
			Serial.print("L=");
			Serial.print(g_LidarL.m_distCM);
			Serial.print("/");
			Serial.print(config.lidarLim[g_ppmROLL]);

			Serial.print(" UP=");
			Serial.print(g_LidarUP.m_distCM);
			Serial.print("/");
			Serial.print(config.lidarLim[g_ppmTHROTTLE]);

			Serial.print(" R=");
			Serial.print(g_LidarR.m_distCM);
			Serial.print("/");
			Serial.print(config.lidarLim[g_ppmROLL]);

			Serial.print(" DOWN=");
			Serial.print(g_LidarDOWN.m_distCM);
			Serial.print("/");
			Serial.print(config.lidarLim[g_ppmTHROTTLE]);
			Serial.print(" [");

			Serial.print(g_bPPMthrough);
			Serial.print("] ");

			g_pUSBSerial->print(g_ppm[0]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->print(g_ppm[1]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->print(g_ppm[2]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->print(g_ppm[3]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->print(g_ppm[4]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->print(g_ppm[5]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->print(g_ppm[6]);
			g_pUSBSerial->print(" ");
			g_pUSBSerial->println(g_ppm[7]);
		}
#endif
		break;

	case 4:
		g_counter = 0;
		break;

	default:
		break;
	}
	g_counter++;
}