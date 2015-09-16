
#define LD_TIMEOUT 60000
#define LD_NUM 4

volatile uint8_t val;
volatile unsigned long timeNow;
volatile unsigned long timeOld;
volatile unsigned int ppmIdx;
volatile unsigned int pulseLength;

Serial_* g_pUSBSerial;

struct LIDAR_UNIT
{
	unsigned long m_distCM;
	uint8_t	m_pinPWM;
	uint8_t m_pinTrigger;

	uint8_t m_ppmChannel;
	uint8_t m_P;
	uint8_t m_D;


};

LIDAR_UNIT g_pLidar[LD_NUM];

LIDAR_UNIT* pLidarUP;
LIDAR_UNIT* pLidarL;
LIDAR_UNIT* pLidarR;
LIDAR_UNIT* pLidarDOWN;

//Yaw, Pitch, Roll
float g_attitude[3];
unsigned long g_dist;


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
		g_ppm[ppmIdx] = pulseLength;
		ppmIdx++;
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
		g_pUSBSerial->println(F("FALCON_LIDAR"));
	}

	g_pRFSerial->begin(115200);

	//Pin setup
	pinMode(PPM_INPUT_PIN, INPUT);
	pinMode(PPM_OUTPUT_PIN, OUTPUT);

	//Init PPM generator
	PPM_init(config);

	//Vehicle Link
/*	g_VLink.m_pConfig = &config;
	g_VLink.init();
	g_VLink.m_pOprMode = &g_opeMode;
	g_VLink.m_pUartSerial = g_pRFSerial;
	g_VLink.m_pHostSerial = g_pUSBSerial;

	g_VLink.m_channelValues[config.yawChannel.ppmIdx] = config.PWMCenter;
	g_VLink.m_channelValues[config.controlChannel[PITCH].ppmIdx] = config.controlChannel[PITCH].center;
	g_VLink.m_channelValues[config.controlChannel[ROLL].ppmIdx] = config.controlChannel[ROLL].center;
	g_VLink.m_channelValues[config.throttleChannel.ppmIdx] = config.PWMLenFrom;
	g_VLink.m_channelValues[config.buttonChannel[0].ppmIdx] = config.buttonChannel[0].modePPM[0];
*/

	//Enable Arduino interrupt detection
	attachInterrupt(4, ppmInt, RISING);
	timeNow = 0;
	timeOld = 0;
	ppmIdx = 0;

	//Lidar
	g_pLidar[0].m_pinPWM = 21;
	g_pLidar[1].m_pinPWM = 20;
	g_pLidar[2].m_pinPWM = 19;
	g_pLidar[3].m_pinPWM = 18;

	g_pLidar[0].m_pinTrigger = 15;
	g_pLidar[1].m_pinTrigger = 14;
	g_pLidar[2].m_pinTrigger = 16;
	g_pLidar[3].m_pinTrigger = 10;

	pLidarL = &g_pLidar[0];
	pLidarUP = &g_pLidar[1];
	pLidarR = &g_pLidar[2];
	pLidarDOWN = &g_pLidar[3];

	for (int i = 0; i < LD_NUM; i++)
	{
		pinMode(g_pLidar[i].m_pinPWM, INPUT);
		pinMode(g_pLidar[i].m_pinTrigger, OUTPUT);
	}

}


void deviceLoop()
{
	int i;

	// Update Lidar sensors
	for (i = 0; i < LD_NUM; i++)
	{
		g_pLidar[i].m_distCM = pulseIn(g_pLidar[i].m_pinPWM, HIGH, LD_TIMEOUT)*0.1; //microseconds, 10 usec = 1cm
		if (g_pLidar[i].m_distCM == 0)
		{
			digitalWrite(g_pLidar[i].m_pinTrigger, LOW);
			delayMicroseconds(10);
			digitalWrite(g_pLidar[i].m_pinTrigger, HIGH);
		}
	}

	//Update switches
	//Reset All Lidars
	//Save to flash


	//Update Attitude, TODO: change to using IMU
	g_attitude[PITCH] = cos(abs(g_ppm[config.controlChannel[PITCH].ppmIdx] - 1500)*0.002);
	g_attitude[ROLL] = cos(abs(g_ppm[config.controlChannel[ROLL].ppmIdx] - 1500)*0.002);

	pLidarL->m_distCM *= g_attitude[ROLL];
	pLidarR->m_distCM *= g_attitude[ROLL];
	pLidarUP->m_distCM *= g_attitude[PITCH];


	// Left
	g_dist = pLidarL->m_distCM - config.lidarLim[ROLL];
	if (g_dist < 0)
	{
		g_ppm[ROLL] = 1600;
	}

	// Right


	// Up

	// Altitude





	// slow rate actions
	switch (g_counter)
	{
	case 1:
		if (g_bHostConnected)
		{
			for (i = 0; i < LD_NUM; i++)
			{
				Serial.print(g_pLidar[i].m_distCM);
				Serial.print(" ");
			}

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
		break;
	case 2:
		if (!g_bHostConnected)
		{
			if (Serial)
			{
				g_pUSBSerial = &Serial;
				g_bHostConnected = true;
			}
		}
		break;
	case 3:
		//TODO: adjust lidarLim
		break;
	case 5:
		g_counter = 0;
		break;

	default:
		break;
	}
	g_counter++;
}