

HardwareSerial* g_pUSBSerial;


#include "IMU.h"
#include "FSLPsensor.h"
#include "Controller.h"


//IMU
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}
IMU g_IMU;

FSLPsensor g_fslpL;
FSLPsensor g_fslpR;
Controller g_Controller;


void deviceSetup()
{
	//Setup OLED Display UI
	g_UI.init(&g_OLED);
	g_UI.draw(OPE_BOOT, NULL);

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
	pinMode(BATT_SENSE, INPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	//Init PPM generator
	PPM_init(config);

	//Vehicle Link
	g_VLink.m_pConfig = &config;
	g_VLink.init();
	g_VLink.m_pOprMode = &g_opeMode;
	g_VLink.m_pRFSerial = g_pRFSerial;
	g_VLink.m_pHostSerial = g_pUSBSerial;

	g_VLink.m_channelValues[config.yawChannel.ppmIdx] = config.PWMCenter;
	g_VLink.m_channelValues[config.controlChannel[PITCH].ppmIdx] = config.controlChannel[PITCH].center;
	g_VLink.m_channelValues[config.controlChannel[ROLL].ppmIdx] = config.controlChannel[ROLL].center;
	g_VLink.m_channelValues[config.throttleChannel.ppmIdx] = config.PWMLenFrom;
	g_VLink.m_channelValues[config.buttonChannel[0].ppmIdx] = config.buttonChannel[0].modePPM[0];

	//FSLP sensor
	g_fslpR.setup(A2, A1, A0, A3);
	g_fslpL.setup(A6, A5, A4, A7);

	//Controller components
	g_Controller.m_pfslpL = &g_fslpL;
	g_Controller.m_pfslpR = &g_fslpR;
	g_Controller.m_throttle = 0;
	g_Controller.m_pThrottleChannel = &config.throttleChannel;
	g_Controller.m_pYawChannel = &config.yawChannel;
	g_Controller.m_pConfig = &config;
	g_Controller.m_pVLink = &g_VLink;
	//	g_Controller.m_ypr[YAW]=0;
	g_Controller.m_ypr[PITCH] = 0;
	g_Controller.m_ypr[ROLL] = 0;
	g_Controller.m_pOpeMode = &g_opeMode;
	g_Controller.m_modeChangedL = false;

	//IMU Init
	if (g_IMU.init() != true)
	{
		g_pUSBSerial->println(F("IMU_FAIL"));
		g_UI.draw(OPE_BOOT, NULL);
		g_bBootSuccess = false;
		tone(BUZZER_PIN, 1000, LONG_BEEP);
		return;
	}

	// enable Arduino interrupt detection
	attachInterrupt(6, dmpDataReady, RISING);

	// if programming failed, don't try to do anything
	if (!g_IMU.m_dmpReady)
	{
		g_pUSBSerial->println(F("DMP_FAIL"));
		g_UI.draw(OPE_BOOT, NULL);
		g_bBootSuccess = false;
		tone(BUZZER_PIN, 1000, LONG_BEEP);
		return;
	}

	g_pUSBSerial->println(F("FALCON_START"));
	tone(BUZZER_PIN, 500, LONG_BEEP);

}



void deviceLoop()
{
	int i;

	if (!g_bBootSuccess)
	{
		g_UI.draw(OPE_BOOT, NULL);
		return;
	}

	if (g_opeMode == OPE_SERIAL_BRIDGE)
	{
		Serial_Bridge();
		return;
	}

	if (g_opeMode != OPE_RC_BRIDGE)
	{
		if (mpuInterrupt || (g_IMU.m_fifoCount >= g_IMU.m_packetSize))
		{
			// reset interrupt flag and get INT_STATUS byte
			mpuInterrupt = false;
			g_IMU.update();
			g_Controller.updateAttitude(PITCH, g_IMU.m_ypr[2]);
			g_Controller.updateAttitude(ROLL, g_IMU.m_ypr[1]);

			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 980, 2025);
			}

			//			printf("MPU Updated\n");
		}
	}
	else if (g_opeMode == OPE_RC_BRIDGE)
	{
		if (g_VLink.receiveFromHost())
		{
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 980, 2025);
			}
			//			printf("PPM VLink\n");
			//			tone(BUZZER_PIN, 800, 500);
		}
	}

	// slow rate actions
	switch (g_counter)
	{
	case 1:
		//			g_VLink.receiveFromRF();
		if (g_heartBeatCounter++ >= 10)
		{
			g_VLink.sendHostHeartBeat();
			g_heartBeatCounter = 0;
		}
		break;
	case 2:
		if (g_opeMode != OPE_RC_BRIDGE)
		{
			g_VLink.receiveFromHost();
		}
		break;

	case 3:
		g_UI.draw(g_opeMode, (int16_t*)g_ppm);	//g_VLink.m_channelValues
		break;

	case 4:
		g_Controller.updateYaw();
		break;

	case 5:
		g_Controller.updateThrottle();

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
			/*				g_pUSBSerial->print("ypr\t");
			g_pUSBSerial->print(g_IMU.m_ypr[0]);
			g_pUSBSerial->print("\t");
			g_pUSBSerial->print(g_IMU.m_ypr[1]);
			g_pUSBSerial->print("\t");
			g_pUSBSerial->print(g_IMU.m_ypr[2]);
			g_pUSBSerial->print("\tpre:\t");
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
