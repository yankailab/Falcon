
#include "falconcommon.h"
#include <Arduino.h>
#include <EEPROM.h>

#ifdef RC_TRANSMITTER
#include <SPI.h>
#include <Wire.h>
#include "UI.h"
#include "U8glib.h"
U8GLIB_SSD1306_128X64 g_OLED(U8G_I2C_OPT_NO_ACK);
UI g_UI;
#endif

//	COMMON INCLUDES
#include "EEPROMAnything.h"
#include "VehicleLink.h"
#include "variables.h"
#include "PPMGenerator.h"

#ifdef UART_RC_BRIDGE
Serial_* g_pUSBSerial;
#else
HardwareSerial* g_pUSBSerial;
#endif
HardwareSerial* g_pRFSerial;

//Common classes
VehicleLink g_VLink;
config_t config;
#include "util.h"

//Program
int g_counter = 0;

//switches
bool g_bPrintIMU = false;
bool g_bBootSuccess = true;
bool g_bHostConnected = false;

//General
uint8_t g_opeMode = /*OPE_SERIAL_BRIDGE;//*/OPE_RC_BRIDGE;// OPE_MANUAL;

#ifdef UART_RC_BRIDGE
//Temporary for test
volatile uint8_t val;
volatile unsigned long timeNow;
volatile unsigned long timeOld;
volatile unsigned int ppmIdx;
volatile unsigned int pulseLength;

unsigned long laser1;
unsigned long laser2;
unsigned long laser3;

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

#endif

#ifdef RC_TRANSMITTER
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
#endif

#ifdef RECEIVER_SIDE
	#include "PPMGenerator.h"
	#include <SoftwareSerial.h>

	SoftwareSerial g_pTELSerial(TEL_RX_PIN,TEL_TX_PIN);
#endif

//INITIAL SETUP 
void setup() 
{

#ifdef SERIAL_TRANSMITTER
	//RF RSSI
	pinMode(A12, INPUT);
	//RF CTS
	pinMode(A13, INPUT);
#endif

#ifdef RC_TRANSMITTER
	//Setup OLED Display UI
	g_UI.init(&g_OLED);
	g_UI.draw(OPE_BOOT, NULL);
#endif
	
	// Read Config or fill with default settings
	if (EEPROM.read(0) == VERSION)
	{
		EEPROM_readAnything(0, config);
	}
	else
	{
		setDefaultParameters();
		EEPROM_writeAnything(0, config);
		RFmoduleConfig();
	}
	
	g_pUSBSerial = &Serial;
	g_pRFSerial = &Serial1;

#ifdef SERIAL_TRANSMITTER
	g_pRFSerial->begin(57600);
#endif

#ifdef UART_RC_BRIDGE
	g_pRFSerial->begin(115200);
#endif

#ifdef USB_DEBUG
	// wait for Leonardo enumeration, others continue immediately
	while (!(*g_pUSBSerial));
#endif

	while (!(*g_pUSBSerial));

	if (*g_pUSBSerial)
	{
		//To host side (Android device etc.)
		g_pUSBSerial->begin(115200);
		g_bHostConnected = true;
		g_pUSBSerial->println(F("FALCON_ON"));
	}

#ifdef UART_RC_BRIDGE
	//Init PPM generator
	PPM_init(config);

	//Vehicle Link
	g_VLink.m_pConfig = &config;
	g_VLink.init();
	g_VLink.m_pOprMode = &g_opeMode;
	g_VLink.m_pUartSerial = g_pRFSerial;
	g_VLink.m_pHostSerial = g_pUSBSerial;

	g_VLink.m_channelValues[config.yawChannel.ppmIdx] = config.PWMCenter;
	g_VLink.m_channelValues[config.controlChannel[PITCH].ppmIdx] = config.controlChannel[PITCH].center;
	g_VLink.m_channelValues[config.controlChannel[ROLL].ppmIdx] = config.controlChannel[ROLL].center;
	g_VLink.m_channelValues[config.throttleChannel.ppmIdx] = config.PWMLenFrom;
	g_VLink.m_channelValues[config.buttonChannel[0].ppmIdx] = config.buttonChannel[0].modePPM[0];

	//Temporal test
	// enable Arduino interrupt detection
	attachInterrupt(4, ppmInt, RISING);
	timeNow = 0;
	timeOld = 0;
	ppmIdx = 0;
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
#endif

#ifdef RC_TRANSMITTER
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
	g_Controller.m_ypr[PITCH]=0;
	g_Controller.m_ypr[ROLL]=0;
	g_Controller.m_pOpeMode = &g_opeMode;
	g_Controller.m_modeChangedL = false;

	//IMU Init
	if(g_IMU.init()!=true)
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

#endif

#ifdef SERIAL_RECEIVER
	//Init PPM generator
	PPM_init();

	g_pTELSerial.begin(57600);

	//Vehicle Link
	g_VLink.m_pRFSerial = g_pRFSerial;
	g_VLink.m_pHostSerial = &g_pTELSerial;
	g_VLink.m_numChannel = 8;

#endif

}


//-------------------------------------
//MAIN PROGRAM LOOP
//-------------------------------------

#ifdef UART_RC_BRIDGE

void loop() 
{
	int i;
	if (g_bHostConnected)
	{
		laser1 = pulseIn(A1, HIGH); // Count how long the pulse is high in microseconds
		laser2 = pulseIn(A2, HIGH);
		laser3 = pulseIn(A3, HIGH);

		laser1 *= 0.1;
		laser2 *= 0.1;
		laser3 *= 0.1;


		Serial.print(laser1);
		Serial.print(" ");
		Serial.print(laser2);
		Serial.print(" ");
		Serial.print(laser3);
		Serial.print(" ");

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
	return;
	
	if(g_opeMode==OPE_SERIAL_BRIDGE)
	{
		Serial_Bridge();
		return;
	}
	
	if (g_bHostConnected)
	{
		if (g_VLink.receiveFromHost())
		{
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 1000, 2000);
			}
		}
	}
	else
	{
		if (g_VLink.receiveFromUART())
		{
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 1000, 2000);
			}
		}
	}


	// slow rate actions
	switch (g_counter)
	{
	case 1:
		if (g_bHostConnected)
		{
			g_VLink.sendHostHeartBeat();
		}
		else
		{
			g_VLink.sendUartHeartBeat();
		}
		break;
	case 10:
		if (!g_bHostConnected)
		{
			if (Serial)
			{
				g_pUSBSerial = &Serial;
				g_bHostConnected = true;
			}
		}
		break;
	case 100:
		g_counter = 0;
		break;

	default:
		break;
	}
	g_counter++;
	
}

#endif


#ifdef SERIAL_TRANSMITTER

void loop() 
{
	int i;

	if (!g_bBootSuccess)
	{
		g_UI.draw(OPE_BOOT, NULL);
		return;
	}

	if(g_opeMode==OPE_SERIAL_BRIDGE)
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

#ifdef RC_TRANSMITTER
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 980, 2025);
			}

//			printf("MPU Updated\n");
#else
			g_VLink.sendRFupdateAttitude();
#endif
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

			if(g_bPrintIMU)
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
	
			g_counter=0;
			break;

		default:
			break;
	}
	g_counter++;

}

#endif

#ifdef SERIAL_RECEIVER
void loop() 
{
	int i;
	
	g_VLink.receiveFromRF();
	g_VLink.receiveFromHost();

	for(i=0; i<RC_CHANNEL_NUM;i++)
	{
		g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i],980,2025);
	}
	
	// slow rate actions
	switch (g_counter)
	{
		case 1:
			break;
		case 2:
			break;

		case 3:
			break;

		case 4:
			break;

		case 5:
			break;

		case 6:
			break;

		case 1000:
/*			if(g_bPrintIMU)
			{
				g_pUSBSerial->print("ypr\t");
				g_pUSBSerial->print(g_ppm[config.controlChannel[YAW].ppmIdx]);
				g_pUSBSerial->print("\t");
				g_pUSBSerial->print(g_ppm[config.controlChannel[PITCH].ppmIdx]);
				g_pUSBSerial->print("\t");
				g_pUSBSerial->print(g_ppm[config.controlChannel[ROLL].ppmIdx]);
				g_pUSBSerial->print("\tThr:\t");
				g_pUSBSerial->println(g_ppm[config.throttleChannel.ppmIdx]);
			}
	*/
			g_counter=0;
			break;

		default:
			break;
	}
	g_counter++;
}

#endif





