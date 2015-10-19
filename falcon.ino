﻿
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
#include "PPMOutput.h"
#include "util.h"

//#include "DEVICE_COMMON.h"
//#include "DEVICE_SERIAL_TRANSMITTER.h"
//#include "DEVICE_SERIAL_RECEIVER.h"
//#include "DEVICE_RC_TRANSMITTER.h"
//#include "DEVICE_UART_RC_BRIDGE.h"
//#include "DEVICE_LIDAR_CONTROLLER.h"
#include "DEVICE_LIDAR_LOCKER_2560.h"
DEVICE_LIDAR_LOCKER_2560 g_Device;

config_t g_config;

//
//IMU
//
volatile bool mpuInterrupt = false; //indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	//mpuInterrupt=true;
	g_Device.m_bMpuInterrupt = true;
}

//
//PPM Input
//
void ppmInt()
{
	g_Device.m_PPMInput.ppmInt();
}

//
//INITIAL SETUP 
//
void setup() 
{
	// Read Config or fill with default settings
	if (EEPROM.read(0) == VERSION)
	{
		EEPROM_readAnything(0, g_config);
	}
	else
	{
		setDefaultParameters(&g_config);
		EEPROM_writeAnything(0, g_config);
	}

	g_Device.m_pUSBSerial = &Serial;
#ifndef ATMEGA_A328
	g_Device.m_pRFSerial = &Serial1;
#endif
	
	g_Device.deviceSetup(&g_config);

	// enable Arduino interrupt detection
	attachInterrupt(6, dmpDataReady, RISING);

	//Init PPM output
	PPM_init(g_config);

	g_Device.m_PPMInput.m_pPPMOut = g_ppm;
	attachInterrupt(PPM_INPUT_INT, ppmInt, RISING);
	//2,3 for I2C, 4 or 5 for GPS port, this is the Arduino INT, not the Atmega2560 INT!



}


//
//MAIN PROGRAM LOOP
//
void loop() 
{
	g_Device.deviceLoop();
}

