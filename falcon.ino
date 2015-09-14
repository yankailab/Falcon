
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

#include "DEVICE_COMMON.h"
//#include "DEVICE_SERIAL_TRANSMITTER.h"
//#include "DEVICE_SERIAL_RECEIVER.h"
//#include "DEVICE_RC_TRANSMITTER.h"
//#include "DEVICE_UART_RC_BRIDGE.h"
#include "DEVICE_LIDAR_CONTROLLER.h"

#include "util.h"




//INITIAL SETUP 
void setup() 
{
	//Common setup

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
	
	deviceSetup();

}


//-------------------------------------
//MAIN PROGRAM LOOP
//-------------------------------------

void loop() 
{
	deviceLoop();
	
}

