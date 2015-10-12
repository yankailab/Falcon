
void RFmoduleConfig (void)
{
	//	g_pSerial->print(F("AT+NAME:FALCON"));
	//	g_pSerial->println(F("AT+BAUD:8"));
}

/* */
void Serial_Bridge(void)
{
	int inByte;

	// read from port 1, send to port 0:
	if (g_pUSBSerial->available())
	{
		inByte = g_pUSBSerial->read();
		g_pRFSerial->write(inByte);
	}

	// read from port 0, send to port 1:
	if (g_pRFSerial->available())
	{
		inByte = g_pRFSerial->read();
		g_pUSBSerial->write(inByte);
	}
}

/* */
void setDefaultParameters()
{
	config.vers = VERSION;

	config.PPMframeLength = 22500;
	config.PPMPulseLength = 500;
	config.PWMLenFrom = 1000;
	config.PWMLenTo = 2000;
	config.PWMCenter = 1500;

	config.controlChannel[PITCH].ppmIdx = 1;
	config.controlChannel[PITCH].center = 1500;
	config.controlChannel[PITCH].deadzone = 50;
	config.controlChannel[PITCH].factor = 500;
	config.controlChannel[PITCH].lowerLimit = -500;
	config.controlChannel[PITCH].upperLimit = 500;

	config.controlChannel[ROLL].ppmIdx = 0;
	config.controlChannel[ROLL].center = 1500;
	config.controlChannel[ROLL].deadzone = 50;
	config.controlChannel[ROLL].factor = -500;
	config.controlChannel[ROLL].lowerLimit = -500;
	config.controlChannel[ROLL].upperLimit = 500;

/*	config.controlChannel[YAW].ppmIdx = 3;
	config.controlChannel[YAW].center = 1500;
	config.controlChannel[YAW].deadzone = 50;
	config.controlChannel[YAW].factor = -500;
	config.controlChannel[YAW].lowerLimit = -500;
	config.controlChannel[YAW].upperLimit = 500;
*/
	config.yawChannel.ppmIdx = 3;
	config.yawChannel.touchThreshold = 5;
	config.yawChannel.modeChangeThreshold = 50;
	config.yawChannel.deadzone = 1;
	config.yawChannel.factor = -1;
//	config.yawChannel.lowerLimit = -10;
//	config.yawChannel.upperLimit = 10;
	config.yawChannel.lowerLength = 350;
	config.yawChannel.upperLength = 650;

	config.throttleChannel.ppmIdx = 2;
	config.throttleChannel.touchThreshold = 5;
	config.throttleChannel.modeChangeThreshold = 50;
	config.throttleChannel.deadzone = 1;
	config.throttleChannel.factor = -1;
	config.throttleChannel.lowerLength = 350;
	config.throttleChannel.upperLength = 650;

	config.buttonChannel[0].ppmIdx = 4;
	config.buttonChannel[0].modePPM[0] = 1165;
	config.buttonChannel[0].modePPM[1] = 1295;
	config.buttonChannel[0].modePPM[2] = 1425;
	config.buttonChannel[0].modePPM[3] = 1555;
	config.buttonChannel[0].modePPM[4] = 1685;
	config.buttonChannel[0].modePPM[5] = 1815;

	config.thresholdBTN = 1;

	config.lidarLim[config.controlChannel[ROLL].ppmIdx] = 0;
	config.lidarLim[config.controlChannel[PITCH].ppmIdx] = 0;
	config.lidarLim[config.throttleChannel.ppmIdx] = 0;

	config.cAvoidPWM[config.controlChannel[ROLL].ppmIdx] = 100;
	config.cAvoidPWM[config.controlChannel[PITCH].ppmIdx] = 100;
	config.cAvoidPWM[config.throttleChannel.ppmIdx] = 60;
	config.PWM_THR_UP_Lim = 1580;
}
