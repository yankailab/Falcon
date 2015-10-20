
int sign(double x) { return (x>0) - (x<0); }

void RFmoduleConfig (void)
{
	//	g_pSerial->print(F("AT+NAME:FALCON"));
	//	g_pSerial->println(F("AT+BAUD:8"));
}

/* 
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
*/

/*
void setDefaultParameters(config_t* pConfig)
{
	pConfig->vers = VERSION;

	pConfig->PPMframeLength = 22500;
	pConfig->PPMPulseLength = 500;
	pConfig->PWMLenFrom = 1000;
	pConfig->PWMLenTo = 2000;
	pConfig->PWMCenter = 1500;

	pConfig->controlChannel[PITCH].ppmIdx = 1;
	pConfig->controlChannel[PITCH].center = 1500;
	pConfig->controlChannel[PITCH].deadzone = 50;
	pConfig->controlChannel[PITCH].factor = 500;
	pConfig->controlChannel[PITCH].lowerLimit = -500;
	pConfig->controlChannel[PITCH].upperLimit = 500;

	pConfig->controlChannel[ROLL].ppmIdx = 0;
	pConfig->controlChannel[ROLL].center = 1500;
	pConfig->controlChannel[ROLL].deadzone = 50;
	pConfig->controlChannel[ROLL].factor = -500;
	pConfig->controlChannel[ROLL].lowerLimit = -500;
	pConfig->controlChannel[ROLL].upperLimit = 500;

//	pConfig->controlChannel[YAW].ppmIdx = 3;
//	pConfig->controlChannel[YAW].center = 1500;
//	pConfig->controlChannel[YAW].deadzone = 50;
//	pConfig->controlChannel[YAW].factor = -500;
//	pConfig->controlChannel[YAW].lowerLimit = -500;
//	pConfig->controlChannel[YAW].upperLimit = 500;

	pConfig->yawChannel.ppmIdx = 3;
	pConfig->yawChannel.touchThreshold = 5;
	pConfig->yawChannel.modeChangeThreshold = 50;
	pConfig->yawChannel.deadzone = 1;
	pConfig->yawChannel.factor = -1;
//	pConfig->yawChannel.lowerLimit = -10;
//	pConfig->yawChannel.upperLimit = 10;
	pConfig->yawChannel.lowerLength = 350;
	pConfig->yawChannel.upperLength = 650;

	pConfig->throttleChannel.ppmIdx = 2;
	pConfig->throttleChannel.touchThreshold = 5;
	pConfig->throttleChannel.modeChangeThreshold = 50;
	pConfig->throttleChannel.deadzone = 1;
	pConfig->throttleChannel.factor = -1;
	pConfig->throttleChannel.lowerLength = 350;
	pConfig->throttleChannel.upperLength = 650;

	pConfig->buttonChannel[0].ppmIdx = 4;
	pConfig->buttonChannel[0].modePPM[0] = 1165;
	pConfig->buttonChannel[0].modePPM[1] = 1295;
	pConfig->buttonChannel[0].modePPM[2] = 1425;
	pConfig->buttonChannel[0].modePPM[3] = 1555;
	pConfig->buttonChannel[0].modePPM[4] = 1685;
	pConfig->buttonChannel[0].modePPM[5] = 1815;

	pConfig->thresholdBTN = 1;

	pConfig->lidarLim[pConfig->controlChannel[ROLL].ppmIdx] = 0;
	pConfig->lidarLim[pConfig->controlChannel[PITCH].ppmIdx] = 0;
	pConfig->lidarLim[pConfig->throttleChannel.ppmIdx] = 0;

	pConfig->cAvoidPWM[pConfig->controlChannel[ROLL].ppmIdx] = 100;
	pConfig->cAvoidPWM[pConfig->controlChannel[PITCH].ppmIdx] = 100;
	pConfig->cAvoidPWM[pConfig->throttleChannel.ppmIdx] = 60;
	pConfig->PWM_THR_UP_Lim = 1580;
}
*/