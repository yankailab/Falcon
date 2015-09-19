
#ifndef ATMEGA_A328
Serial_* g_pUSBSerial;
#else
HardwareSerial* g_pUSBSerial;
#endif

void deviceSetup()
{
#ifndef ATMEGA_A328
	g_pRFSerial->begin(115200);
#endif

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


	//Init PPM generator
	PPM_init(config);

	//Vehicle Link
	g_VLink.m_pConfig = &config;
	g_VLink.init();
	g_VLink.m_pOprMode = &g_opeMode;

#ifndef ATMEGA_A328
	g_VLink.m_pHostSerial = g_pUSBSerial;
	g_VLink.m_pUartSerial = g_pRFSerial;
#else
	g_VLink.m_pHostSerial = g_pUSBSerial;
#endif

	g_VLink.m_channelValues[config.yawChannel.ppmIdx] = config.PWMCenter;
	g_VLink.m_channelValues[config.controlChannel[PITCH].ppmIdx] = config.controlChannel[PITCH].center;
	g_VLink.m_channelValues[config.controlChannel[ROLL].ppmIdx] = config.controlChannel[ROLL].center;
	g_VLink.m_channelValues[config.throttleChannel.ppmIdx] = config.PWMLenFrom;
	g_VLink.m_channelValues[config.buttonChannel[0].ppmIdx] = config.buttonChannel[0].modePPM[0];

	g_opeMode = OPE_SERIAL_BRIDGE;

}

void deviceLoop()
{
	int i;
	/*
	if(g_opeMode==OPE_SERIAL_BRIDGE)
	{
		Serial_Bridge();
		return;
	}
	*/

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
#ifndef ATMEGA_A328
		if (g_VLink.receiveFromUART())
		{
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 1000, 2000);
			}
		}
#endif
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
#ifndef ATMEGA_A328
			g_VLink.sendUartHeartBeat();
#endif
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