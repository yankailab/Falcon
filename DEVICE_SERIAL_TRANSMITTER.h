HardwareSerial* g_pUSBSerial;



void deviceSetup()
{
	//RF RSSI
	pinMode(A12, INPUT);
	//RF CTS
	pinMode(A13, INPUT);

	g_pRFSerial->begin(57600);

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


			g_VLink.sendRFupdateAttitude();
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
