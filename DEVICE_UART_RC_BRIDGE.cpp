#include "DEVICE_UART_RC_BRIDGE.h"
#include "PPMOutput.h"

void DEVICE_UART_RC_BRIDGE::deviceSetup()
{
	m_bPrintIMU = true;// false;
	m_bBootSuccess = true;
	m_bHostConnected = false;
	m_counter = 0;

#ifndef ATMEGA_A328
	m_pRFSerial->begin(115200);
#endif

#ifdef USB_DEBUG
	// wait for Leonardo enumeration, others continue immediately
	while (!(*m_pUSBSerial));
#endif

	if (*m_pUSBSerial)
	{
		//To host side (Android device etc.)
		m_pUSBSerial->begin(115200);
		m_bHostConnected = true;
		m_pUSBSerial->println(F("FALCON_ON"));
	}

	//Init PPM generator
	PPM_init(22500,500);

	//Vehicle Link
	m_VLink.init();

#ifndef ATMEGA_A328
	m_VLink.m_pHostSerial = m_pUSBSerial;
	m_VLink.m_pUartSerial = m_pRFSerial;
#else
	m_VLink.m_pHostSerial = m_pUSBSerial;
#endif

}

void DEVICE_UART_RC_BRIDGE::deviceLoop()
{
	int i;
	/*
	if(g_opeMode==OPE_SERIAL_BRIDGE)
	{
	Serial_Bridge();
	return;
	}
	*/

	if (m_bHostConnected)
	{
		if (m_VLink.receiveFromHost())
		{
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)m_VLink.m_channelValues[i], 1000, 2000);
			}
		}
	}
	else
	{
#ifndef ATMEGA_A328
		if (m_VLink.receiveFromUART())
		{
			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				g_ppm[i] = constrain((uint16_t)m_VLink.m_channelValues[i], 1000, 2000);
			}
		}
#endif
	}


	// slow rate actions
	switch (m_counter)
	{
	case 1:
		if (m_bHostConnected)
		{
			m_VLink.sendHostHeartBeat();
		}
		else
		{
#ifndef ATMEGA_A328
			m_VLink.sendUartHeartBeat();
#endif
		}
		break;
	case 10:
		if (!m_bHostConnected)
		{
			if (Serial)
			{
				m_pUSBSerial = &Serial;
				m_bHostConnected = true;
			}
		}
		break;
	case 100:
		m_counter = 0;
		break;

	default:
		break;
	}
	m_counter++;
}

