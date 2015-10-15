#include "PPMInput.h"

void PPMInput::init(config_t* pConfig)
{
	m_timeNow = 0;
	m_timeOld = 0;
	m_ppmIdx = 0;

	//PPM input/output
	m_bPPMthrough = 0xffff; //through mode for all channels, 00011111
	m_pPPMSwitch = &m_inputPPM[7];
	m_inputPPMSwitch = 0;

	m_ppmROLL = pConfig->controlChannel[ROLL].ppmIdx;
	m_ppmPITCH = pConfig->controlChannel[PITCH].ppmIdx;
	m_ppmTHROTTLE = pConfig->throttleChannel.ppmIdx;
}


void PPMInput::ppmInt()
{
	m_timeOld = m_timeNow;
	m_timeNow = micros();
	m_pulseLength = m_timeNow - m_timeOld;

	if (m_pulseLength >= 2500)
	{
		m_ppmIdx = 0;
	}
	else
	{
/*		m_inputPPM[m_ppmIdx] = m_pulseLength;
		if (BIT_ON(m_bPPMthrough, m_ppmIdx))
		{
			if (m_ppmIdx == m_ppmTHROTTLE)
			{
				if (g_opeMode == OPE_UP_COLLISION_AVOID)
				{
					if (m_pulseLength >= config.PWM_THR_UP_Lim)
					{
						m_pulseLength = config.PWM_THR_UP_Lim;
					}
				}
			}

			g_ppm[m_ppmIdx] = m_pulseLength;
		}
*/
		m_pPPMOut[m_ppmIdx] = m_pulseLength;
		m_ppmIdx++;
	}
}

void PPMInput::updateSwitch()
{
	/*
	//Update main switch status
	if (*m_pPPMSwitch < SWITCH_LOW)
	{
		if (m_inputPPMSwitch >= SWITCH_LOW)
		{
			//All Lidars OFF
			digitalWrite(g_LidarUP.m_pinTrigger, LOW);
			digitalWrite(g_LidarL.m_pinTrigger, LOW);
			digitalWrite(g_LidarR.m_pinTrigger, LOW);
			//			digitalWrite(g_LidarDOWN.m_pinTrigger, LOW);

			g_opeMode = OPE_PPM_THROUGH;
			g_bPPMthrough = 0xffff;

#ifdef USB_DEBUG
			g_pUSBSerial->println("ALL_LIDARS_OFF");
			g_pUSBSerial->println("PASS_THROUGHT_MODE");
#endif
		}

	}
	else if (*g_pPPMSwitch < SWITCH_MID)
	{
		if (g_inputPPMSwitch <= SWITCH_LOW || g_inputPPMSwitch > SWITCH_MID)
		{
			//UP Lidars ON
			digitalWrite(g_LidarUP.m_pinTrigger, HIGH);
			digitalWrite(g_LidarL.m_pinTrigger, LOW);
			digitalWrite(g_LidarR.m_pinTrigger, LOW);
			//			digitalWrite(g_LidarDOWN.m_pinTrigger, HIGH);

			g_opeMode = OPE_UP_COLLISION_AVOID;
			g_bPPMthrough = 0xff1f;

#ifdef USB_DEBUG
			g_pUSBSerial->println("UP_LIDAR_ON");
			g_pUSBSerial->println("UP_COLLISION_AVOID_MODE");
#endif
		}

	}
	else
	{
		if (g_inputPPMSwitch <= SWITCH_MID)
		{
			//All Lidars ON
			digitalWrite(g_LidarUP.m_pinTrigger, HIGH);
			digitalWrite(g_LidarL.m_pinTrigger, HIGH);
			digitalWrite(g_LidarR.m_pinTrigger, HIGH);

#ifdef USB_DEBUG
			g_pUSBSerial->println("ALL_LIDAR_ON");
			g_pUSBSerial->println("ALL_COLLISION_AVOID_MODE");
#endif
			g_opeMode = OPE_UP_COLLISION_AVOID;// OPE_ALL_COLLISION_AVOID;
			g_bPPMthrough = 0xff1f;
		}

	}
	g_inputPPMSwitch = *g_pPPMSwitch;

	*/
}

