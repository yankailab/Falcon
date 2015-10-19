#include "PPMInput.h"

void PPMInput::init(config_t* pConfig)
{
	m_timeNow = 0;
	m_timeOld = 0;
	m_ppmIdx = 0;

	//PPM input/output
	m_bPPMthrough = 0xffff; //through mode for all channels, 00011111
	m_pModeSwitch = &m_inputPPM[PPM_CHANNEL_MODE];
	m_prevModeSwitch = 0;

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

uint8_t PPMInput::updateSwitch()
{
	uint8_t newMode;

	newMode = 0;

	//Update main switch status
	if (*m_pModeSwitch < SWITCH_LOW)
	{
		if (m_prevModeSwitch >= SWITCH_LOW)
		{
			//All Lidars OFF
			newMode = OPE_PPM_THROUGH;
			m_bPPMthrough = 0xffff;

#ifdef USB_DEBUG
			g_pUSBSerial->println("PASS_THROUGHT_MODE");
#endif
		}

	}
	else if (*m_pModeSwitch < SWITCH_MID)
	{
		if (m_prevModeSwitch <= SWITCH_LOW || m_prevModeSwitch > SWITCH_MID)
		{
			newMode = OPE_ALL_COLLISION_AVOID;
			m_bPPMthrough = 0xff1f;

#ifdef USB_DEBUG
			g_pUSBSerial->println("ALL_COLLISION_AVOID_MODE");
#endif
		}

	}
	else
	{
		if (m_prevModeSwitch <= SWITCH_MID)
		{
			newMode = OPE_REFERENCE_LOCK;
			m_bPPMthrough = 0xff1f;

#ifdef USB_DEBUG
			g_pUSBSerial->println("REFERENCE_LOCK_MODE");
#endif
		}

	}

	m_prevModeSwitch = *m_pModeSwitch;
	return newMode;

	
}

