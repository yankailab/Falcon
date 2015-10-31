#include "PPMInput.h"

void PPMInput::init(void)
{
	m_timeNow = 0;
	m_timeOld = 0;
	m_ppmIdx = 0;

	//PPM input/output
	m_bPPMthrough = 0xffff; //through mode for all channels, 00011111
//	m_pModeSwitch = &m_inputPPM[m_ppmMODE];
	m_prevModeSwitch = 0;
}


void PPMInput::ppmInt(void)
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
		m_inputPPM[m_ppmIdx] = m_pulseLength;
		if (BIT_ON(m_bPPMthrough, m_ppmIdx))
		{
			if (*m_pOpeMode == OPE_ALL_COLLISION_AVOID)
			{
				m_pulseLength = constrain(m_pulseLength, m_PWMLimLow[m_ppmIdx], m_PWMLimHigh[m_ppmIdx]);

/*				if (m_ppmIdx == m_ppmTHROTTLE)
				{
					m_pulseLength = constrain(m_pulseLength, m_cAvoidThrLowLim, m_cAvoidThrHighLim);
				}
				else if (m_ppmIdx == m_ppmROLL)
				{
					m_pulseLength = constrain(m_pulseLength, m_cAvoidRollLowLim, m_cAvoidRollHighLim);
				}
				else if (m_ppmIdx == m_ppmPITCH)
				{
					m_pulseLength = constrain(m_pulseLength, m_cAvoidPitchLowLim, m_cAvoidPitchHighLim);
				}
*/			}

			m_pPPMOut[m_ppmIdx] = m_pulseLength;
		}

		m_ppmIdx++;
	}
}

uint8_t PPMInput::updateModeSwitch(void)
{
	uint8_t newMode;

	newMode = *m_pOpeMode;

	//Update main switch status
	if (*m_pModeSwitch < SWITCH_LOW)
	{
		if (m_prevModeSwitch >= SWITCH_LOW)
		{
			newMode = OPE_PPM_THROUGH;
		}
	}
	else if (*m_pModeSwitch < SWITCH_MID)
	{
		if (m_prevModeSwitch <= SWITCH_LOW || m_prevModeSwitch > SWITCH_MID)
		{
			newMode = OPE_ALL_COLLISION_AVOID;
		}
	}
	else
	{
		if (m_prevModeSwitch <= SWITCH_MID)
		{
			newMode = OPE_REFERENCE_LOCK;
		}
	}

	m_prevModeSwitch = *m_pModeSwitch;
	return newMode;
}

