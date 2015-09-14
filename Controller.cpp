#include "Controller.h"


Controller::Controller(void)
{
}


Controller::~Controller(void)
{
}

void Controller::updateThrottle(void)
{
	int16_t move;

	m_pfslpL->update();

	//touch released
	if(m_pfslpL->m_pressure < m_pThrottleChannel->touchThreshold)
	{
		m_pfslpL->m_bTouch = false;
		m_modeChangedL = false;
		if (*m_pOpeMode == OPE_ALT_HOLD)
		{
			//auto recover to center point in Alt Hold mode
			m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = m_pConfig->PWMCenter;
		}
		return;
	}

	//avoid multi touch
	if(m_modeChangedL==false) 
	{
		//Touch in the center button area
		if(m_pfslpL->m_pressure > m_pThrottleChannel->modeChangeThreshold
			&& (m_pfslpL->m_position < m_pThrottleChannel->upperLength)
			&& (m_pfslpL->m_position > m_pThrottleChannel->lowerLength))
		{
//			tone(BUZZER_PIN,800,SHORT_BEEP);
			m_modeChangedL = true;

			if (*m_pOpeMode == OPE_MANUAL)
			{
				*m_pOpeMode = OPE_ALT_HOLD;
				m_pVLink->m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
			}
			else// if (*m_pOpeMode == OPE_ALT_HOLD)
			{
				*m_pOpeMode = OPE_MANUAL;
				m_pVLink->m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
			}
			return;
		}
	}

	//New touch started
	if (m_pfslpL->m_bTouch == false)
	{
		m_pfslpL->m_bTouch = true;
	}

	move = m_pfslpL->m_pressure*m_pThrottleChannel->factor;

	if (*m_pOpeMode == OPE_MANUAL)
	{
		if (m_pfslpL->m_position > m_pThrottleChannel->upperLength)
		{
			m_throttle = constrain(m_throttle + move, 0, 1000);
		}
		else if (m_pfslpL->m_position < m_pThrottleChannel->lowerLength)
		{
			m_throttle = constrain(m_throttle - move, 0, 1000);
		}
	}
	else if (*m_pOpeMode == OPE_ALT_HOLD)
	{
		if (m_pfslpL->m_position > m_pThrottleChannel->upperLength)
		{
			m_throttle = constrain(600 + move*10, 0, 1000);
		}
		else if (m_pfslpL->m_position < m_pThrottleChannel->lowerLength)
		{
			m_throttle = constrain(400 - move*10, 0, 1000);
		}
	}

	m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = 
		map(m_throttle, 0, 1000, m_pConfig->PWMLenFrom, m_pConfig->PWMLenTo);

	m_pfslpL->m_lastPosition = m_pfslpL->m_position;

}

void Controller::updateYaw(void)
{
	int16_t move;

	m_pfslpR->update();

	//touch released
	if (m_pfslpR->m_pressure < m_pYawChannel->touchThreshold)
	{
		m_pfslpR->m_bTouch = false;
		m_modeChangedR = false;

		//auto recover to center point in Alt Hold mode
		m_pVLink->m_channelValues[m_pYawChannel->ppmIdx] = m_pConfig->PWMCenter;

		return;
	}

	//avoid multi touch
	if (m_modeChangedR == false)
	{
		//Touch in the center button area
		if (m_pfslpR->m_pressure > m_pYawChannel->modeChangeThreshold
			&& (m_pfslpR->m_position < m_pYawChannel->upperLength)
			&& (m_pfslpR->m_position > m_pYawChannel->lowerLength))
		{
//			tone(BUZZER_PIN, 1000, SHORT_BEEP);
			m_modeChangedR = true;

			return;
		}
	}

	//New touch started
	if (m_pfslpR->m_bTouch == false)
	{
		m_pfslpR->m_bTouch = true;
	}

	move = m_pfslpR->m_pressure*m_pYawChannel->factor;

	if (m_pfslpR->m_position > m_pYawChannel->upperLength)
	{
		m_yaw = constrain(600 + move * 10, 0, 1000);
	}
	else if (m_pfslpR->m_position < m_pYawChannel->lowerLength)
	{
		m_yaw = constrain(400 - move * 10, 0, 1000);
	}

	m_pVLink->m_channelValues[m_pYawChannel->ppmIdx] =
		map(m_yaw, 0, 1000, m_pConfig->PWMLenFrom, m_pConfig->PWMLenTo);

	m_pfslpR->m_lastPosition = m_pfslpR->m_position;

}

void Controller::updateAttitude(byte iChannel, float val)
{
	int16_t ival;

	Control_Channel* pChannel = &m_pConfig->controlChannel[iChannel];
	ival = constrain(((int)(val*(float)pChannel->factor)), 
						pChannel->lowerLimit, 
						pChannel->upperLimit);

	if((ival > -pChannel->deadzone)&&(ival < pChannel->deadzone))
	{
		m_ypr[iChannel] = pChannel->center;
	}
	else
	{
		m_ypr[iChannel] = pChannel->center+ival;
	}

	m_pVLink->m_channelValues[pChannel->ppmIdx] = m_ypr[iChannel];
//		map(m_ypr[iChannel], pChannel->lowerLimit, pChannel->upperLimit,
//							 m_pConfig->PWMLenFrom, m_pConfig->PWMLenTo);
}



/*
void Controller::updateThrottleManual(void)
{
	int16_t move;

	m_pfslpL->update();

	if(m_pfslpL->m_pressure < m_pThrottleChannel->touchThreshold)
	{
		m_pfslpL->m_bTouch = false;
		m_modeChanged = false;
		return;
	}

	if(m_modeChanged==false)
	{
		if(m_pfslpL->m_pressure > m_pThrottleChannel->modeChangeThreshold)
		{
			tone(BUZZER_PIN,500,500);
			*m_pOpeMode = OPE_ALT_HOLD;
			m_modeChanged = true;
			m_pVLink->m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
			return;
		}
	}

	if(m_pfslpL->m_bTouch==false)
	{
		if(m_pfslpL->m_pressure > m_pThrottleChannel->touchThreshold)
		{
			//New touch started
			m_pfslpL->m_bTouch = true;
			m_pfslpL->m_lastPosition = m_pfslpL->m_position;
		}
		return;
	}

	move = m_pfslpL->m_position - m_pfslpL->m_lastPosition;
	m_pfslpL->m_lastPosition = m_pfslpL->m_position;

	move = constrain(move*m_pThrottleChannel->factor, 
							m_pThrottleChannel->lowerLimit, 
							m_pThrottleChannel->upperLimit);

	m_throttle = constrain(m_throttle + move, 0, 1000);

	m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = 
		map(m_throttle, 0, 1000, m_pConfig->PWMLenFrom, m_pConfig->PWMLenTo);

}

void Controller::updateThrottleAltHold(void)
{
	m_pfslpL->update();

	if(m_pfslpL->m_pressure < m_pThrottleChannel->touchThreshold)
	{
		m_pfslpL->m_bTouch = false;
		m_modeChanged = false;
		m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = 1500;
		return;
	}

	if(m_modeChanged==false)
	{
		if(m_pfslpL->m_pressure > m_pThrottleChannel->modeChangeThreshold)
		{
			tone(BUZZER_PIN,500,500);
			*m_pOpeMode = OPE_MANUAL;
			m_modeChanged = true;
			m_pVLink->m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
			return;
		}
	}

	if(m_pfslpL->m_bTouch==false)
	{
		if(m_pfslpL->m_pressure > m_pThrottleChannel->touchThreshold)
		{
			//New touch started
			m_pfslpL->m_bTouch = true;
			m_pfslpL->m_lastPosition = m_pfslpL->m_position;
		}
		return;
	}

	if(m_pfslpL->m_position > m_pThrottleChannel->upperLength)
	{
		m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = 1100;//m_pConfig->PWMLenFrom;
	}
	else if(m_pfslpL->m_position < m_pThrottleChannel->lowerLength)
	{
		m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = 1900;//m_pConfig->PWMLenTo;
	}
	else
	{
		m_pVLink->m_channelValues[m_pThrottleChannel->ppmIdx] = 1500;
	}
}



*/