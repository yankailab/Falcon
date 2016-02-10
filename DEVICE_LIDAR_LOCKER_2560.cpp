#include "DEVICE_LIDAR_LOCKER_2560.h"



void DEVICE_LIDAR_LOCKER_2560::deviceSetup(void)
{
	uint8_t i;

	m_bPrintOut = true;//TODO: false;
	m_bBootSuccess = true;
	m_bHostConnected = false;
	m_counter = 0;
	m_timeNow = 0;
	m_timePrev = 0;

#ifdef USB_DEBUG
	// wait for Leonardo enumeration, others continue immediately
	while (!(*g_pUSBSerial));
#endif

	if (*m_pUSBSerial)
	{
		//To host side (Android device etc.)
		m_pUSBSerial->begin(115200);
		m_bHostConnected = true;
		m_pUSBSerial->println(F("FALCON_ON"));
	}

	//IMU Init
	if (m_IMU.init() != true)
	{
		m_pUSBSerial->println(F("IMU_FAIL"));
		m_bBootSuccess = false;
		return;
	}

	// if programming failed, don't try to do anything
	if (!m_IMU.m_dmpReady)
	{
		m_pUSBSerial->println(F("DMP_FAIL"));
		m_bBootSuccess = false;
		return;
	}

	//LidarLite Setup
	for (i = 0; i < NUM_LIDAR; i++)
	{
		m_pLidar[i].m_setting = m_config.lidar[i];
		m_pLidar[i].m_distCM = 0;
		m_pLidar[i].m_prevDistCM = 0;
		m_pLidar[i].m_diverge = 0;
		m_pLidar[i].m_iHistory = 0;
		pinMode(m_pLidar[i].m_setting.m_pinEN, OUTPUT);
	}

	//m_LidarLite.begin(LIDAR_MODE, false, false);

	digitalWrite(A3, HIGH);
	m_LidarLite.begin();
	//	resetLidar();
	m_pLidarUP = &m_pLidar[m_config.m_LidarIdxUP];
	m_pLidarL = &m_pLidar[m_config.m_LidarIdxL];
	m_pLidarR = &m_pLidar[m_config.m_LidarIdxR];
	m_pLidarF = &m_pLidar[m_config.m_LidarIdxF];

#ifdef LIDAR_STARTUPCALIB
	//Lidar Calibration
	lidarCalibration(m_pLidarUP);
	lidarCalibration(m_pLidarL);
	lidarCalibration(m_pLidarR);
	lidarCalibration(m_pLidarF);
#endif

	//Init PPM input
	m_PPMInput.m_pOpeMode = &m_opeMode;
	m_PPMInput.m_pModeSwitch = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxControlMode];

	for (i = 0; i < RC_CHANNEL_NUM; i++)
	{
		//TODO: use calibrated center
		m_PPMInput.m_PWMLimLow[i] = 1500 - m_config.PWMRange;
		m_PPMInput.m_PWMLimHigh[i] = 1500 + m_config.PWMRange;
	}

	m_PPMInput.init();

	m_pPWMOutPitch = &m_PPMInput.m_pPPMOut[m_config.m_ppmIdxPitch];
	m_pPWMOutRoll = &m_PPMInput.m_pPPMOut[m_config.m_ppmIdxRoll];
	m_pPWMOutThrottle = &m_PPMInput.m_pPPMOut[m_config.m_ppmIdxThrottle];

	m_pPWMInPitch = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxPitch];
	m_pPWMInRoll = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxRoll];
	m_pPWMInThrottle = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxThrottle];

	//Set up initial reference choice
	m_pLidarRoll = m_pLidarL;

	//Init system mode
	m_opeMode = OPE_PPM_THROUGH;
//	m_opeMode = OPE_ALL_COLLISION_AVOID;

	m_pUSBSerial->println(F("FALCON_START"));

}


void DEVICE_LIDAR_LOCKER_2560::deviceLoop()
{
	int i;
	float cosRoll, cosPitch;

	if (!m_bBootSuccess)
	{
		m_pUSBSerial->println(F("FALCON_FAIL"));
		return;
	}

	//Update System Time
	m_timePrev = m_timeNow;
	m_timeNow = micros();
	m_dTime = m_timeNow - m_timePrev;

	//Update IMU
	if (m_bMpuInterrupt || (m_IMU.m_fifoCount >= m_IMU.m_packetSize))
	{
		// reset interrupt flag and get INT_STATUS byte
		m_bMpuInterrupt = false;
		m_IMU.update();
	}

	//m_IMU.m_ypr[0]; pitch [2]front is -, roll [1] left is +
	cosRoll = abs(cos(m_IMU.m_ypr[1]));
	cosPitch = abs(cos(m_IMU.m_ypr[2]));

	//Update Lidars
	updateLidar(m_pLidarUP, cosRoll*cosPitch, 0.0);
//	updateLidar(m_pLidarL, cosRoll, m_config.m_dTdist);
//	updateLidar(m_pLidarR, cosRoll, m_config.m_dTdist);
//	updateLidar(m_pLidarF, cosPitch, m_config.m_dTdist);

	//Main controller
	if (m_opeMode == OPE_REFERENCE_LOCK)
	{
		if (m_dTime > m_config.m_inputDtime)
		{
			//Roll Stick input
//			updateStickInput(m_pLidarUP, *m_pPWMInThrottle);
//			updateStickInput(m_pLidarRoll, *m_pPWMInRoll);
//			updateStickInput(m_pLidarF, *m_pPWMInPitch);
		}

		//	updateRefLockPWM(m_pLidarUP, m_pPWMOutThrottle, m_config.m_ppmIdxThrottle);
//		updateRefLockPWM(m_pLidarRoll, m_pPWMOutRoll, m_config.m_ppmIdxRoll);
		//	updateRefLockPWM(m_pLidarF, m_pPWMOutPitch, m_config.m_ppmIdxPitch);

		collisionAvoid(m_pLidarUP, m_pPWMOutThrottle, m_config.m_ppmIdxThrottle, 500, 100);
//		collisionAvoid(m_pLidarF, m_pPWMOutPitch, m_config.m_ppmIdxPitch, 150, 500);
//		collisionAvoid(m_pLidarL, m_pPWMOutRoll, m_config.m_ppmIdxRoll, 100, 500);
//		collisionAvoid(m_pLidarR, m_pPWMOutRoll, m_config.m_ppmIdxRoll, 500, 100);
	}
	else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
	{
		collisionAvoid(m_pLidarUP, m_pPWMOutThrottle, m_config.m_ppmIdxThrottle, 500, 100);
//		collisionAvoid(m_pLidarF, m_pPWMOutPitch, m_config.m_ppmIdxPitch, 150, 500);
//		collisionAvoid(m_pLidarL, m_pPWMOutRoll, m_config.m_ppmIdxRoll, 100, 500);
//		collisionAvoid(m_pLidarR, m_pPWMOutRoll, m_config.m_ppmIdxRoll, 500, 100);

	}

	//Slow rate actions
	switch (m_counter)
	{
	case 1:
/*		if (m_opeMode == OPE_REFERENCE_LOCK)
		{
			m_pLidarR->m_setting.m_P = (((int)m_PPMInput.m_inputPPM[m_config.cAvoidALT_PPMIdx]) - 1500) * 1;
			m_pLidarR->m_setting.m_D = (((int)m_PPMInput.m_inputPPM[m_config.cAvoidROLL_PPMIdx]) - 1500) * 1;
			m_pLidarL->m_setting.m_P = -m_pLidarR->m_setting.m_P;
			m_pLidarL->m_setting.m_D = -m_pLidarR->m_setting.m_D;
			m_config.m_dTdist = abs(m_PPMInput.m_inputPPM[7] - 1025) * 0.0015;
		}
		else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
		{
*/			//Update controller setting input
			//*0.001*4000 ,up to 40m
			m_config.lidarLim[m_config.m_ppmIdxThrottle] = abs(((int)m_PPMInput.m_inputPPM[m_config.cAvoidALT_PPMIdx]) - 1000) * 1;
			m_config.lidarLim[m_config.m_ppmIdxRoll] = 0;// abs(((int)m_PPMInput.m_inputPPM[m_config.cAvoidROLL_PPMIdx]) - 1100) * 1;
			m_config.lidarLim[m_config.m_ppmIdxPitch] = 0;// m_config.lidarLim[m_config.m_ppmIdxRoll];
//		}
		break;

	case 2:
		//Update control mode
		m_opeMode = m_PPMInput.updateModeSwitch();
		if (m_opeMode == OPE_PPM_THROUGH)
		{
			SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
			SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
			SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxPitch);

			for (i = 0; i < RC_CHANNEL_NUM; i++)
			{
				m_PPMInput.m_PWMLimLow[i] = 1500 - m_config.PWMRange;
				m_PPMInput.m_PWMLimHigh[i] = 1500 + m_config.PWMRange;
			}
		}
		break;

	case 3:
		if (m_PPMInput.m_inputPPM[m_config.m_ppmIdxControlMode] > SWITCH_MID)
		{
			//PWM Center Calibration
			m_pLidarUP->m_setting.m_PWMcenter = constrain(*m_pPWMInThrottle, PWM_CENTER_LOW, PWM_CENTER_HIGH);
//			m_pLidarL->m_setting.m_PWMcenter = constrain(*m_pPWMInRoll, PWM_CENTER_LOW, PWM_CENTER_HIGH);
//			m_pLidarR->m_setting.m_PWMcenter = constrain(*m_pPWMInRoll, PWM_CENTER_LOW, PWM_CENTER_HIGH);
//			m_pLidarF->m_setting.m_PWMcenter = constrain(*m_pPWMInPitch, PWM_CENTER_LOW, PWM_CENTER_HIGH);

			m_pUSBSerial->println("PWM Center Calibrated");
		}

	case 4:
		serialPrint();
		m_counter = 0;
		break;

	default:
		m_counter = 0;
		break;
	}
	m_counter++;

}

LIDAR_UNIT* DEVICE_LIDAR_LOCKER_2560::decideLidar(LIDAR_UNIT* pCurrent, LIDAR_UNIT* pLidar1, LIDAR_UNIT* pLidar2)
{
	//if current choice works okay, just keep using it
	if (pCurrent)
	{
		if (pCurrent->m_distCM > m_config.m_lidarRefRangeMin && pCurrent->m_distCM < m_config.m_lidarRefRangeMax)
		{
			return pCurrent;
		}
	}

	if (pLidar1->m_distCM == 0)return pLidar2;
	if (pLidar2->m_distCM == 0)return pLidar1;

	if (pLidar1->m_distCM > m_config.m_lidarRefRangeMax)return pLidar2;
	if (pLidar2->m_distCM > m_config.m_lidarRefRangeMax)return pLidar1;

	if (pLidar1->m_distCM < m_config.m_lidarRefRangeMin)return pLidar2;
	if (pLidar2->m_distCM < m_config.m_lidarRefRangeMin)return pLidar1;

	if (pLidar1->m_diverge > pLidar2->m_diverge)return pLidar2;

	return pLidar1;
}

void DEVICE_LIDAR_LOCKER_2560::updateRefLockPWM(LIDAR_UNIT* pLidar, uint16_t* pPWM, uint8_t ppmIdx)
{
	long err, pwm;

	//Stop ref control and free user input if the distance is 0
	if (pLidar->m_distCM == 0)
	{
		//Reset diverge when distance is infinite
		pLidar->m_diverge = 0;
		pLidar->m_lockCM = 0;

		//Disable controller and let PPM through
		SETBIT_ON(m_PPMInput.m_bPPMthrough, ppmIdx);
		return;
	}
	else if (BIT_ON(m_PPMInput.m_bPPMthrough, ppmIdx))
	{
		//Start ref control again
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, ppmIdx);
		pLidar->m_lockCM = pLidar->m_distCM;
		pLidar->m_prevDistCM = pLidar->m_distCM;
		pLidar->m_integErr = 0;
		pLidar->m_prevErr = 0;
	}

	//if the distance suddenly changes across a threshold, update the reference to new distance
	if (abs(pLidar->m_distCM - pLidar->m_prevDistCM) > m_config.m_lidarRefChangeDist)
	{
		pLidar->m_lockCM = pLidar->m_distCM;
		pLidar->m_prevDistCM = pLidar->m_distCM;
		pLidar->m_integErr = 0;
		pLidar->m_prevErr = 0;
	}

	//PID calculation
	err = pLidar->m_distCM - pLidar->m_lockCM;
	pLidar->m_integErr = constrain(pLidar->m_integErr + err, -m_config.m_errLim, m_config.m_errLim);

	pwm = pLidar->m_setting.m_PWMcenter +
		(long)((float)(pLidar->m_setting.m_P * err
		+ pLidar->m_setting.m_D * (err - pLidar->m_prevErr)
		+ constrain(pLidar->m_setting.m_I * pLidar->m_integErr,
		-pLidar->m_setting.m_Imax,
		pLidar->m_setting.m_Imax)) * m_config.m_pwmFactor);
	
	pwm = constrain(pwm, m_config.PWMLenFrom, m_config.PWMLenTo);
	pLidar->m_prevErr = err;

	*pPWM = pwm;
}

void DEVICE_LIDAR_LOCKER_2560::updateLidar(LIDAR_UNIT* pLidar, float factor, float dTdist)
{
	float dist;
	
#ifdef LIDAR_CONTINUOUS
	dist = factor * ((float)m_LidarLite.distanceContinuous(pLidar->m_setting.m_address));
#else
	dist = factor * ((float)m_LidarLite.distance());// (true, true, pLidar->m_setting.m_address));
#endif

	if (dist < 0)
	{
		resetLidar();
		return;
	}

	dist -= pLidar->m_setting.m_offset;
	if (dist < 0)dist = 0;
	pLidar->m_pHistory[pLidar->m_iHistory] = dist;
	if (++pLidar->m_iHistory >= m_config.m_filterWindow)pLidar->m_iHistory = 0;

	dist = medianFilter(pLidar);
	pLidar->m_prevDistCM = pLidar->m_distCM;
	pLidar->m_distCM = constrain(dist + (dist - (float)pLidar->m_distCM) * dTdist, 0, m_config.m_lidarRangeMax);
	pLidar->m_diverge *= m_config.m_divergeFactor;
	pLidar->m_diverge += abs(dist - pLidar->m_distCM);
}

long DEVICE_LIDAR_LOCKER_2560::medianFilter(LIDAR_UNIT* pLidar)
{
	uint8_t i, j;
	long data[HISTORY_BUF];
	long tmp;

	for (i = 0; i<m_config.m_filterWindow; i++)
	{
		data[i] = pLidar->m_pHistory[i];

		for (j = i; j>0; j--)
		{
			if (data[j] < data[j - 1])
			{
				SWITCH(data[j], data[j - 1], tmp);
			}
			else
			{
				break;
			}
		}
	}

	return ((float)(data[m_config.m_medianIdx] + pLidar->m_distCM))*0.5;
}

void DEVICE_LIDAR_LOCKER_2560::updateStickInput(LIDAR_UNIT* pLidar, int16_t PWM)
{
	long stickInput;

	stickInput = PWM - pLidar->m_setting.m_PWMcenter;
	if (abs(stickInput) > m_config.m_deadZone)
	{
		stickInput = (stickInput > 0) ? 1 : -1;
		pLidar->m_lockCM = constrain(pLidar->m_lockCM + pLidar->m_setting.m_dSpeed*stickInput,
			m_config.m_lidarRefRangeMin,
			m_config.m_lidarRefRangeMax);
	}

}

void DEVICE_LIDAR_LOCKER_2560::resetLidar(void)
{
	int i;

	// Array of pins connected to the sensor Power Enable lines
	int lidarPins[NUM_LIDAR];  //	= { A0, A1, A2, A3 };
	unsigned char lidarAddresses[NUM_LIDAR];  //	= { 0x66, 0x64, 0x68, 0x70 };

	for (i = 0; i < NUM_LIDAR; i++)
	{
		lidarPins[i] = m_pLidar[i].m_setting.m_pinEN;
		lidarAddresses[i] = m_pLidar[i].m_setting.m_address;

		digitalWrite(lidarPins[i], LOW);
	}
	delay(20);

	m_LidarLite.changeAddressMultiPwrEn(NUM_LIDAR, lidarPins, lidarAddresses, false);

#ifdef LIDAR_CONTINUOUS
	for (i = 0; i < NUM_LIDAR; i++)
	{
		m_LidarLite.beginContinuous(false, LIDAR_INTERVAL, 0xff, lidarAddresses[i]);
	}
#endif
}


void DEVICE_LIDAR_LOCKER_2560::collisionAvoid(LIDAR_UNIT* pLidar, uint16_t* pPWM, uint8_t ppmIdx, int16_t dPwmLow, int16_t dPwmHigh)
{
	long dist;

	if (pLidar->m_distCM == 0)
	{
		m_PPMInput.m_PWMLimHigh[ppmIdx] = pLidar->m_setting.m_PWMcenter + m_config.PWMRange;
		m_PPMInput.m_PWMLimLow[ppmIdx] = pLidar->m_setting.m_PWMcenter - m_config.PWMRange;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, ppmIdx);
		return;
	}

	dist = pLidar->m_distCM - m_config.lidarLim[ppmIdx];
	if (dist < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, ppmIdx);
		*pPWM = pLidar->m_setting.m_PWMcenter + pLidar->m_setting.m_cAvoidPWM;
	}
	else if (dist < m_config.lidar[ppmIdx].m_criticalRegion)
	{
		m_PPMInput.m_PWMLimHigh[ppmIdx] = pLidar->m_setting.m_PWMcenter + dPwmHigh;
		m_PPMInput.m_PWMLimLow[ppmIdx] = pLidar->m_setting.m_PWMcenter - dPwmLow;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, ppmIdx);
	}
	else
	{
		m_PPMInput.m_PWMLimHigh[ppmIdx] = pLidar->m_setting.m_PWMcenter + m_config.PWMRange;
		m_PPMInput.m_PWMLimLow[ppmIdx] = pLidar->m_setting.m_PWMcenter - m_config.PWMRange;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, ppmIdx);
	}
}

/*
void DEVICE_LIDAR_LOCKER_2560::collisionAvoid()
{
	long distL, distR, dist;

	// UP direction
	dist = m_pLidarUP->m_distCM - m_config.lidarLim[m_config.m_ppmIdxThrottle];
	if (dist < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
		*m_pPWMOutThrottle = 1500 + m_pLidarUP->m_setting.m_cAvoidPWM;// m_config.cAvoidPWM[m_config.m_ppmIdxThrottle];
	}
	else if (dist < m_config.lidar[m_config.m_ppmIdxThrottle].m_criticalRegion)
	{
		m_PPMInput.m_PWMLimHigh[m_config.m_ppmIdxThrottle] = 1650;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
	}
	else
	{
		m_PPMInput.m_PWMLimHigh[m_config.m_ppmIdxThrottle] = m_pLidarUP->m_setting.m_PWMcenter + m_config.PWMRange;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
	}

	// Foward direction
	dist = m_pLidarF->m_distCM - m_config.lidarLim[m_config.m_ppmIdxPitch];
	if (dist < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxPitch);
		*m_pPWMOutPitch = 1500 + m_pLidarF->m_setting.m_cAvoidPWM;// m_config.cAvoidPWM[m_config.m_ppmIdxPitch];
	}
	else if (dist < m_config.lidar[m_config.m_ppmIdxPitch].m_criticalRegion)
	{
		m_PPMInput.m_PWMLimLow[m_config.m_ppmIdxPitch] = 1350;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxPitch);
	}
	else
	{
		m_PPMInput.m_PWMLimLow[m_config.m_ppmIdxPitch] = m_pLidarF->m_setting.m_PWMcenter - m_config.PWMRange;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxPitch);
	}


	//Roll Axis
	distL = m_pLidarL->m_distCM - m_config.lidarLim[m_config.m_ppmIdxRoll];
	distR = m_pLidarR->m_distCM - m_config.lidarLim[m_config.m_ppmIdxRoll];

	if (distL < 0 && distR < 0)
	{
		if (distL < distR)
		{
			distR = 0;
		}
		else
		{
			distL = 0;
		}
	}

	if (distL < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
		*m_pPWMOutRoll = m_pLidarL->m_setting.m_PWMcenter + m_pLidarL->m_setting.m_cAvoidPWM;// m_config.cAvoidPWM[m_config.m_ppmIdxRoll];
	}
	else if (distR < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
		*m_pPWMOutRoll = m_pLidarR->m_setting.m_PWMcenter + m_pLidarR->m_setting.m_cAvoidPWM;// m_config.cAvoidPWM[m_config.m_ppmIdxRoll];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
	}

}
*/

void DEVICE_LIDAR_LOCKER_2560::lidarCalibration(LIDAR_UNIT* pLidar)
{
	int i;
	for (i = 0; i < LIDAR_CALIB_SAMPLE; i++)
	{
		updateLidar(pLidar, 1.0, 0.0);
	}

	pLidar->m_setting.m_offset = constrain(pLidar->m_distCM - 100, 0, 2000);

}


void DEVICE_LIDAR_LOCKER_2560::setDefaultParameters(void)
{
	m_config.vers = VERSION;

	m_config.PPMframeLength = 22500;
	m_config.PPMPulseLength = 500;
	m_config.PWMLenFrom = 1390;
	m_config.PWMLenTo = 1610;
	m_config.PWMRange = 500;

	m_config.m_ppmIdxPitch = 1;
	m_config.m_ppmIdxRoll = 0;
	m_config.m_ppmIdxThrottle = 2;
	m_config.m_ppmIdxYaw = 3;
	m_config.m_ppmIdxControlMode = 7;
	m_config.m_ppmIdxFlightMode = 4;

	m_config.lidarLim[m_config.m_ppmIdxRoll] = 0;
	m_config.lidarLim[m_config.m_ppmIdxPitch] = 0;
	m_config.lidarLim[m_config.m_ppmIdxThrottle] = 0;

	m_config.cAvoidALT_PPMIdx = 6;
	m_config.cAvoidROLL_PPMIdx = 6;

	m_config.m_errLim = 10000;
	m_config.m_lidarRefRangeMin = 250;
	m_config.m_lidarRefRangeMax = 3000;
	m_config.m_lidarRefChangeDist = 25;
	m_config.m_lidarRangeMax = 4000;
	m_config.m_deadZone = 100;
	m_config.m_divergeFactor = 0.9;
	m_config.m_pwmFactor = 0.01;
	m_config.m_inputDtime = 10000;	//1000 usec, 	

	//Filter
	m_config.m_filterWindow = 9;
	m_config.m_medianIdx = 5;
	m_config.m_dTdist = 0.5;

	//Lidar
	m_config.m_LidarIdxUP = 3;
	m_config.m_LidarIdxL = 2;
	m_config.m_LidarIdxR = 1;
	m_config.m_LidarIdxF = 0;
	m_config.m_LidarIdxB = 0;

	m_config.lidar[m_config.m_LidarIdxUP].m_P = 90;
	m_config.lidar[m_config.m_LidarIdxUP].m_I = 0;
	m_config.lidar[m_config.m_LidarIdxUP].m_Imax = 50;
	m_config.lidar[m_config.m_LidarIdxUP].m_D = 0;
	m_config.lidar[m_config.m_LidarIdxUP].m_criticalRegion = 500;
	m_config.lidar[m_config.m_LidarIdxUP].m_dSpeed = -1;
	m_config.lidar[m_config.m_LidarIdxUP].m_address = 0x70;
	m_config.lidar[m_config.m_LidarIdxUP].m_pinEN = A3;
	m_config.lidar[m_config.m_LidarIdxUP].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxUP].m_cAvoidPWM = -90;
	m_config.lidar[m_config.m_LidarIdxUP].m_PWMcenter = 1500;

	m_config.lidar[m_config.m_LidarIdxL].m_P = -210;
	m_config.lidar[m_config.m_LidarIdxL].m_I = 0;
	m_config.lidar[m_config.m_LidarIdxL].m_Imax = 100;
	m_config.lidar[m_config.m_LidarIdxL].m_D = -360;
	m_config.lidar[m_config.m_LidarIdxL].m_criticalRegion = 300;
	m_config.lidar[m_config.m_LidarIdxL].m_dSpeed = 1;
	m_config.lidar[m_config.m_LidarIdxL].m_address = 0x68;
	m_config.lidar[m_config.m_LidarIdxL].m_pinEN = A2;
	m_config.lidar[m_config.m_LidarIdxL].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxL].m_cAvoidPWM = 75;
	m_config.lidar[m_config.m_LidarIdxL].m_PWMcenter = 1500;

	m_config.lidar[m_config.m_LidarIdxR].m_P = 210;
	m_config.lidar[m_config.m_LidarIdxR].m_I = 0;// 15;
	m_config.lidar[m_config.m_LidarIdxR].m_Imax = 100;
	m_config.lidar[m_config.m_LidarIdxR].m_D = 360;
	m_config.lidar[m_config.m_LidarIdxR].m_criticalRegion = 300;
	m_config.lidar[m_config.m_LidarIdxR].m_dSpeed = -1;
	m_config.lidar[m_config.m_LidarIdxR].m_address = 0x64;
	m_config.lidar[m_config.m_LidarIdxR].m_pinEN = A1;
	m_config.lidar[m_config.m_LidarIdxR].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxR].m_cAvoidPWM = -75;
	m_config.lidar[m_config.m_LidarIdxR].m_PWMcenter = 1500;

	m_config.lidar[m_config.m_LidarIdxF].m_P = 5;
	m_config.lidar[m_config.m_LidarIdxF].m_I = 0;
	m_config.lidar[m_config.m_LidarIdxF].m_Imax = 50;
	m_config.lidar[m_config.m_LidarIdxF].m_D = 0;
	m_config.lidar[m_config.m_LidarIdxF].m_criticalRegion = 500;
	m_config.lidar[m_config.m_LidarIdxF].m_dSpeed = 1;
	m_config.lidar[m_config.m_LidarIdxF].m_address = 0x66;
	m_config.lidar[m_config.m_LidarIdxF].m_pinEN = A0;
	m_config.lidar[m_config.m_LidarIdxF].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxF].m_cAvoidPWM = 65;
	m_config.lidar[m_config.m_LidarIdxF].m_PWMcenter = 1500;

}

void DEVICE_LIDAR_LOCKER_2560::serialPrint()
{
	if (m_bPrintOut)
	{
		if (m_opeMode == OPE_REFERENCE_LOCK)
		{
			m_pUSBSerial->print("[REF LOCK] UP:");
			m_pUSBSerial->print(m_pLidarUP->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarUP->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarUP->m_diverge);
			m_pUSBSerial->print("),  L:");
			m_pUSBSerial->print(m_pLidarL->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarL->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarL->m_diverge);
			m_pUSBSerial->print("),  R:");
			m_pUSBSerial->print(m_pLidarR->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarR->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarR->m_diverge);
			m_pUSBSerial->print("),  F:");
			m_pUSBSerial->print(m_pLidarF->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarF->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarF->m_diverge);

			m_pUSBSerial->print(");  PWM: Thr:");
			m_pUSBSerial->print(*m_pPWMOutThrottle);
			m_pUSBSerial->print(", Roll:");
			m_pUSBSerial->print(*m_pPWMOutRoll);
			
/*			long err;
			m_pUSBSerial->print(", Err:");
			err = m_pLidarUP->m_lockCM - m_pLidarUP->m_distCM;
			m_pUSBSerial->print(err);
			
			m_pUSBSerial->print(", intErr:");
			m_pUSBSerial->print(m_pLidarUP->m_integErr);

			m_pUSBSerial->print(", P*e:");
			m_pUSBSerial->print(m_pLidarUP->m_setting.m_P * err);

			m_pUSBSerial->print(", I*intE:");
			m_pUSBSerial->print(constrain(m_pLidarUP->m_setting.m_I * m_pLidarUP->m_integErr,
				-m_pLidarUP->m_setting.m_Imax,
				m_pLidarUP->m_setting.m_Imax));
*/
			m_pUSBSerial->print(", P_L:");
			m_pUSBSerial->print(m_pLidarRoll->m_setting.m_P);
			m_pUSBSerial->print(", D_L:");
			m_pUSBSerial->print(m_pLidarRoll->m_setting.m_D);
			m_pUSBSerial->print(", dT:");
			m_pUSBSerial->println(m_config.m_dTdist);

		}
		else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
		{
			m_pUSBSerial->print("[COLLISION AVOID] L:");
			m_pUSBSerial->print(m_pLidarL->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_config.m_ppmIdxRoll]);
			m_pUSBSerial->print(", UP:");
			m_pUSBSerial->print(m_pLidarUP->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_config.m_ppmIdxThrottle]);
			m_pUSBSerial->print(", R:");
			m_pUSBSerial->print(m_pLidarR->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_config.m_ppmIdxRoll]);
			m_pUSBSerial->print(", F:");
			m_pUSBSerial->print(m_pLidarF->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_config.m_ppmIdxPitch]);

			m_pUSBSerial->print(", Pitch:");
			m_pUSBSerial->print(m_IMU.m_ypr[2]);
			m_pUSBSerial->print(", Roll:");
			m_pUSBSerial->print(m_IMU.m_ypr[1]);

			m_pUSBSerial->print("  PWM: Thr:");
			m_pUSBSerial->print(*m_pPWMOutThrottle);
			m_pUSBSerial->print(" Roll:");
			m_pUSBSerial->println(*m_pPWMOutRoll);
		}
		else //PPM through
		{
			m_pUSBSerial->print("[PPM THROUGH] PPM: ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[0]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[1]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[2]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[3]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[4]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[5]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[6]);
			m_pUSBSerial->print(" ");
			m_pUSBSerial->println(m_PPMInput.m_pPPMOut[7]);
		}

	}

}
