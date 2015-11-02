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
		m_pLidar[i].m_diverge = 0;
		m_pLidar[i].m_iHistory = 0;
		pinMode(m_pLidar[i].m_setting.m_pinEN, OUTPUT);
	}

	m_LidarLite.begin(LIDAR_MODE, false, false);
	resetLidar();
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
	m_PPMInput.m_pModeSwitch = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxMode];

	for (i = 0; i < RC_CHANNEL_NUM; i++)
	{
		m_PPMInput.m_PWMLimLow[i] = m_config.PWMCenter - m_config.PWMRange;
		m_PPMInput.m_PWMLimHigh[i] = m_config.PWMCenter + m_config.PWMRange;
	}

	m_PPMInput.init();

	m_pPWMOutPitch = &m_PPMInput.m_pPPMOut[m_config.m_ppmIdxPitch];
	m_pPWMOutRoll = &m_PPMInput.m_pPPMOut[m_config.m_ppmIdxRoll];
	m_pPWMOutThrottle = &m_PPMInput.m_pPPMOut[m_config.m_ppmIdxThrottle];

	m_pPWMInPitch = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxPitch];
	m_pPWMInRoll = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxRoll];
	m_pPWMInThrottle = &m_PPMInput.m_inputPPM[m_config.m_ppmIdxThrottle];

	//Init system mode
	m_opeMode = OPE_PPM_THROUGH;

	m_pUSBSerial->println(F("FALCON_START"));

}


void DEVICE_LIDAR_LOCKER_2560::deviceLoop()
{
	int i;
	uint8_t opeMode;
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
	updateLidar(m_pLidarUP, cosRoll*cosPitch);
	updateLidar(m_pLidarL, cosRoll);
	updateLidar(m_pLidarR, cosRoll);
	updateLidar(m_pLidarF, cosPitch);

	//TODO: reset diverge when distance is infinite

	//Main controller
	if (m_opeMode == OPE_REFERENCE_LOCK)
	{
		decideLidar(m_pLidarRoll, m_pLidarL, m_pLidarR);

		if (m_dTime > m_config.m_inputDtime)
		{
			//Roll Stick input
			updateStickInput(m_pLidarUP, *m_pPWMInThrottle);
//			updateStickInput(m_pLidarRoll, *m_pPWMInRoll);
//			updateStickInput(m_pLidarF, *m_pPWMInPitch);
		}

		referenceLock();
	}
	else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
	{
		collisionAvoid();
	}


	//Slow rate actions
	switch (m_counter)
	{
	case 1:
		if (m_opeMode == OPE_REFERENCE_LOCK)
		{
			m_pLidarR->m_setting.m_P = (((int)m_PPMInput.m_inputPPM[m_config.cAvoidALT_PPMIdx]) - 1500) * 1;
			m_pLidarR->m_setting.m_D = (((int)m_PPMInput.m_inputPPM[m_config.cAvoidROLL_PPMIdx]) - 1500) * 2;
			m_config.m_dTdist = abs(m_PPMInput.m_inputPPM[7] - 1000) * 0.005;
		}
		else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
		{
			//Update controller setting input
			//*0.001*4000 ,up to 40m
			m_config.lidarLim[m_config.m_ppmIdxThrottle] = abs(m_PPMInput.m_inputPPM[m_config.cAvoidALT_PPMIdx] - 1000) * 1;
			m_config.lidarLim[m_config.m_ppmIdxRoll] = abs(m_PPMInput.m_inputPPM[m_config.cAvoidROLL_PPMIdx] - 1000) * 1;
			m_config.lidarLim[m_config.m_ppmIdxPitch] = m_config.lidarLim[m_config.m_ppmIdxRoll];
		}
		break;

	case 2:
		//Update control mode
		opeMode = m_PPMInput.updateModeSwitch();
		if (opeMode != m_opeMode)
		{
			//Mode is changed, init the reference lock distance
			if (opeMode == OPE_REFERENCE_LOCK)
			{
				startRefLock();
			}
			else
			{
				SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
				SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
				SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxPitch);
			}

			m_opeMode = opeMode;
		}
		break;

	case 3:
		serialPrint();
		m_counter = 0;
		break;

	default:
		m_counter = 0;
		break;
	}
	m_counter++;

}

void DEVICE_LIDAR_LOCKER_2560::referenceLock()
{
	updateRefLockPWM(m_pLidarUP, m_pPWMOutThrottle);
	//TODO
	updateRefLockPWM(m_pLidarRoll, m_pPWMOutRoll);
//	updateRefLockPWM(m_pLidarF, m_pPWMOutPitch);
}

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
		m_PPMInput.m_PWMLimHigh[m_config.m_ppmIdxThrottle] = m_config.PWMCenter + m_config.PWMRange;
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
	}

	// Foward direction
	dist = m_pLidarF->m_distCM - m_config.lidarLim[m_config.m_ppmIdxPitch];
	//TODO: what if dist = 0?
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
		m_PPMInput.m_PWMLimLow[m_config.m_ppmIdxPitch] = m_config.PWMCenter - m_config.PWMRange;
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
		*m_pPWMOutRoll = m_config.PWMCenter + m_pLidarL->m_setting.m_cAvoidPWM;// m_config.cAvoidPWM[m_config.m_ppmIdxRoll];
	}
	else if (distR < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
		*m_pPWMOutRoll = m_config.PWMCenter + m_pLidarR->m_setting.m_cAvoidPWM;// m_config.cAvoidPWM[m_config.m_ppmIdxRoll];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
	}
	
}

void DEVICE_LIDAR_LOCKER_2560::decideLidar(LIDAR_UNIT* pUseLidar, LIDAR_UNIT* pLidar1, LIDAR_UNIT* pLidar2)
{
	m_pLidarRoll = m_pLidarR;
	/*	
	if (m_pLidarL->m_distCM < m_config.m_lidarRangeLim)
	{
		m_pLidarRoll = m_pLidarL;
	}
	else
	{
		m_pLidarRoll = m_pLidarR;
	}
	*/
}

void DEVICE_LIDAR_LOCKER_2560::lidarCalibration(LIDAR_UNIT* pLidar)
{
	int i;
	for (i = 0; i < LIDAR_CALIB_SAMPLE; i++)
	{
		updateLidar(pLidar, 1.0);
	}

	pLidar->m_setting.m_offset = constrain(pLidar->m_distCM-100,0,2000);

}

long DEVICE_LIDAR_LOCKER_2560::medianFilter(LIDAR_UNIT* pLidar)
{
	uint8_t i;
	long minv, midv, maxv, tmp;
	long r1, r2, r3;

	minv = pLidar->m_pHistory[0];
	midv = pLidar->m_pHistory[1];
	maxv = pLidar->m_pHistory[2];
	if (minv > midv)SWITCH(minv, midv, tmp);
	if (midv > maxv)SWITCH(midv, maxv, tmp);
	if (minv > midv)SWITCH(minv, midv, tmp);

	minv = midv;
	midv = pLidar->m_pHistory[3];
	maxv = pLidar->m_pHistory[4];
	if (minv > midv)SWITCH(minv, midv, tmp);
	if (midv > maxv)SWITCH(midv, maxv, tmp);
	if (minv > midv)SWITCH(minv, midv, tmp);

	return ((float)(midv+pLidar->m_distCM))*0.5;
/*
	minv = pLidar->m_pHistory[0];
	midv = pLidar->m_pHistory[1];
	maxv = pLidar->m_pHistory[2];
	if (minv > midv)SWITCH(minv, midv, tmp);
	if (midv > maxv)SWITCH(midv, maxv, tmp);
	if (minv > midv)SWITCH(minv, midv, tmp);
	r1 = midv;

	minv = pLidar->m_pHistory[3];
	midv = pLidar->m_pHistory[4];
	maxv = pLidar->m_pHistory[5];
	if (minv > midv)SWITCH(minv, midv, tmp);
	if (midv > maxv)SWITCH(midv, maxv, tmp);
	if (minv > midv)SWITCH(minv, midv, tmp);
	r2 = midv;

	minv = pLidar->m_pHistory[6];
	midv = pLidar->m_pHistory[7];
	maxv = pLidar->m_pHistory[8];
	if (minv > midv)SWITCH(minv, midv, tmp);
	if (midv > maxv)SWITCH(midv, maxv, tmp);
	if (minv > midv)SWITCH(minv, midv, tmp);
	r3 = midv;

	if (r1 > r2)SWITCH(r1, r2, tmp);
	if (r2 > r3)SWITCH(r2, r3, tmp);
	if (r1 > r2)SWITCH(r1, r2, tmp);
	return r2;
*/

}

void DEVICE_LIDAR_LOCKER_2560::updateLidar(LIDAR_UNIT* pLidar, float factor)
{
	float dist;
	
#ifdef LIDAR_CONTINUOUS
	dist = factor * ((float)m_LidarLite.distance(true, true, pLidar->m_setting.m_address));
#else
	dist = factor * ((float)m_LidarLite.distanceContinuous(pLidar->m_setting.m_address));
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
	pLidar->m_distCM = dist + (dist - (float)pLidar->m_distCM) * m_config.m_dTdist;
	pLidar->m_diverge *= m_config.m_divergeFactor;
	pLidar->m_diverge += abs(dist - pLidar->m_distCM);
}

void DEVICE_LIDAR_LOCKER_2560::updateRefLockPWM(LIDAR_UNIT* pLidar, uint16_t* pPWM)
{
	long err, pwm;

	err = pLidar->m_lockCM - pLidar->m_distCM;
	pLidar->m_integErr = constrain(pLidar->m_integErr + err, -m_config.m_errLim, m_config.m_errLim);
	pwm = m_config.PWMCenter +
		(long)((float)(pLidar->m_setting.m_P * err
		+ pLidar->m_setting.m_D * (err - pLidar->m_prevErr)
		+ constrain(pLidar->m_setting.m_I * pLidar->m_integErr,
		-pLidar->m_setting.m_Imax,
		pLidar->m_setting.m_Imax)) * m_config.m_pwmFactor);
	pwm = constrain(pwm, m_config.PWMLenFrom, m_config.PWMLenTo);
	pLidar->m_prevErr = err;
	*pPWM = pwm;
}


void DEVICE_LIDAR_LOCKER_2560::updateStickInput(LIDAR_UNIT* pLidar, int16_t PWM)
{
	long stickInput;

	stickInput = PWM - m_config.PWMCenter;
	if (abs(stickInput) > m_config.m_deadZone)
	{
		stickInput = (stickInput > 0) ? 1 : -1;
		pLidar->m_lockCM = constrain(pLidar->m_lockCM + pLidar->m_setting.m_dSpeed*stickInput,
			m_config.m_lidarRangeMin,
			m_config.m_lidarRangeMax);
	}

}

void DEVICE_LIDAR_LOCKER_2560::startRefLock(void)
{
	int i;
	for (i = 0; i < NUM_LIDAR; i++)
	{
		m_pLidar[i].m_lockCM = m_pLidar[i].m_distCM;
		m_pLidar[i].m_integErr = 0;
		m_pLidar[i].m_prevErr = 0;
	}

	SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxThrottle);
	SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxRoll);
	//TODO
	SETBIT_ON(m_PPMInput.m_bPPMthrough, m_config.m_ppmIdxPitch);
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

void DEVICE_LIDAR_LOCKER_2560::setDefaultParameters(void)
{
	m_config.vers = VERSION;

	m_config.PPMframeLength = 22500;
	m_config.PPMPulseLength = 500;
	m_config.PWMLenFrom = 1350;
	m_config.PWMLenTo = 1650;
	m_config.PWMRange = 500;
	m_config.PWMCenter = 1500;

	m_config.m_ppmIdxPitch = 1;
	m_config.m_ppmIdxRoll = 0;
	m_config.m_ppmIdxThrottle = 2;
	m_config.m_ppmIdxYaw = 3;
	m_config.m_ppmIdxMode = 4;// 7;

	m_config.lidarLim[m_config.m_ppmIdxRoll] = 0;
	m_config.lidarLim[m_config.m_ppmIdxPitch] = 0;
	m_config.lidarLim[m_config.m_ppmIdxThrottle] = 0;

	m_config.cAvoidALT_PPMIdx = 5;
	m_config.cAvoidROLL_PPMIdx = 6;

	m_config.m_errLim = 10000;
	m_config.m_lidarRangeMin = 150;
	m_config.m_lidarRangeMax = 3000;
	m_config.m_deadZone = 100;
	m_config.m_divergeFactor = 0.9;
	m_config.m_pwmFactor = 0.01;
	m_config.m_inputDtime = 10000;	//1000 usec, 

	//Filter
	m_config.m_filterWindow = 5;// 9;
	m_config.m_dTdist = 0.25;

	//Lidar
	m_config.m_LidarIdxUP = 3;
	m_config.m_LidarIdxL = 2;
	m_config.m_LidarIdxR = 1;
	m_config.m_LidarIdxF = 0;
	m_config.m_LidarIdxB = 0;

	m_config.lidar[m_config.m_LidarIdxUP].m_P = -100;
	m_config.lidar[m_config.m_LidarIdxUP].m_I = 0;
	m_config.lidar[m_config.m_LidarIdxUP].m_Imax = 500;
	m_config.lidar[m_config.m_LidarIdxUP].m_D = 0;
	m_config.lidar[m_config.m_LidarIdxUP].m_criticalRegion = 500;
	m_config.lidar[m_config.m_LidarIdxUP].m_dSpeed = -1;
	m_config.lidar[m_config.m_LidarIdxUP].m_address = 0x70;
	m_config.lidar[m_config.m_LidarIdxUP].m_pinEN = A3;
	m_config.lidar[m_config.m_LidarIdxUP].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxUP].m_cAvoidPWM = -65;

	m_config.lidar[m_config.m_LidarIdxL].m_P = 5;
	m_config.lidar[m_config.m_LidarIdxL].m_I = 0;
	m_config.lidar[m_config.m_LidarIdxL].m_Imax = 50;
	m_config.lidar[m_config.m_LidarIdxL].m_D = 0;
	m_config.lidar[m_config.m_LidarIdxL].m_criticalRegion = 500;
	m_config.lidar[m_config.m_LidarIdxL].m_dSpeed = 1;
	m_config.lidar[m_config.m_LidarIdxL].m_address = 0x68;
	m_config.lidar[m_config.m_LidarIdxL].m_pinEN = A2;
	m_config.lidar[m_config.m_LidarIdxL].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxL].m_cAvoidPWM = 80;

	m_config.lidar[m_config.m_LidarIdxR].m_P = 5;
	m_config.lidar[m_config.m_LidarIdxR].m_I = 0;// 15;
	m_config.lidar[m_config.m_LidarIdxR].m_Imax = 8000;
	m_config.lidar[m_config.m_LidarIdxR].m_D = 0;
	m_config.lidar[m_config.m_LidarIdxR].m_criticalRegion = 500;
	m_config.lidar[m_config.m_LidarIdxR].m_dSpeed = -1;
	m_config.lidar[m_config.m_LidarIdxR].m_address = 0x64;
	m_config.lidar[m_config.m_LidarIdxR].m_pinEN = A1;
	m_config.lidar[m_config.m_LidarIdxR].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxR].m_cAvoidPWM = -70;

	m_config.lidar[m_config.m_LidarIdxF].m_P = 5;
	m_config.lidar[m_config.m_LidarIdxF].m_I = 0;
	m_config.lidar[m_config.m_LidarIdxF].m_Imax = 50;
	m_config.lidar[m_config.m_LidarIdxF].m_D = 0;
	m_config.lidar[m_config.m_LidarIdxF].m_criticalRegion = 500;
	m_config.lidar[m_config.m_LidarIdxF].m_dSpeed = 1;
	m_config.lidar[m_config.m_LidarIdxF].m_address = 0x66;
	m_config.lidar[m_config.m_LidarIdxF].m_pinEN = A0;
	m_config.lidar[m_config.m_LidarIdxF].m_offset = 0;
	m_config.lidar[m_config.m_LidarIdxF].m_cAvoidPWM = 85;

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
			
			long err;
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

			m_pUSBSerial->print(", P_R:");
			m_pUSBSerial->print(m_pLidarR->m_setting.m_P);
			m_pUSBSerial->print(", D_R:");
			m_pUSBSerial->println(m_pLidarR->m_setting.m_D);

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
