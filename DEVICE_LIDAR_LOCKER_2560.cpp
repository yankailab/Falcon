#include "DEVICE_LIDAR_LOCKER_2560.h"

void DEVICE_LIDAR_LOCKER_2560::referenceLock()
{
	long err, pwm;

	// UP direction
	err = m_pLidarUP->m_lockCM - m_pLidarUP->m_distCM;
	m_pLidarUP->m_integErr = constrain(m_pLidarUP->m_integErr + err, -m_config.m_errLim, m_config.m_errLim);
	pwm = m_config.PWMCenter +
		m_pLidarUP->m_setting.m_P * err
		+ m_pLidarUP->m_setting.m_D * (err - m_pLidarUP->m_prevErr)
		+ constrain(m_pLidarUP->m_setting.m_I * m_pLidarUP->m_integErr, 
					-m_pLidarUP->m_setting.m_Imax,
					m_pLidarUP->m_setting.m_Imax);
	pwm = (long)((float)pwm * m_config.m_pwmFactor);
	pwm = constrain(pwm, m_config.PWMLenFrom, m_config.PWMLenTo);
	m_pLidarUP->m_prevErr = err;
	m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmTHROTTLE] = pwm;
	
	// Roll Axis
	err = m_pLidarRoll->m_lockCM - m_pLidarRoll->m_distCM;
	m_pLidarRoll->m_integErr = constrain(m_pLidarRoll->m_integErr + err, -m_config.m_errLim, m_config.m_errLim);
	pwm = m_config.PWMCenter +
		m_pLidarRoll->m_setting.m_P * err
		+ m_pLidarRoll->m_setting.m_D * (err - m_pLidarRoll->m_prevErr)
		+ constrain(m_pLidarRoll->m_setting.m_I * m_pLidarRoll->m_integErr,
					-m_pLidarRoll->m_setting.m_Imax,
					m_pLidarRoll->m_setting.m_Imax);
	pwm = (long)((float)pwm * m_config.m_pwmFactor);
	pwm = constrain(pwm, m_config.PWMLenFrom, m_config.PWMLenTo);
	m_pLidarRoll->m_prevErr = err;
	m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = pwm;

}

void DEVICE_LIDAR_LOCKER_2560::collisionAvoid()
{
	long distL, distR, distUP;
	
	// UP direction
	distUP = m_pLidarUP->m_distCM - m_config.lidarLim[m_PPMInput.m_ppmTHROTTLE];
	if (distUP < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmTHROTTLE);
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmTHROTTLE] = 1500 - m_config.cAvoidPWM[m_PPMInput.m_ppmTHROTTLE];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmTHROTTLE);
	}

	//Roll Axis
	distL = m_pLidarL->m_distCM - m_config.lidarLim[m_PPMInput.m_ppmROLL];
	distR = m_pLidarR->m_distCM - m_config.lidarLim[m_PPMInput.m_ppmROLL];

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
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = m_config.PWMCenter + m_config.cAvoidPWM[m_PPMInput.m_ppmROLL];
	}
	else if (distR < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = m_config.PWMCenter - m_config.cAvoidPWM[m_PPMInput.m_ppmROLL];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
	}
	
}


void DEVICE_LIDAR_LOCKER_2560::deviceSetup(void)
{
	uint8_t i;

	m_bPrintOut = true;// false;
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
	// Array of pins connected to the sensor Power Enable lines
	int lidarPins[] = { A0, A1, A2 };
	unsigned char lidarAddresses[] = { 0x66, 0x64, 0x68 };

	m_LidarLite.begin();
	m_LidarLite.changeAddressMultiPwrEn(NUM_LIDAR, lidarPins, lidarAddresses, false);
	for (i = 0; i < NUM_LIDAR; i++)
	{
		m_pLidar[i].m_setting.m_address = lidarAddresses[i];
		m_pLidar[i].m_setting.m_pinEN = lidarPins[i];
		m_pLidar[i].m_distCM = 0;
		m_pLidar[i].m_diverge = 0;
		m_pLidar[i].m_bLocked = false;

		pinMode(lidarPins[i], OUTPUT);
	}

	m_pLidarUP = &m_pLidar[0];
	m_pLidarL = &m_pLidar[1];
	m_pLidarR = &m_pLidar[2];
	
	//Init PPM input
	m_PPMInput.m_ppmROLL = m_config.m_ppmIdxRoll;
	m_PPMInput.m_ppmPITCH = m_config.m_ppmIdxPitch;
	m_PPMInput.m_ppmTHROTTLE = m_config.m_ppmIdxThrottle;
	m_PPMInput.m_ppmMODE = m_config.m_ppmIdxMode;
	m_PPMInput.m_pOpeMode = &m_opeMode;
	m_PPMInput.init();

	//Init system mode
	m_opeMode = OPE_PPM_THROUGH;

	m_pUSBSerial->println(F("FALCON_START"));

}


void DEVICE_LIDAR_LOCKER_2560::deviceLoop()
{
	int i;
	uint8_t opeMode;
	float cosRoll, cosPitch;
	float dist;
	long stickInput;

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
	dist = cosRoll * cosPitch * ((float)m_LidarLite.distance(true, true, m_pLidarUP->m_setting.m_address));
	m_pLidarUP->m_distCM = (dist + (float)m_pLidarUP->m_distCM) * 0.5;
	m_pLidarUP->m_diverge *= m_config.m_divergeFactor;
	m_pLidarUP->m_diverge += abs(dist - m_pLidarUP->m_distCM);

	dist = cosRoll * ((float)m_LidarLite.distance(true, true, m_pLidarL->m_setting.m_address));
	m_pLidarL->m_distCM = (dist + (float)m_pLidarL->m_distCM) * 0.5;
	m_pLidarL->m_diverge *= m_config.m_divergeFactor;
	m_pLidarL->m_diverge += abs(dist - m_pLidarL->m_distCM);

	dist = cosRoll * ((float)m_LidarLite.distance(true, true, m_pLidarR->m_setting.m_address));
	m_pLidarR->m_distCM = (dist + (float)m_pLidarR->m_distCM) * 0.5;
	m_pLidarR->m_diverge *= m_config.m_divergeFactor;
	m_pLidarR->m_diverge += abs(dist - m_pLidarR->m_distCM);

	//TODO: reset diverge when distance is infinite
	//TODO: detect Lidar Failure

	//Main controller
	if (m_opeMode == OPE_REFERENCE_LOCK)
	{
		decideRollLidar();

		if (m_dTime > m_config.m_inputDtime)
		{
			//Roll Stick input
			stickInput = m_PPMInput.m_inputPPM[m_PPMInput.m_ppmROLL] - m_config.PWMCenter;
			if (abs(stickInput) > m_config.m_deadZone)
			{
				stickInput = (stickInput > 0) ? 1 : -1;
				m_pLidarRoll->m_lockCM = constrain(m_pLidarRoll->m_lockCM + m_config.m_rollDSpeed*stickInput,
					m_config.m_lidarRangeMin,
					m_config.m_lidarRangeMax);
			}

			//Throttle Stick input
			stickInput = m_PPMInput.m_inputPPM[m_PPMInput.m_ppmTHROTTLE] - m_config.PWMCenter;
			if (abs(stickInput) > m_config.m_deadZone)
			{
				stickInput = (stickInput > 0) ? 1 : -1;
				m_pLidarUP->m_lockCM = constrain(m_pLidarUP->m_lockCM + m_config.m_altDSpeed*stickInput,
					m_config.m_lidarRangeMin,
					m_config.m_lidarRangeMax);
			}
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
	case 0:
		//Update controller setting input
		//*0.001*4000 ,up to 40m
		//TODO
		m_config.lidarLim[m_PPMInput.m_ppmTHROTTLE] = 250;// abs(m_PPMInput.m_inputPPM[5] - 1000) * 1;
		m_config.lidarLim[m_PPMInput.m_ppmROLL] = 250;// abs(m_PPMInput.m_inputPPM[6] - 1000) * 1;
		break;

	case 1:
		//Update control mode
		opeMode = m_PPMInput.updateModeSwitch();
		if (opeMode != m_opeMode)
		{
			//Mode is changed, init the reference lock distance
			if (opeMode == OPE_REFERENCE_LOCK)
			{
				for (i = 0; i < NUM_LIDAR; i++)
				{
					m_pLidar[i].m_lockCM = m_pLidar[i].m_distCM;
					m_pLidar[i].m_integErr = 0;
					m_pLidar[i].m_prevErr = 0;
				}

				SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmTHROTTLE);
				SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
			}
			else
			{
				SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmTHROTTLE);
				SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
			}

			m_opeMode = opeMode;
		}
		break;

	case 2:
		serialPrint();
		break;

	default:
		m_counter = 0;
		break;
	}
	m_counter++;

}

void DEVICE_LIDAR_LOCKER_2560::decideRollLidar()
{
	m_pLidarRoll = m_pLidarL;
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

void DEVICE_LIDAR_LOCKER_2560::setDefaultParameters(void)
{
	m_config.vers = VERSION;

	m_config.PPMframeLength = 22500;
	m_config.PPMPulseLength = 500;
	m_config.PWMLenFrom = 1250;
	m_config.PWMLenTo = 1750;
	m_config.PWMCenter = 1500;

	m_config.m_ppmIdxPitch = 1;
	m_config.m_ppmIdxRoll = 0;
	m_config.m_ppmIdxThrottle = 2;
	m_config.m_ppmIdxYaw = 3;
	m_config.m_ppmIdxMode = 1;// 7;//TODO:temporal test

	m_config.lidarLim[m_config.m_ppmIdxRoll] = 0;
	m_config.lidarLim[m_config.m_ppmIdxPitch] = 0;
	m_config.lidarLim[m_config.m_ppmIdxThrottle] = 0;

	m_config.cAvoidPWM[m_config.m_ppmIdxRoll] = 60;
	m_config.cAvoidPWM[m_config.m_ppmIdxPitch] = 60;
	m_config.cAvoidPWM[m_config.m_ppmIdxThrottle] = 60;
	m_config.PWM_THR_UP_Lim = 1580;

	m_config.m_lidarRangeMin = 100;
	m_config.m_lidarRangeMax = 3000;
	m_config.m_rollDSpeed = 1;
	m_config.m_altDSpeed = 1;
	m_config.m_deadZone = 100;
	m_config.m_divergeFactor = 0.9;
	m_config.m_pwmFactor = 1.0;
	m_config.m_inputDtime = 10000;	//1000 usec, 

	for (int i = 0; i < NUM_LIDAR; i++)
	{
		m_pLidar[i].m_setting.m_P = 1;
		m_pLidar[i].m_setting.m_I = 0;
		m_pLidar[i].m_setting.m_Imax = 50;
		m_pLidar[i].m_setting.m_D = 0;
	}

	
}

void DEVICE_LIDAR_LOCKER_2560::serialPrint()
{
	if (m_bPrintOut)
	{
		if (m_opeMode == OPE_REFERENCE_LOCK)
		{
			m_pUSBSerial->print("[REF LOCK] L:");
			m_pUSBSerial->print(m_pLidarL->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarL->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarL->m_diverge);
			m_pUSBSerial->print("),  UP:");
			m_pUSBSerial->print(m_pLidarUP->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarUP->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarUP->m_diverge);
			m_pUSBSerial->print("),  R:");
			m_pUSBSerial->print(m_pLidarR->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_pLidarR->m_lockCM);
			m_pUSBSerial->print("(");
			m_pUSBSerial->print(m_pLidarR->m_diverge);

			m_pUSBSerial->print("), Pitch:");
			m_pUSBSerial->print(m_IMU.m_ypr[2]);
			m_pUSBSerial->print(", Roll:");
			m_pUSBSerial->print(m_IMU.m_ypr[1]);

			m_pUSBSerial->print("PWM: Thr:");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmTHROTTLE]);
			m_pUSBSerial->print(", Roll:");
			m_pUSBSerial->println(m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL]);

		}
		else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
		{
			m_pUSBSerial->print("[COLLISION AVOID] L:");
			m_pUSBSerial->print(m_pLidarL->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_PPMInput.m_ppmROLL]);
			m_pUSBSerial->print(", UP:");
			m_pUSBSerial->print(m_pLidarUP->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_PPMInput.m_ppmTHROTTLE]);
			m_pUSBSerial->print(", R:");
			m_pUSBSerial->print(m_pLidarR->m_distCM);
			m_pUSBSerial->print("/");
			m_pUSBSerial->print(m_config.lidarLim[m_PPMInput.m_ppmROLL]);

			m_pUSBSerial->print(", Pitch:");
			m_pUSBSerial->print(m_IMU.m_ypr[2]);
			m_pUSBSerial->print(", Roll:");
			m_pUSBSerial->print(m_IMU.m_ypr[1]);

			m_pUSBSerial->print("  PWM: Thr:");
			m_pUSBSerial->print(m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmTHROTTLE]);
			m_pUSBSerial->print(" Roll:");
			m_pUSBSerial->println(m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL]);
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