#include "DEVICE_LIDAR_LOCKER_2560.h"


void DEVICE_LIDAR_LOCKER_2560::referenceLock()
{
	long err, pwm;

	// UP direction
	err = m_pLidarUP->m_lockCM - m_pLidarUP->m_distCM;
	m_pLidarUP->m_integErr += err;
	pwm = PWM_CENTER +
		m_pLidarUP->m_P * err
		+ m_pLidarUP->m_D * (err - m_pLidarUP->m_prevErr)
		+ constrain(m_pLidarUP->m_I * m_pLidarUP->m_integErr, m_Imax, -m_Imax);
	constrain(pwm, PWM_LOW, PWM_HIGH);
	m_pLidarUP->m_prevErr = err;

	m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmTHROTTLE] = pwm;


	//Roll Axis
	distL = m_pLidarL->m_distCM - m_pConfig->lidarLim[m_PPMInput.m_ppmROLL];
	distR = m_pLidarR->m_distCM - m_pConfig->lidarLim[m_PPMInput.m_ppmROLL];

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
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = 1500 + m_pConfig->cAvoidPWM[m_PPMInput.m_ppmROLL];
	}
	else if (distR < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = 1500 - m_pConfig->cAvoidPWM[m_PPMInput.m_ppmROLL];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
	}
}

void DEVICE_LIDAR_LOCKER_2560::collisionAvoid()
{
	long distL, distR, distUP;
	
	// UP direction
	distUP = m_pLidarUP->m_distCM - m_pConfig->lidarLim[m_PPMInput.m_ppmTHROTTLE];
	if (distUP < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmTHROTTLE);
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmTHROTTLE] = 1500 - m_pConfig->cAvoidPWM[m_PPMInput.m_ppmTHROTTLE];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmTHROTTLE);
	}

	//Roll Axis
	distL = m_pLidarL->m_distCM - m_pConfig->lidarLim[m_PPMInput.m_ppmROLL];
	distR = m_pLidarR->m_distCM - m_pConfig->lidarLim[m_PPMInput.m_ppmROLL];

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
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = 1500 + m_pConfig->cAvoidPWM[m_PPMInput.m_ppmROLL];
	}
	else if (distR < 0)
	{
		SETBIT_OFF(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
		m_PPMInput.m_pPPMOut[m_PPMInput.m_ppmROLL] = 1500 - m_pConfig->cAvoidPWM[m_PPMInput.m_ppmROLL];
	}
	else
	{
		SETBIT_ON(m_PPMInput.m_bPPMthrough, m_PPMInput.m_ppmROLL);
	}
	
}



void DEVICE_LIDAR_LOCKER_2560::deviceSetup(config_t* pConfig)
{
	uint8_t i;

	m_bPrintIMU = true;// false;
	m_bBootSuccess = true;
	m_bHostConnected = false;
	m_pConfig = pConfig;
	m_counter = 0;

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
	unsigned char lidarAddresses[] = { 0x66, 0x68, 0x64 };

	m_LidarLite.begin();
	m_LidarLite.changeAddressMultiPwrEn(3, lidarPins, lidarAddresses, false);
	for (i = 0; i < 3; i++)
	{
		m_pLidar[i].m_address = lidarAddresses[i];
		m_pLidar[i].m_pinEN = lidarPins[i];
		pinMode(lidarPins[i], OUTPUT);
	}

	m_pLidarUP = &m_pLidar[0];
	m_pLidarL = &m_pLidar[1];
	m_pLidarR = &m_pLidar[2];
	
	//Init PPM input
	m_PPMInput.init(pConfig);

	//Init system mode
	m_opeMode = OPE_PPM_THROUGH;

	m_pUSBSerial->println(F("FALCON_START"));

}



void DEVICE_LIDAR_LOCKER_2560::deviceLoop()
{
	int i;
	uint8_t opeMode;

	if (!m_bBootSuccess)
	{
		m_pUSBSerial->println(F("FALCON_FAIL"));
		return;
	}

	//Update IMU
	if (m_bMpuInterrupt || (m_IMU.m_fifoCount >= m_IMU.m_packetSize))
	{
		// reset interrupt flag and get INT_STATUS byte
		m_bMpuInterrupt = false;
		m_IMU.update();
	}

	//Update Lidars
	for (i = 0; i < 3; i++)
	{
		//m_IMU.m_ypr[0];
		//TODO: detect Lidar Failure
		//TODO: add Filter, calculate using gyro
		m_pLidar[i].m_distCM = m_LidarLite.distance(true, true, m_pLidar[i].m_address);
		//Update Attitude, TODO: change to using IMU
		//		g_attitude[PITCH] = cos(abs(g_ppm[config.controlChannel[PITCH].ppmIdx] - 1500)*0.002);
		//		g_attitude[ROLL] = cos(abs(g_ppm[config.controlChannel[ROLL].ppmIdx] - 1500)*0.002);
		//pLidarL->m_distCM *= g_attitude[ROLL];
		//pLidarR->m_distCM *= g_attitude[ROLL];
		//		pLidarUP->m_distCM *= g_attitude[PITCH];
	}

	//Update controller setting input
	//*0.001*4000 ,up to 40m
	m_pConfig->lidarLim[m_PPMInput.m_ppmTHROTTLE] = abs(m_PPMInput.m_inputPPM[5] - 1000) * 1;
	m_pConfig->lidarLim[m_PPMInput.m_ppmROLL] = abs(m_PPMInput.m_inputPPM[6] - 1000) * 1;

	//Update control mode
	opeMode = m_PPMInput.updateSwitch();
	if (opeMode != m_opeMode)
	{
		//Mode is changed, init the reference lock distance
		if (opeMode == OPE_REFERENCE_LOCK)
		{
			for (i = 0; i < 3; i++)
			{
				m_pLidar[i].m_lockCM = m_pLidar[i].m_distCM;
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

	//Main controller
	if (m_opeMode == OPE_REFERENCE_LOCK)
	{
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
		if (m_bPrintIMU)
		{
			m_pUSBSerial->print("Sensor 0x66: ");
			m_pUSBSerial->print(m_pLidar[0].m_distCM);
			m_pUSBSerial->print(", Sensor 0x68: ");
			m_pUSBSerial->print(m_pLidar[1].m_distCM);
			m_pUSBSerial->print(", Sensor 0x64: ");
			m_pUSBSerial->print(m_pLidar[2].m_distCM);

			m_pUSBSerial->print("   ypr\t");
			m_pUSBSerial->print(m_IMU.m_ypr[0]);
			m_pUSBSerial->print("\t");
			m_pUSBSerial->print(m_IMU.m_ypr[1]);
			m_pUSBSerial->print("\t");
			m_pUSBSerial->print(m_IMU.m_ypr[2]);

			m_pUSBSerial->print(" ");
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

		m_counter = 0;
		break;

	default:
		break;
	}
	m_counter++;

}
