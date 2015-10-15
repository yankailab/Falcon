#include "DEVICE_LIDAR_LOCKER_2560.h"

void DEVICE_LIDAR_LOCKER_2560::collisionAvoid()
{
	long distL, distR, distUP;
	
	// UP direction
	g_LidarUP.m_distCM = pulseIn(g_LidarUP.m_pinPWM, HIGH, LD_TIMEOUT)*0.1;
	if (g_LidarUP.m_distCM == 0)
	{
	resetLidar(g_LidarUP.m_pinTrigger);
	}
	else
	{
	distUP = g_LidarUP.m_distCM - config.lidarLim[g_ppmTHROTTLE];
	if (distUP < 0)
	{
	SETBIT_OFF(g_bPPMthrough, g_ppmTHROTTLE);
	g_ppm[g_ppmTHROTTLE] = 1500 - config.cAvoidPWM[g_ppmTHROTTLE];
	}
	else
	{
	SETBIT_ON(g_bPPMthrough, g_ppmTHROTTLE);
	}
	}


	if (g_opeMode == OPE_ALL_COLLISION_AVOID)
	{
	// Update Lidar sensors, pulsedIn in microseconds, 10 usec = 1cm
	g_LidarL.m_distCM = pulseIn(g_LidarL.m_pinPWM, HIGH, LD_TIMEOUT)*0.1;
	if (g_LidarL.m_distCM == 0)
	{
	resetLidar(g_LidarL.m_pinTrigger);
	return;
	}

	g_LidarR.m_distCM = pulseIn(g_LidarR.m_pinPWM, HIGH, LD_TIMEOUT)*0.1;
	if (g_LidarR.m_distCM == 0)
	{
	resetLidar(g_LidarR.m_pinTrigger);
	return;
	}

	//Update Attitude, TODO: change to using IMU
	//		g_attitude[PITCH] = cos(abs(g_ppm[config.controlChannel[PITCH].ppmIdx] - 1500)*0.002);
	//		g_attitude[ROLL] = cos(abs(g_ppm[config.controlChannel[ROLL].ppmIdx] - 1500)*0.002);
	//pLidarL->m_distCM *= g_attitude[ROLL];
	//pLidarR->m_distCM *= g_attitude[ROLL];
	//		pLidarUP->m_distCM *= g_attitude[PITCH];

	//Roll Axis
	distL = g_LidarL.m_distCM - config.lidarLim[g_ppmROLL];
	distR = g_LidarR.m_distCM - config.lidarLim[g_ppmROLL];

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
	SETBIT_OFF(g_bPPMthrough, g_ppmROLL);
	g_ppm[g_ppmROLL] = 1500 + config.cAvoidPWM[g_ppmROLL];
	}
	else if (distR < 0)
	{
	SETBIT_OFF(g_bPPMthrough, g_ppmROLL);
	g_ppm[g_ppmROLL] = 1500 - config.cAvoidPWM[g_ppmROLL];
	}
	else
	{
	SETBIT_ON(g_bPPMthrough, g_ppmROLL);
	}

	}

	
}



void DEVICE_LIDAR_LOCKER_2560::deviceSetup(config_t* pConfig)
{
	uint8_t i;

	m_bPrintIMU = true;// false;
	m_bBootSuccess = true;
	m_bHostConnected = false;
	m_pConfig = pConfig;

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
	
	//Init PPM input
	m_PPMInput.init(pConfig);

	//Init system mode
	m_opeMode = OPE_PPM_THROUGH;

	m_pUSBSerial->println(F("FALCON_START"));

}



void DEVICE_LIDAR_LOCKER_2560::deviceLoop()
{
	int i;

	if (!m_bBootSuccess)
	{
		m_pUSBSerial->println(F("FALCON_FAIL"));
		return;
	}

	if (m_bMpuInterrupt || (m_IMU.m_fifoCount >= m_IMU.m_packetSize))
	{
		// reset interrupt flag and get INT_STATUS byte
		m_bMpuInterrupt = false;
		m_IMU.update();
		//		g_Controller.updateAttitude(PITCH, g_IMU.m_ypr[2]);
		//		g_Controller.updateAttitude(ROLL, g_IMU.m_ypr[1]);
	}

	//Fast functions
	m_PPMInput.updateSwitch();

	//Update controller setting input
	//*0.001*4000 ,up to 40m
	//	config.lidarLim[g_ppmTHROTTLE] = abs(g_inputPPM[5] - 1000) * 1;
	//	config.lidarLim[g_ppmROLL] = abs(g_inputPPM[6] - 1000) * 1;

	//Collision avoid mode
	if (m_opeMode == OPE_UP_COLLISION_AVOID)
	{
		collisionAvoid();
	}
	else if (m_opeMode == OPE_ALL_COLLISION_AVOID)
	{
		collisionAvoid();
	}




	// slow rate actions
	switch (m_counter)
	{

	case 1:
		//		g_Controller.updateThrottle();

		if (m_bPrintIMU)
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

			m_pUSBSerial->print("Sensor 0x66: ");
			m_pUSBSerial->print(m_LidarLite.distance(true, true, 0x66));
			m_pUSBSerial->print(", Sensor 0x68: ");
			m_pUSBSerial->print(m_LidarLite.distance(true, true, 0x68));
			m_pUSBSerial->print(", Sensor 0x64: ");
			m_pUSBSerial->print(m_LidarLite.distance(true, true, 0x64));

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
		
			/*			g_pUSBSerial->print("\tpre:\t");
			g_pUSBSerial->print(g_FSLP.m_pressure);
			g_pUSBSerial->print("\tpos:\t");
			g_pUSBSerial->print(g_FSLP.m_position);
			g_pUSBSerial->print("\tThr:\t");
			g_pUSBSerial->println(g_Controller.m_throttle);
			*/
		}

		m_counter = 0;
		break;

	default:
		break;
	}
	m_counter++;

}
