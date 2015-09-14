#pragma once

#include <Arduino.h>
#include "FSLPsensor.h"
#include "variables.h"
#include "VehicleLink.h"

class Controller
{
public:
	Controller(void);
	~Controller(void);

	void updateThrottle(void);
	void updateYaw(void);
	void updateAttitude(byte iChannel, float val);

	int16_t m_ypr[3];
	int16_t m_throttle;
	int16_t m_yaw;
	uint16_t m_button;
	uint8_t* m_pOpeMode;

	VehicleLink* m_pVLink;
	config_t* m_pConfig;
	FSLPsensor* m_pfslpL;
	FSLPsensor* m_pfslpR;
	FSLPChannel* m_pThrottleChannel;
	FSLPChannel* m_pYawChannel;

	bool m_modeChangedL;
	bool m_modeChangedR;

	uint16_t m_RSSI;
};

