#pragma once
#include "falconcommon.h"
#include "variables.h"

#define SWITCH_LOW 1250
#define SWITCH_MID 1650




class PPMInput
{
public:
	void init(config_t* pConfig);
	void ppmInt();
	void updateSwitch();

	volatile uint8_t m_val;
	volatile unsigned long m_timeNow;
	volatile unsigned long m_timeOld;
	volatile unsigned int m_ppmIdx;
	volatile unsigned int m_pulseLength;

	//Switches
	/*
	Channel 6: Up distance
	Channel 7: Left/Right distance
	*/
	uint16_t m_inputPPMSwitch;
	uint16_t m_inputPPM[RC_CHANNEL_NUM];
	uint16_t* m_pPPMSwitch;

	uint8_t m_ppmROLL;
	uint8_t m_ppmPITCH;
	uint8_t m_ppmTHROTTLE;

	//8 channels currently
	uint16_t m_bPPMthrough;
	uint16_t* m_pPPMOut;

};

