#pragma once
#include "falconcommon.h"

#define SWITCH_LOW 1250
#define SWITCH_MID 1650

/*
Channel 6: Up distance
Channel 7: Left/Right distance
*/

class PPMInput
{
public:
	void init(void);
	void ppmInt(void);
	uint8_t updateModeSwitch(void);

	volatile uint8_t m_val;
	volatile unsigned long m_timeNow;
	volatile unsigned long m_timeOld;
	volatile unsigned int m_ppmIdx;
	volatile unsigned int m_pulseLength;



	//Switches
	int16_t m_prevModeSwitch;
	int16_t m_inputPPM[RC_CHANNEL_NUM];
	int16_t* m_pModeSwitch;
	int8_t* m_pOpeMode;

	uint8_t m_ppmROLL;
	uint8_t m_ppmPITCH;
	uint8_t m_ppmTHROTTLE;
	uint8_t m_ppmMODE;

	//8 channels currently
	uint16_t m_bPPMthrough;
	uint16_t* m_pPPMOut;

};

