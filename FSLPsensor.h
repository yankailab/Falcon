#pragma once
#include <Arduino.h>
/*
const int fslpSenseLine = A2;//A1;//A0;
const int fslpDriveLine1 = 15;//A2;//12;
const int fslpDriveLine2 = A3;//A3;//A2;
const int fslpBotR0 = 14;//A0;//13;
*/
/*
const int fslpSenseLine = A2;
const int fslpDriveLine1 = A1;
const int fslpDriveLine2 = A0;
const int fslpBotR0 = A3;
*/
/*
A0;
16;
A1;
10;
*/

class FSLPsensor
{
public:
	FSLPsensor(void);
	~FSLPsensor(void);

	void setup(int SL,int DL1, int DL2,int BotR);
	void update(void);
	void analogReset(void);
	int fslpGetPressure(void);
	int fslpGetPosition(void);

	int	m_pressure;
	int	m_position;
	int m_lastPosition;	//Sensor position

//	long m_Throttle;
	bool m_bTouch;

private:
	int m_SL;
	int m_DL1;
	int m_DL2;
	int m_BotR;

};

