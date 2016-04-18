#ifndef __PPM_OUTPUT__
#define __PPM_OUTPUT__

#include "falconcommon.h"

#ifdef ATMEGA_A328
	#define DEFAULT_PWM 1500
#endif

#ifdef ATMEGA_A32U4
	#define DEFAULT_PWM 1500
#endif

//this array holds the servo values for the ppm signal
//change theese values in your code (usually servo values are between 1000 and 2000)
uint16_t g_ppm[RC_CHANNEL_NUM];
uint16_t PPM_FrLen;			//22500		//set the PPM frame length in microseconds (1ms = 1000µs)
uint16_t PPM_PulseLen;		//500		//set the pulse length

#ifdef ATMEGA_A328
ISR(TIMER1_COMPA_vect)
{
	static boolean state = true;
	TCNT1 = 0;

	if (state)
	{	//start pulse
		digitalWrite(PPM_OUTPUT_PIN, PPM_ON_STATE);
		OCR1A = PPM_PulseLen * 2;
		state = false;
	}
	else
	{
		//end pulse and calculate when to start the next pulse
		static byte			cur_chan_numb;
		static unsigned int calc_rest;

		digitalWrite(PPM_OUTPUT_PIN, !PPM_ON_STATE);
		state = true;

		if (cur_chan_numb >= RC_CHANNEL_NUM)
		{
			cur_chan_numb = 0;
			calc_rest = calc_rest + PPM_PulseLen;
			OCR1A = (PPM_FrLen - calc_rest) * 2;
			calc_rest = 0;
		}
		else
		{
			OCR1A = (g_ppm[cur_chan_numb] - PPM_PulseLen) * 2;
			calc_rest = calc_rest + g_ppm[cur_chan_numb];
			cur_chan_numb++;
		}
	}
}
#endif

#ifdef ATMEGA_A32U4
ISR(TIMER3_COMPA_vect)
{
	static boolean state = true;
	TCNT3 = 0;

	if (state)
	{	//start pulse
		digitalWrite(PPM_OUTPUT_PIN, PPM_ON_STATE);
		OCR3A = PPM_PulseLen * 2;
		state = false;
	}
	else
	{
		//end pulse and calculate when to start the next pulse
		static byte			cur_chan_numb;
		static unsigned int calc_rest;

		digitalWrite(PPM_OUTPUT_PIN, !PPM_ON_STATE);
		state = true;

		if (cur_chan_numb >= RC_CHANNEL_NUM)
		{
			cur_chan_numb = 0;
			calc_rest = calc_rest + PPM_PulseLen;
			OCR3A = (PPM_FrLen - calc_rest) * 2;
			calc_rest = 0;
		}
		else
		{
			OCR3A = (g_ppm[cur_chan_numb] - PPM_PulseLen) * 2;
			calc_rest = calc_rest + g_ppm[cur_chan_numb];
			cur_chan_numb++;
		}
	}
}

#endif

void PPM_init(int FrLen, int PulseLen)
{
	int i;

	//initiallize default ppm values
	for(i = 0; i < RC_CHANNEL_NUM; i++)
	{
		g_ppm[i] = DEFAULT_PWM;
	}

	pinMode(PPM_OUTPUT_PIN, OUTPUT);
	digitalWrite(PPM_OUTPUT_PIN, !PPM_ON_STATE); //set the PPM signal pin to the default state (off)
	
	PPM_FrLen = FrLen;// config.PPMframeLength;
	PPM_PulseLen = PulseLen;// config.PPMPulseLength;


#ifdef ATMEGA_A328
	cli();
	TCCR1A = 0;						// set entire TCCR1 register to 0
	TCCR1B = 0;
	TCNT1 = 0;

	OCR1A = 100;					// compare match register, change this
	TCCR1B |= (1 << WGM12);			// turn on CTC mode
	TCCR1B |= (1 << CS11);			// 8 prescaler: 0,5 microseconds at 16mhz
	TIMSK1 |= (1 << OCIE1A);		// enable timer compare interrupt
	sei();

#endif

#ifdef ATMEGA_A32U4
	cli();
	TCCR3A = 0;						// set entire TCCR1 register to 0
	TCCR3B = 0;
	TCNT3 = 0;

	OCR3A = 100;					// compare match register, change this
	TCCR3B |= (1 << WGM32);			// turn on CTC mode
	TCCR3B |= (1 << CS31);			// 8 prescaler: 0,5 microseconds at 16mhz
	TIMSK3 |= (1 << OCIE3A);		// enable timer compare interrupt
	sei();
#endif
}

#endif

