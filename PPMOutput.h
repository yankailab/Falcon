#pragma once
#include "falconcommon.h"
#include "variables.h"

#ifdef ATMEGA_A328
/*
#define default_servo_value 250
//1500  //set the default servo value
	#define PPM_FrLen	3600
//7500
//22500  //set the PPM frame length in microseconds (1ms = 1000µs)
	#define PPM_PulseLen	80
//500  //set the pulse length
int waitTime;
*/

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

/*
ISR(TIMER2_COMPA_vect)
{						//leave this alone
	static boolean	state = true;

	TCNT2 = 0;

	if (waitTime > 0)
	{
		if (waitTime > 255)
		{
			waitTime -= 255;
			OCR2A = 255;
		}
		else
		{
			OCR2A = waitTime;
			waitTime = 0;
		}

		return;
	}

	if (state)
	{					//start pulse
		digitalWrite(sigPin, onState);
		waitTime = 0;	//PPM_PulseLen * 2 - 255;
		OCR2A = PPM_PulseLen;
		state = false;
	}
	else
	{					//end pulse and calculate when to start the next pulse
		static byte			cur_chan_numb;
		static unsigned int calc_rest;

		digitalWrite(sigPin, !onState);
		state = true;

		if (cur_chan_numb >= chanel_number)
		{
			cur_chan_numb = 0;
			calc_rest = calc_rest + PPM_PulseLen;	//
			waitTime = (PPM_FrLen - calc_rest) - 255;
			OCR2A = 255;	//(PPM_FrLen - calc_rest) * 2;
			calc_rest = 0;
		}
		else
		{
			waitTime = 0;	//(ppm[cur_chan_numb] - PPM_PulseLen) * 2 - 255;
			OCR2A = (ppm[cur_chan_numb] - PPM_PulseLen);
			calc_rest = calc_rest + ppm[cur_chan_numb];
			cur_chan_numb++;
		}
	}
}
*/

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

void PPM_init(config_t config)
{
	int i;

	//initiallize default ppm values
	for(i = 0; i < RC_CHANNEL_NUM; i++)
	{
		g_ppm[i] = DEFAULT_PWM;
	}

	pinMode(PPM_OUTPUT_PIN, OUTPUT);
	digitalWrite(PPM_OUTPUT_PIN, !PPM_ON_STATE); //set the PPM signal pin to the default state (off)
	
	PPM_FrLen = config.PPMframeLength;
	PPM_PulseLen = config.PPMPulseLength;


#ifdef ATMEGA_A328
	/*
	waitTime = 0;

	cli();

	//set timer2 interrupt at 8kHz
	TCCR2A = 0;						// set entire TCCR2A register to 0
	TCCR2B = 0;						// same for TCCR2B
	TCNT2 = 0;						//initialize counter value to 0

	// set compare match register for 8khz increments
	OCR2A = 100;					//249;// = (16*10^6) / (8000*8) - 1 (must be <256)

	// turn on CTC mode
	TCCR2A |= (1 << WGM21);

	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS20);

	//  TCCR2B |= (1 << CS21);
	TCCR2B |= (1 << CS22);

	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);
	sei();
	*/

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

