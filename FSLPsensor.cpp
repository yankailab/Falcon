#include "FSLPsensor.h"

/* */

FSLPsensor::FSLPsensor(void)
{
}

/* */
FSLPsensor::~FSLPsensor(void)
{
}

void FSLPsensor::setup(int SL, int DL1, int DL2, int BotR)
{
	m_SL = SL;
	m_DL1 = DL1;
	m_DL2 = DL2;
	m_BotR = BotR;

	m_bTouch = false;
	m_lastPosition = 0;
	m_position = 0;
}

/* */
void FSLPsensor::update()
{
	m_pressure = fslpGetPressure();

	if (m_pressure == 0)
	{
		// There is no detectable pressure, so measuring
		// the position does not make sense.
		m_position = 0;
		m_lastPosition = 0;
	}
	else
	{
		m_position = fslpGetPosition(); // Raw reading, from 0 to 1023.
	}

//	char	report[80];
//	sprintf(report, "pressure: %5d   position: %5d\n", m_pressure, m_position);
//	g_pSerial->print(report);
}

// This function follows the steps described in the FSLP
// integration guide to measure the position of a force on the
// sensor.  The return value of this function is proportional to
// the physical distance from drive line 2, and it is between
// 0 and 1023.  This function does not give meaningful results

// if fslpGetPressure is returning 0.
int FSLPsensor::fslpGetPosition()
{
	// Step 1 - Clear the charge on the sensor.
	pinMode(m_SL, OUTPUT);
	digitalWrite(m_SL, LOW);

	pinMode(m_DL1, OUTPUT);
	digitalWrite(m_DL1, LOW);

	pinMode(m_DL2, OUTPUT);
	digitalWrite(m_DL2, LOW);

	pinMode(m_BotR, OUTPUT);
	digitalWrite(m_BotR, LOW);

	// Step 2 - Set up appropriate drive line voltages.
	digitalWrite(m_DL1, HIGH);
	pinMode(m_BotR, INPUT);
	pinMode(m_SL, INPUT);

	// Step 3 - Wait for the voltage to stabilize.
	delayMicroseconds(10);

	// Step 4 - Take the measurement.
	analogReset();
	return analogRead(m_SL);
}

// This function follows the steps described in the FSLP
// integration guide to measure the pressure on the sensor.
// The value returned is usually between 0 (no pressure)
// and 500 (very high pressure), but could be as high as

// 32736.
int FSLPsensor::fslpGetPressure()
{
	// Step 1 - Set up the appropriate drive line voltages.
	pinMode(m_DL1, OUTPUT);
	digitalWrite(m_DL1, HIGH);

	pinMode(m_BotR, OUTPUT);
	digitalWrite(m_BotR, LOW);

	pinMode(m_SL, INPUT);

	pinMode(m_DL2, INPUT);

	// Step 2 - Wait for the voltage to stabilize.
	delayMicroseconds(10);

	// Step 3 - Take two measurements.
	analogReset();

	int v1 = analogRead(m_DL2);
	analogReset();

	int v2 = analogRead(m_SL);

	// Step 4 - Calculate the pressure.
	// Detailed information about this formula can be found in the
	// FSLP Integration Guide.
	if (v1 == v2)
	{
		// Avoid dividing by zero, and return maximum reading.
		return 32 * 1023;
	}

	return 32 * v2 / (v1 - v2);
}

// Performs an ADC reading on the internal GND channel in order
// to clear any voltage that might be leftover on the ADC.

// Only works on AVR boards and silently fails on others.
void FSLPsensor::analogReset()
{
	#if defined(ADMUX)
		#if defined(ADCSRB) && defined(MUX5)
			// Code for the ATmega2560 and ATmega32U4
			ADCSRB |= (1 << MUX5);	
		#endif

		ADMUX = 0x1F; //connect to 0V

		// Start the conversion and wait for it to finish.
		ADCSRA |= (1 << ADSC);
		loop_until_bit_is_clear(ADCSRA, ADSC);
	#endif
}

