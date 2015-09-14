#pragma once

struct Control_Channel
{
	uint8_t ppmIdx;
	int16_t center;	//PWM Offset
	int16_t factor;
	int16_t deadzone;
	int16_t lowerLimit;
	int16_t upperLimit;
};

struct FSLPChannel
{
	uint8_t ppmIdx;
	int16_t touchThreshold;
	int16_t modeChangeThreshold;
	int16_t factor;
	int16_t deadzone;
//	int16_t lowerLimit;
//	int16_t upperLimit;
	int16_t lowerLength;
	int16_t upperLength;
	int16_t pwmCenter;
};

struct Switch_Channel
{
	uint8_t ppmIdx;
	uint16_t modePPM[6];
};

struct config_t
{
	uint8_t vers;

	int16_t PPMframeLength;
	int16_t PPMPulseLength;
	int16_t PWMLenFrom;
	int16_t PWMLenTo;
	int16_t PWMCenter;

	Control_Channel controlChannel[2];
	FSLPChannel throttleChannel;
	FSLPChannel yawChannel;

	Switch_Channel buttonChannel[2];

	int16_t thresholdBTN;
};// config;

