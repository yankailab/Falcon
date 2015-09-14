//-------------------------------------
//PPM generator
//-------------------------------------
/*
// PPMOut requires two buffers:
// Input buffer containing input samples in microseconds
// Work buffer of ((channels + 1) * 2) elements for internal calculations and frame buffering
// This setup removes any limit on the number of channels you want, and makes sure the library doesn't use more
// memory than it really needs, since the client code supplies the buffers.
#define PPM_OUT_PIN 9
#define CHANNELS 8

#define PPM_PULSE_LENGTH 500
#define PPM_PAUSE_LENGTH 20000

uint16_t g_input[CHANNELS];                   // Input buffer in microseconds
uint8_t  g_work[PPMOUT_WORK_SIZE(CHANNELS)];  // we need to have a work buffer for the PPMOut class
rc::PPMOut g_PPMOut(CHANNELS, g_input, g_work, CHANNELS);

void PPM_inita(void)
{
	// Initialize timer1, this is required for all features that use Timer1
	// (PPMIn/PPMOut/ServoIn/ServoOut)
	rc::Timer1::init();
	
	//Initial position
	for (uint8_t i = 0;  i < CHANNELS; ++i)
	{		
		// fill input buffer, convert raw values to microseconds
		// g_input[i] = map(analogRead(g_pins[i]), 0, 1024, 1000, 2000);
		g_input[i] = map(0, 0, 1024, 1000, 2000);
	}
	
	// initialize PPMOut with some settings
	g_PPMOut.setPulseLength(PPM_PULSE_LENGTH);   // pulse length in microseconds 448
	g_PPMOut.setPauseLength(PPM_PAUSE_LENGTH);   // length of pause after last channel in microseconds 10448
	// note: this is also called the end of frame, or start of frame, and is usually around 10ms
	
	g_PPMOut.setChannelCount(CHANNELS);
	g_PPMOut.start(PPM_OUT_PIN);
	// start PPMOut, use pin 9 (pins 9 and 10 are preferred)
}

void PPM_updatea(void)
{
	// update the input buffer
	for (uint8_t i = 0;  i < CHANNELS; ++i)
	{
		// fill input buffer, convert raw values to microseconds
//		g_input[i] = map(analogRead(g_pins[i]), 0, 1024, 1000, 2000);
		g_input[i] = map(100, 0, 1024, 1000, 2000);
	}
	
	// tell PPMOut there are new values available in the input buffer
	g_PPMOut.update();
}

*/