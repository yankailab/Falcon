HardwareSerial* g_pUSBSerial;
#include <SoftwareSerial.h>

#define TEL_RX_PIN 9
#define TEL_TX_PIN 8

SoftwareSerial g_pTELSerial(TEL_RX_PIN, TEL_TX_PIN);


void deviceSetup()
{

#ifdef USB_DEBUG
	// wait for Leonardo enumeration, others continue immediately
	while (!(*g_pUSBSerial));
#endif

	if (*g_pUSBSerial)
	{
		//To host side (Android device etc.)
		g_pUSBSerial->begin(115200);
		g_bHostConnected = true;
		g_pUSBSerial->println(F("FALCON_ON"));
	}

	//Init PPM generator
	PPM_init();

	g_pTELSerial.begin(57600);

	//Vehicle Link
	g_VLink.m_pRFSerial = g_pRFSerial;
	g_VLink.m_pHostSerial = &g_pTELSerial;
	g_VLink.m_numChannel = 8;

}

void deviceLoop()
{
	int i;

	g_VLink.receiveFromRF();
	g_VLink.receiveFromHost();

	for (i = 0; i<RC_CHANNEL_NUM; i++)
	{
		g_ppm[i] = constrain((uint16_t)g_VLink.m_channelValues[i], 980, 2025);
	}

	// slow rate actions
	switch (g_counter)
	{
	case 1:
		break;
	case 2:
		break;

	case 3:
		break;

	case 4:
		break;

	case 5:
		break;

	case 6:
		break;

	case 1000:
		/*			if(g_bPrintIMU)
		{
		g_pUSBSerial->print("ypr\t");
		g_pUSBSerial->print(g_ppm[config.controlChannel[YAW].ppmIdx]);
		g_pUSBSerial->print("\t");
		g_pUSBSerial->print(g_ppm[config.controlChannel[PITCH].ppmIdx]);
		g_pUSBSerial->print("\t");
		g_pUSBSerial->print(g_ppm[config.controlChannel[ROLL].ppmIdx]);
		g_pUSBSerial->print("\tThr:\t");
		g_pUSBSerial->println(g_ppm[config.throttleChannel.ppmIdx]);
		}
		*/
		g_counter = 0;
		break;

	default:
		break;
	}
	g_counter++;
}