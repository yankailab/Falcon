#include "UI.h"
#include <string.h>



UI::UI()
{
}


UI::~UI()
{
}

void UI::init(U8GLIB_SSD1306_128X64* pDisplay)
{
	m_pDisplay = pDisplay;

	//OLED
	m_pDisplay->setFont(u8g_font_6x10);// (u8g_font_unifont);

	const char channelName[] = "CH  ";

	strcpy(buf_c0, channelName);
	strcpy(buf_c1, channelName);
	strcpy(buf_c2, channelName);
	strcpy(buf_c3, channelName);
	strcpy(buf_c4, channelName);
	strcpy(buf_c5, channelName);
	strcpy(buf_c6, channelName);
	strcpy(buf_c7, channelName);

	buf_c0[2] = '1';
	buf_c1[2] = '2';
	buf_c2[2] = '3';
	buf_c3[2] = '4';
	buf_c4[2] = '5';
	buf_c5[2] = '6';
	buf_c6[2] = '7';
	buf_c7[2] = '8';

}

void UI::draw(int8_t OprMode, int16_t* pChannels)
{
	const char* strMode[] = {"MODE: SERIAL BRIDGE","MODE: MANUAL","MODE: ALT HOLD","MODE: LOTER","MODE: AUTO","MODE: RC BRIDGE","Booting..."};

	if (pChannels)
	{
		itoa(pChannels[0], &buf_c0[4], 10);
		itoa(pChannels[1], &buf_c1[4], 10);
		itoa(pChannels[2], &buf_c2[4], 10);
		itoa(pChannels[3], &buf_c3[4], 10);
		itoa(pChannels[4], &buf_c4[4], 10);
		itoa(pChannels[5], &buf_c5[4], 10);
		itoa(pChannels[6], &buf_c6[4], 10);
		itoa(pChannels[7], &buf_c7[4], 10);
	}
	else
	{
		return;
	}

	//OLED reset
	m_pDisplay->firstPage();

	do
	{
		//Operation mode
		m_pDisplay->drawStr(0, 10, strMode[OprMode]);

		m_pDisplay->drawStr(0, 20, buf_c0);
		m_pDisplay->drawStr(0, 30, buf_c1);
		m_pDisplay->drawStr(0, 40, buf_c2);
		m_pDisplay->drawStr(0, 50, buf_c3);

		m_pDisplay->drawStr(64, 20, buf_c4);
		m_pDisplay->drawStr(64, 30, buf_c5);
		m_pDisplay->drawStr(64, 40, buf_c6);
		m_pDisplay->drawStr(64, 50, buf_c7);

	} while (m_pDisplay->nextPage());


}
