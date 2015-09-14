#pragma once

#include "U8glib.h"

#define BUF_LEN 16

class UI
{
public:
	UI();
	~UI();

	void init(U8GLIB_SSD1306_128X64* pDisplay);
	void draw(int8_t OprMode, int16_t* pChannels);

private:

	U8GLIB_SSD1306_128X64* m_pDisplay;

	char buf_c0[BUF_LEN];
	char buf_c1[BUF_LEN];
	char buf_c2[BUF_LEN];
	char buf_c3[BUF_LEN];
	char buf_c4[BUF_LEN];
	char buf_c5[BUF_LEN];
	char buf_c6[BUF_LEN];
	char buf_c7[BUF_LEN];
};

