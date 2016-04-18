#include "VehicleLink.h"

/* */
VehicleLink::VehicleLink(void)
{
}

/* */
VehicleLink::~VehicleLink(void)
{
}

#ifdef RC_TRANSMITTER

void VehicleLink::init(void)
{
	int i;
	for (i = 0; i < RC_CHANNEL_NUM; i++)
	{
		m_channelValues[i] = m_pConfig->PWMCenter;
	}

	m_numChannel = RC_CHANNEL_NUM;
	m_hostCMD.m_cmd = 0;
	m_rfCMD.m_cmd = 0;
}


/* */
void VehicleLink::sendRFupdateAttitude(void)
{
	uint16_t data;
	uint8_t i;
	uint8_t	data1;
	uint8_t	data2;

	m_pRFSerial->write(VL_CMD_BEGIN);
	m_pRFSerial->write(m_numChannel * 2 + 4 - MAVLINK_HEADDER_LEN);
	m_pRFSerial->write(CMD_RC_UPDATE);
	m_pRFSerial->write(m_numChannel);

	for (i = 0; i < m_numChannel; i++)
	{
		data = m_channelValues[i];
		data1 = highByte(data);
		data2 = lowByte(data);

		m_pRFSerial->write(data1);
		m_pRFSerial->write(data2);
	}
}

/* */
void VehicleLink::receiveFromRF(void)
{
	byte	inByte;

	while (m_pRFSerial->available() > 0)
	{
		inByte = m_pRFSerial->read();

		if (m_rfCMD.m_cmd != 0)
		{
			m_rfCMD.m_pBuf[m_rfCMD.m_iByte] = inByte;
			m_rfCMD.m_iByte++;

			if (m_rfCMD.m_iByte == 2)
			{
				m_rfCMD.m_payloadLen = inByte;
			}
			else if (m_rfCMD.m_iByte == m_rfCMD.m_payloadLen + MAVLINK_HEADDER_LEN + 1)
			{
				if (m_rfCMD.m_cmd == VL_CMD_BEGIN)
				{
					//decode the command
				}
				else
				{
					m_pHostSerial->write(m_rfCMD.m_pBuf, m_rfCMD.m_iByte);
				}

				m_rfCMD.m_cmd = 0;
				return;
			}
		}
		else
		if (inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_rfCMD.m_cmd = inByte;
			m_rfCMD.m_pBuf[0] = inByte;
			m_rfCMD.m_iByte = 1;
			m_rfCMD.m_payloadLen = 0;
		}
	}
}

/* */
bool VehicleLink::receiveFromHost(void)
{
	byte	inByte;

	while (m_pHostSerial->available() > 0)
	{
		inByte = m_pHostSerial->read();

		if (m_hostCMD.m_cmd != 0)
		{
			m_hostCMD.m_pBuf[m_hostCMD.m_iByte] = inByte;
			m_hostCMD.m_iByte++;

			if (m_hostCMD.m_iByte == 2)	//Payload length
			{
				m_hostCMD.m_payloadLen = inByte;
			}
			else if (m_hostCMD.m_iByte == m_hostCMD.m_payloadLen + MAVLINK_HEADDER_LEN + 1)
			{
				//decode the command
				hostCommand();
				m_hostCMD.m_cmd = 0;

#ifdef USB_DEBUG
				//					m_pRFSerial->println();
				//					m_pRFSerial->println("MAVLINK cmd from Host:");
				//					m_pRFSerial->println();
#endif

				return true; //Execute one command at a time
			}
		}
		else if (inByte == MAVLINK_BEGIN)//(inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_hostCMD.m_cmd = inByte;
			m_hostCMD.m_pBuf[0] = inByte;
			m_hostCMD.m_iByte = 1;
			m_hostCMD.m_payloadLen = 0;	
		}
	}

	return false;
}

void VehicleLink::hostCommand(void)
{
	int i;
	byte numChannel;
	uint16_t val;

	switch (m_hostCMD.m_pBuf[2]) //Command
	{
	case CMD_RC_UPDATE:
		if (*m_pOprMode != OPE_RC_BRIDGE)break;

		numChannel = m_hostCMD.m_pBuf[3];
		if (m_numChannel > RC_CHANNEL_NUM)
		{
			numChannel = RC_CHANNEL_NUM;
		}

		for (i = 0; i<numChannel; i++)
		{
			val = (int)makeWord(m_hostCMD.m_pBuf[4 + i * 2 + 1], m_hostCMD.m_pBuf[4 + i * 2]);
			m_channelValues[i] = val;
		}

		break;

	case CMD_OPERATE_MODE:
		val = m_hostCMD.m_pBuf[3];
		if (val < 0)return;
		else if (val > OPE_BOOT)return;

		//			tone(BUZZER_PIN, 900, SHORT_BEEP);
		*m_pOprMode = val;
		if (*m_pOprMode == OPE_ALT_HOLD)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
		}
		else if (*m_pOprMode == OPE_MANUAL)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
		}

		break;

	default:

#ifndef RC_TRANSMITTER
		m_pRFSerial->write(m_hostCMD.m_pBuf, m_hostCMD.m_iByte);
#endif
		break;
	}

}

#endif

#ifdef UART_RC_BRIDGE

void VehicleLink::init(void)
{
	int i;
	for (i = 0; i < RC_CHANNEL_NUM; i++)
	{
		m_channelValues[i] = 1500;// m_pConfig->PWMCenter;
	}

	m_numChannel = RC_CHANNEL_NUM;
	m_uartCMD.m_cmd = 0;
	m_hostCMD.m_cmd = 0;
}

#ifndef ATMEGA_A328
bool VehicleLink::receiveFromUART(void)
{
	byte	inByte;

	while (m_pUartSerial->available() > 0)
	{
		inByte = m_pUartSerial->read();

		if (m_uartCMD.m_cmd != 0)
		{
			m_uartCMD.m_pBuf[m_uartCMD.m_iByte] = inByte;
			m_uartCMD.m_iByte++;

			if (m_uartCMD.m_iByte == 2)	//Payload length
			{
				m_uartCMD.m_payloadLen = inByte;
			}
			else if (m_uartCMD.m_iByte == m_uartCMD.m_payloadLen + MAVLINK_HEADDER_LEN)
			{
				//decode the command
				uartCommand();
				m_uartCMD.m_cmd = 0;

				return true; //Execute one command at a time
			}
		}
		else if (inByte == MAVLINK_BEGIN)//(inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_uartCMD.m_cmd = inByte;
			m_uartCMD.m_pBuf[0] = inByte;
			m_uartCMD.m_iByte = 1;
			m_uartCMD.m_payloadLen = 0;
		}
	}

	return false;
}

void VehicleLink::uartCommand(void)
{
	int i;
	byte numChannel;
	uint16_t val;

	switch (m_uartCMD.m_pBuf[2]) //Command
	{
		case CMD_RC_UPDATE:
			if (*m_pOprMode != OPE_RC_BRIDGE)break;

			numChannel = m_uartCMD.m_pBuf[3];
			if(m_numChannel > RC_CHANNEL_NUM)
			{
				numChannel=RC_CHANNEL_NUM;
			}
		
			for(i=0; i<numChannel; i++)
			{
				val = (int)makeWord(m_uartCMD.m_pBuf[4 + i * 2+1], m_uartCMD.m_pBuf[4 + i * 2]);
				m_channelValues[i] = val;
			}
				
			break;

		case CMD_OPERATE_MODE:
			val = m_uartCMD.m_pBuf[3];
			if (val < 0)return;
			else if (val > OPE_BOOT)return;

//			tone(BUZZER_PIN, 900, SHORT_BEEP);
			*m_pOprMode = val;
			if (*m_pOprMode == OPE_ALT_HOLD)
			{
				m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
			}
			else if (*m_pOprMode == OPE_MANUAL)
			{
				m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
			}
			
			sendUartHeartBeat();
			break;

		default:
			break;
	}

}

/* */
void VehicleLink::sendUartHeartBeat(void)
{
	uint16_t data;
	uint8_t i;
	uint8_t	data1;
	uint8_t	data2;

	m_pUartSerial->write(MAVLINK_BEGIN);
	m_pUartSerial->write(1);
	m_pUartSerial->write(CMD_OPERATE_MODE);
	m_pUartSerial->write(*m_pOprMode);

}
#endif

/* */
bool VehicleLink::receiveFromHost(void)
{
	byte	inByte;

	while (m_pHostSerial->available() > 0)
	{
		inByte = m_pHostSerial->read();

		if (m_hostCMD.m_cmd != 0)
		{
			m_hostCMD.m_pBuf[m_hostCMD.m_iByte] = inByte;
			m_hostCMD.m_iByte++;

			if (m_hostCMD.m_iByte == 2)	//Payload length
			{
				m_hostCMD.m_payloadLen = inByte;
			}
			else if (m_hostCMD.m_iByte == m_hostCMD.m_payloadLen + MAVLINK_HEADDER_LEN)
			{
				//decode the command
				hostCommand();
				m_hostCMD.m_cmd = 0;

				return true; //Execute one command at a time
			}
		}
		else if (inByte == MAVLINK_BEGIN)//(inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_hostCMD.m_cmd = inByte;
			m_hostCMD.m_pBuf[0] = inByte;
			m_hostCMD.m_iByte = 1;
			m_hostCMD.m_payloadLen = 0;	
		}
	}

	return false;
}

void VehicleLink::hostCommand(void)
{
	int i;
	byte numChannel;
	uint16_t val;

	switch (m_hostCMD.m_pBuf[2]) //Command
	{
	case CMD_RC_UPDATE:
//		if (*m_pOprMode != OPE_RC_BRIDGE)break;

		numChannel = m_hostCMD.m_pBuf[3];
		if (m_numChannel > RC_CHANNEL_NUM)
		{
			numChannel = RC_CHANNEL_NUM;
		}

		for (i = 0; i<numChannel; i++)
		{
			val = (int)makeWord(m_hostCMD.m_pBuf[4 + i * 2 + 1], m_hostCMD.m_pBuf[4 + i * 2]);
			m_channelValues[i] = val;
		}

		break;

	case CMD_OPERATE_MODE:
		val = m_hostCMD.m_pBuf[3];
		if (val < 0)return;
		else if (val > OPE_BOOT)return;

		*m_pOprMode = val;
		if (*m_pOprMode == OPE_ALT_HOLD)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
		}
		else if (*m_pOprMode == OPE_MANUAL)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
		}

		sendHostHeartBeat();
		break;

	default:

		break;
	}

}

/* */
void VehicleLink::sendHostHeartBeat(void)
{
	uint16_t data;
	uint8_t i;
	uint8_t	data1;
	uint8_t	data2;

	m_pHostSerial->write(MAVLINK_BEGIN);
	m_pHostSerial->write(1);
	m_pHostSerial->write(CMD_OPERATE_MODE);
	m_pHostSerial->write(*m_pOprMode);

}
#endif

#ifdef SERIAL_RECEIVER

/* */
void VehicleLink::sendRFupdateAttitude(void)
{
/*	int16_t data;
	uint8_t i;
	byte	data1;
	byte	data2;

	m_pRFSerial->write(VL_CMD_BEGIN);
	m_pRFSerial->write(m_numChannel * 2 + 2);
	m_pRFSerial->write(CMD_RC_UPDATE);
	m_pRFSerial->write(m_numChannel);

	for (i = 0; i < m_numChannel; i++)
	{
		data = m_channelValues[i];
		data1 = data >> 8;
		data2 = data;

		m_pRFSerial->write(data1);
		m_pRFSerial->write(data2);
	}
	*/
}

/* */
void VehicleLink::receiveFromRF(void)
{
	byte	inByte;

	while (m_pRFSerial->available() > 0)
	{
		inByte = m_pRFSerial->read();

		if (m_rfCMD.m_cmd != 0)
		{
			m_rfCMD.m_pBuf[m_rfCMD.m_iByte] = inByte;
			m_rfCMD.m_iByte++;

			if (m_rfCMD.m_iByte == 2)
			{
				m_rfCMD.m_payloadLen = inByte;
			}
			else if (m_rfCMD.m_iByte == m_rfCMD.m_payloadLen + MAVLINK_HEADDER_LEN)
			{
				if (m_rfCMD.m_cmd == VL_CMD_BEGIN)
				{
					vehicleCommand();
				}
				else
				{
					m_pHostSerial->write(m_rfCMD.m_pBuf, m_rfCMD.m_iByte);
				}

				m_rfCMD.m_cmd = 0;
				return;
			}
		}
		else
		if (inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_rfCMD.m_cmd = inByte;
			m_rfCMD.m_pBuf[0] = inByte;
			m_rfCMD.m_iByte = 1;
			m_rfCMD.m_payloadLen = 0;
		}
	}

}

/* */
void VehicleLink::receiveFromHost(void)
{
	byte	inByte;
	
	while (m_pHostSerial->available() > 0)
	{
		inByte = m_pHostSerial->read();

		if (m_hostCMD.m_cmd != 0)
		{
			m_hostCMD.m_pBuf[m_hostCMD.m_iByte] = inByte;
			m_hostCMD.m_iByte++;

			if (m_hostCMD.m_iByte == 2)
			{
				m_hostCMD.m_payloadLen = inByte;
			}
			else if (m_hostCMD.m_iByte == m_hostCMD.m_payloadLen + MAVLINK_HEADDER_LEN)
			{
				if (m_hostCMD.m_cmd == VL_CMD_BEGIN)
				{
					vehicleCommand();
				}
				else
				{
					m_pRFSerial->write(m_hostCMD.m_pBuf, m_hostCMD.m_iByte);
				}

				m_hostCMD.m_cmd = 0;
				return;
			}
		}
		else
		if (inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_hostCMD.m_cmd = inByte;
			m_hostCMD.m_pBuf[0] = inByte;
			m_hostCMD.m_iByte = 1;
			m_hostCMD.m_payloadLen = 0;
		}
	}
}

/* */
void VehicleLink::vehicleCommand(void)
{
	int i;
	byte numChannel;
	uint16_t val;

	switch (m_rfCMD.m_pBuf[2])
	{
		case CMD_RC_UPDATE:
			numChannel = m_rfCMD.m_pBuf[3];
			if(m_numChannel > RC_CHANNEL_NUM)
			{
				numChannel=RC_CHANNEL_NUM;
			}

			for(i=0; i<numChannel; i++)
			{
				val = (int)word(m_rfCMD.m_pBuf[4+i*2],m_rfCMD.m_pBuf[4+i*2+1]);
				m_channelValues[i] = val;
			}

			break;
		default:

			break;
	}

}

#endif




#ifdef LIDAR_CONTROLLER

void VehicleLink::init(void)
{
	int i;
	for (i = 0; i < RC_CHANNEL_NUM; i++)
	{
		m_channelValues[i] = m_pConfig->PWMCenter;
	}

	m_numChannel = RC_CHANNEL_NUM;
	m_uartCMD.m_cmd = 0;
	m_hostCMD.m_cmd = 0;
}

bool VehicleLink::receiveFromUART(void)
{
	byte	inByte;

	while (m_pUartSerial->available() > 0)
	{
		inByte = m_pUartSerial->read();

		if (m_uartCMD.m_cmd != 0)
		{
			m_uartCMD.m_pBuf[m_uartCMD.m_iByte] = inByte;
			m_uartCMD.m_iByte++;

			if (m_uartCMD.m_iByte == 2)	//Payload length
			{
				m_uartCMD.m_payloadLen = inByte;
			}
			else if (m_uartCMD.m_iByte == m_uartCMD.m_payloadLen + MAVLINK_HEADDER_LEN)
			{
				//decode the command
				uartCommand();
				m_uartCMD.m_cmd = 0;

				return true; //Execute one command at a time
			}
		}
		else if (inByte == MAVLINK_BEGIN)//(inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_uartCMD.m_cmd = inByte;
			m_uartCMD.m_pBuf[0] = inByte;
			m_uartCMD.m_iByte = 1;
			m_uartCMD.m_payloadLen = 0;
		}
	}

	return false;
}

void VehicleLink::uartCommand(void)
{
	int i;
	byte numChannel;
	uint16_t val;

	switch (m_uartCMD.m_pBuf[2]) //Command
	{
	case CMD_RC_UPDATE:
		if (*m_pOprMode != OPE_RC_BRIDGE)break;

		numChannel = m_uartCMD.m_pBuf[3];
		if (m_numChannel > RC_CHANNEL_NUM)
		{
			numChannel = RC_CHANNEL_NUM;
		}

		for (i = 0; i<numChannel; i++)
		{
			val = (int)makeWord(m_uartCMD.m_pBuf[4 + i * 2 + 1], m_uartCMD.m_pBuf[4 + i * 2]);
			m_channelValues[i] = val;
		}

		break;

	case CMD_OPERATE_MODE:
		val = m_uartCMD.m_pBuf[3];
		if (val < 0)return;
		else if (val > OPE_BOOT)return;

		//			tone(BUZZER_PIN, 900, SHORT_BEEP);
		*m_pOprMode = val;
		if (*m_pOprMode == OPE_ALT_HOLD)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
		}
		else if (*m_pOprMode == OPE_MANUAL)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
		}

		sendUartHeartBeat();
		break;

	default:
		break;
	}

}

/* */
void VehicleLink::sendUartHeartBeat(void)
{
	uint16_t data;
	uint8_t i;
	uint8_t	data1;
	uint8_t	data2;

	m_pUartSerial->write(MAVLINK_BEGIN);
	m_pUartSerial->write(1);
	m_pUartSerial->write(CMD_OPERATE_MODE);
	m_pUartSerial->write(*m_pOprMode);

}


/* */
bool VehicleLink::receiveFromHost(void)
{
	byte	inByte;

	while (m_pHostSerial->available() > 0)
	{
		inByte = m_pHostSerial->read();

		if (m_hostCMD.m_cmd != 0)
		{
			m_hostCMD.m_pBuf[m_hostCMD.m_iByte] = inByte;
			m_hostCMD.m_iByte++;

			if (m_hostCMD.m_iByte == 2)	//Payload length
			{
				m_hostCMD.m_payloadLen = inByte;
			}
			else if (m_hostCMD.m_iByte == m_hostCMD.m_payloadLen + MAVLINK_HEADDER_LEN)
			{
				//decode the command
				hostCommand();
				m_hostCMD.m_cmd = 0;

				return true; //Execute one command at a time
			}
		}
		else if (inByte == MAVLINK_BEGIN)//(inByte == VL_CMD_BEGIN || inByte == MAVLINK_BEGIN)
		{
			m_hostCMD.m_cmd = inByte;
			m_hostCMD.m_pBuf[0] = inByte;
			m_hostCMD.m_iByte = 1;
			m_hostCMD.m_payloadLen = 0;
		}
	}

	return false;
}

void VehicleLink::hostCommand(void)
{
	int i;
	byte numChannel;
	uint16_t val;

	switch (m_hostCMD.m_pBuf[2]) //Command
	{
	case CMD_RC_UPDATE:
		if (*m_pOprMode != OPE_RC_BRIDGE)break;

		numChannel = m_hostCMD.m_pBuf[3];
		if (m_numChannel > RC_CHANNEL_NUM)
		{
			numChannel = RC_CHANNEL_NUM;
		}

		for (i = 0; i<numChannel; i++)
		{
			val = (int)makeWord(m_hostCMD.m_pBuf[4 + i * 2 + 1], m_hostCMD.m_pBuf[4 + i * 2]);
			m_channelValues[i] = val;
		}

		break;

	case CMD_OPERATE_MODE:
		val = m_hostCMD.m_pBuf[3];
		if (val < 0)return;
		else if (val > OPE_BOOT)return;

		//			tone(BUZZER_PIN, 900, SHORT_BEEP);
		*m_pOprMode = val;
		if (*m_pOprMode == OPE_ALT_HOLD)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[5];
		}
		else if (*m_pOprMode == OPE_MANUAL)
		{
			m_channelValues[m_pConfig->buttonChannel[0].ppmIdx] = m_pConfig->buttonChannel[0].modePPM[0];
		}

		sendHostHeartBeat();
		break;

	default:

		break;
	}

}

/* */
void VehicleLink::sendHostHeartBeat(void)
{
	uint16_t data;
	uint8_t i;
	uint8_t	data1;
	uint8_t	data2;

	m_pHostSerial->write(MAVLINK_BEGIN);
	m_pHostSerial->write(1);
	m_pHostSerial->write(CMD_OPERATE_MODE);
	m_pHostSerial->write(*m_pOprMode);

}
#endif


/*
bool VehicleLink::receiveFromHost(void)
{
	byte inByte;

	if(m_cmd == VL_CMD_BEGIN)	//receiving the cmd packet
	{
		while (m_pHostSerial->available() > 0)
		{
			inByte = m_pHostSerial->read();
			m_pCmdBuf[m_iCmdByte] = inByte;
			m_iCmdByte++;

			if(m_iCmdByte == 2)
			{
				m_payloadLen = inByte;
			}
			else if(m_iCmdByte == m_payloadLen+2)
			{
				//decode the command

				m_cmd = 0;
				break;
			}
		}
	}
	else if(m_cmd == MAVLINK_BEGIN)
	{
		while (m_pHostSerial->available() > 0)
		{
			inByte = m_pHostSerial->read();
			m_pCmdBuf[m_iCmdByte] = inByte;
			m_iCmdByte++;
			m_cmdRecvByte++;

			if(m_iCmdByte == 2)
			{
				m_payloadLen = inByte;
			}
			else if(m_iCmdByte == MAVLINK_PACKET_LEN)
			{
				m_pRFSerial->write(0xFE);
				m_pRFSerial->write(m_iCmdByte);
				m_pRFSerial->write(m_pCmdBuf,m_iCmdByte);
				m_iCmdByte = 0;
				break;
			}
			else if(m_cmdRecvByte == m_payloadLen+7)
			{
				m_pRFSerial->write(0xFE);
				m_pRFSerial->write(m_iCmdByte);
				m_pRFSerial->write(m_pCmdBuf,m_iCmdByte);
				m_cmd = 0;
				break;
			}
		}
	}
	else
	{
		if(m_pHostSerial->available() <= 0)return;

		inByte = m_pHostSerial->read();
		if(inByte==VL_CMD_BEGIN || inByte==MAVLINK_BEGIN)
		{
			m_cmd = inByte;
			m_pCmdBuf[0] = inByte;
			m_iCmdByte = 1;
			m_cmdRecvByte = 1;
			m_payloadLen = 0;
		}
	}
}

*/