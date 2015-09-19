#pragma once

#include <Arduino.h>

#define VERSION 19

//#define ATMEGA_A32U4
#define ATMEGA_A328

//#define RC_TRANSMITTER
//#define SERIAL_TRANSMITTER
//#define SERIAL_RECEIVER
#define UART_RC_BRIDGE
//#define LIDAR_CONTROLLER

#define USB_DEBUG

//
// Vehicle Link
//
#define CMD_BUF_LEN 256
#define LINK_CHANNELS 8
#define VL_CMD_BEGIN 0xFF
#define MAVLINK_BEGIN 0xFE
//#define MAVLINK_HEADDER_LEN 8
#define MAVLINK_HEADDER_LEN 3
//temporal
//0 START MARK
//1 PAYLOAD LENGTH
//2 COMMAND
//3 Payload...

//FALCON COMMANDS
#define CMD_RC_UPDATE 0
#define CMD_OPERATE_MODE 1


//
// Operation mode
//
#define OPE_SERIAL_BRIDGE 0
#define OPE_MANUAL 1
#define OPE_ALT_HOLD 2
#define OPE_LOITER 3
#define OPE_AUTO 4
#define OPE_RC_BRIDGE 5
#define OPE_BOOT 6

#define OPE_PPM_THROUGH 7
#define OPE_UP_COLLISION_AVOID 8
#define OPE_ALL_COLLISION_AVOID 9
#define OPE_REFERENCE_FIX 10

//
// RC Channel
//
//#define YAW 0
#define PITCH 0
#define ROLL 1
//#define THROTTLE 3
//#define BTN 4

//
//Pin definitions
//
#define INTERNAL_LED_PIN 15
#define BUZZER_PIN 13
#define RSSI_PIN 9
#define BATT_SENSE A8

//
//UI
//
#define SHORT_BEEP 150
#define LONG_BEEP 500

#define RC_CHANNEL_NUM	8				//set the number of chanels
#define PPM_ON_STATE	0				//set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN	5				//set PPM signal output pin on the arduino
#define PPM_INPUT_PIN 7

#ifdef RC_TRANSMITTER
#define RC_CHANNEL_NUM	8				//set the number of chanels
#define PPM_ON_STATE	0				//set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN	44				//set PPM signal output pin on the arduino
#endif

#ifdef SERIAL_RECEIVER
#define TEL_RX_PIN 9
#define TEL_TX_PIN 8

#define RC_CHANNEL_NUM	8					//set the number of chanels
#define PPM_ON_STATE	0				//set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUTPUT_PIN	5				//set PPM signal output pin on the arduino

#endif


#define SETBIT_ON(x,y) (x|=(1<<y))
#define SETBIT_OFF(x,y) (x&=(~(1<<y)))
#define BIT_ON(x,y) (x&(1<<y))





