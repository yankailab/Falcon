/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Nano w/ ATmega328, Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define F_CPU 16000000L
#define ARDUINO 10601
#define ARDUINO_AVR_NANO
#define ARDUINO_ARCH_AVR
#define __cplusplus 201103L
#define __AVR__
#define __inline__
#define __asm__(x)
#define __extension__
#define __inline__
#define __volatile__
#define GCC_VERSION 40801
#define volatile(va_arg) 
#define _CONST
#define __builtin_va_start
#define __builtin_va_end
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
#ifndef __builtin_constant_p
	#define __builtin_constant_p __attribute__((__const__))
#endif
#ifndef __builtin_strlen
	#define __builtin_strlen  __attribute__((__const__))
#endif
#define NEW_H
typedef void *__builtin_va_list;
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}



#include <arduino.h>
#include <pins_arduino.h> 
#undef F
#define F(string_literal) ((const PROGMEM char *)(string_literal))
#undef PSTR
#define PSTR(string_literal) ((const PROGMEM char *)(string_literal))
#undef cli
#define cli()
#define pgm_read_byte(address_short)
#define pgm_read_word(address_short)
#define pgm_read_word2(address_short)
#define digitalPinToPort(P)
#define digitalPinToBitMask(P) 
#define digitalPinToTimer(P)
#define analogInPinToBit(P)
#define portOutputRegister(P)
#define portInputRegister(P)
#define portModeRegister(P)
#include <..\falcon\falcon.ino>
#include <..\falcon\Controller.cpp>
#include <..\falcon\Controller.h>
#include <..\falcon\DEVICE_COMMON.h>
#include <..\falcon\DEVICE_LIDAR_CONTROLLER.h>
#include <..\falcon\DEVICE_LIDAR_LOCKER_2560.cpp>
#include <..\falcon\DEVICE_LIDAR_LOCKER_2560.h>
#include <..\falcon\DEVICE_RC_TRANSMITTER.h>
#include <..\falcon\DEVICE_SERIAL_RECEIVER.h>
#include <..\falcon\DEVICE_SERIAL_TRANSMITTER.h>
#include <..\falcon\DEVICE_UART_RC_BRIDGE.cpp>
#include <..\falcon\DEVICE_UART_RC_BRIDGE.h>
#include <..\falcon\EEPROMAnything.h>
#include <..\falcon\FSLPsensor.cpp>
#include <..\falcon\FSLPsensor.h>
#include <..\falcon\I2Cdev.cpp>
#include <..\falcon\I2Cdev.h>
#include <..\falcon\IMU.cpp>
#include <..\falcon\IMU.h>
#include <..\falcon\LIDARLite.cpp>
#include <..\falcon\LIDARLite.h>
#include <..\falcon\Lidar.cpp>
#include <..\falcon\Lidar.h>
#include <..\falcon\MPU6000.cpp>
#include <..\falcon\MPU6000.h>
#include <..\falcon\MPU6050.cpp>
#include <..\falcon\MPU6050.h>
#include <..\falcon\MPU6050_6Axis_MotionApps20.h>
#include <..\falcon\MPU6050_9Axis_MotionApps41.h>
#include <..\falcon\PPMInput.cpp>
#include <..\falcon\PPMInput.h>
#include <..\falcon\PPMOutput.cpp>
#include <..\falcon\PPMOutput.h>
#include <..\falcon\SerialCommand.cpp>
#include <..\falcon\SerialCommand.h>
#include <..\falcon\UI.cpp>
#include <..\falcon\UI.h>
#include <..\falcon\VehicleLink.cpp>
#include <..\falcon\VehicleLink.h>
#include <..\falcon\falconcommon.h>
#include <..\falcon\helper_3dmath.h>
#include <..\falcon\util.h>
#include <..\falcon\variables.h>
#endif
