#pragma once
#include <Arduino.h>


#define USE_MPU6000_SPI
//#define USE_MPU6050_I2C
//#define USE_MPU9000_I2C


#ifdef USE_MPU6050_I2C
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "helper_3dmath.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include <Wire.h>
#endif
#endif


/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using g_pSerial->write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define OUTPUT_READABLE_YAWPITCHROLL


class	IMU
{
public:
	IMU(void);
	~IMU(void);

	bool init(void);
	void update(void);

	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
	// AD0 high = 0x69
	void*		m_pMPU;

	//MPU6050 mpu(0x69); // <-- use for AD0 high
	// MPU control/status vars
	bool		m_dmpReady;				// set true if DMP init was successful
	uint8_t		m_mpuIntStatus;			// holds actual interrupt status byte from MPU
	uint8_t		m_devStatus;			// return status after each device operation (0 = success, !0 = error)
	uint16_t	m_packetSize;			// expected DMP packet size (default is 42 bytes)
	uint16_t	m_fifoCount;			// count of all bytes currently in FIFO
	uint8_t		m_fifoBuffer[64];		// FIFO storage buffer

	// orientation/motion vars
#ifdef USE_MPU6050_I2C
	Quaternion	m_q;				// [w, x, y, z]         quaternion container
	VectorInt16 m_aa;				// [x, y, z]            accel sensor measurements
	VectorInt16 m_aaReal;			// [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 m_aaWorld;			// [x, y, z]            world-frame accel sensor measurements
	VectorFloat m_gravity;			// [x, y, z]            gravity vector
	float		m_euler[3];			// [psi, theta, phi]    Euler angle container
#endif
	float		const180ovPI;
	float		m_ypr[3];			// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};
