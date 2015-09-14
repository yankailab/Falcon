#include "IMU.h"

#ifdef USE_MPU6050_I2C
#include "MPU6050_9Axis_MotionApps41.h"
MPU6050 g_mpu;
#endif

#ifdef USE_MPU9000_I2C
#include "MPU6050_9Axis_MotionApps41.h"
#endif

#ifdef USE_MPU6000_SPI
#include "MPU6000.h"

MPU6000 g_mpu;
#endif

//#define DEBUG_IMU

/* */
IMU::IMU(void)
{
}

/* */
IMU::~IMU(void)
{
}

bool IMU::init(void)
{
	const180ovPI = 180/M_PI;

#ifdef USE_MPU6000_SPI
	// we need to stop the barometer from holding the SPI bus
	pinMode(40, OUTPUT);
	digitalWrite(40, HIGH);

	g_mpu.initialize();
	m_dmpReady = true;
#endif

#ifdef USE_MPU6050_I2C
	m_dmpReady = false;	// set true if DMP init was successful

	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24;						// 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	#endif

	// initialize device
#ifdef DEBUG_IMU
	Serial.println(F("Init I2C"));
#endif
	g_mpu.initialize();

	// verify connection
#ifdef DEBUG_IMU
	Serial.println(F("Test I2C"));
	Serial.println(g_mpu.testConnection() ? F("IMU successful") : F("IMU failed"));
#endif

	// wait for ready
#ifdef DEBUG_IMU
	Serial.println(F("\nSend any character to begin DMP programming and demo: "));
	while (Serial.available() && Serial.read());

	// empty buffer
	while (!Serial.available());

	// wait for data
	while (Serial.available() && Serial.read());

	// empty buffer again
	// load and configure the DMP
	Serial.println(F("Init DMP"));
#endif
	m_devStatus = g_mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	g_mpu.setXGyroOffset(0);//(220);
	g_mpu.setYGyroOffset(0);//(76);
	g_mpu.setZGyroOffset(0);//(-85);
	g_mpu.setZAccelOffset(0);//(1788);	// 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (m_devStatus == 0)
	{
		// turn on the DMP, now that it's ready
#ifdef DEBUG_IMU
		Serial.println(F("Enabling DMP..."));
#endif
		g_mpu.setDMPEnabled(true);
		m_mpuIntStatus = g_mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef DEBUG_IMU
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
		m_dmpReady = true;

		// get expected DMP packet size for later comparison
		m_packetSize = g_mpu.dmpGetFIFOPacketSize();

		m_pMPU = &g_mpu;
		return true;
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
#ifdef DEBUG_IMU
		Serial.print(F("DMP failed (code "));
		Serial.print(m_devStatus);
		Serial.println(F(")"));
#endif
	}

	return false;
#endif
}


/* */
void IMU::update(void)
{
#ifdef USE_MPU6000_SPI
	g_mpu.getYPR();
	m_ypr[0] = g_mpu.m_yaw;
	m_ypr[1] = g_mpu.m_pitch;
	m_ypr[2] = g_mpu.m_roll;
#endif

#ifdef USE_MPU6050_I2C

	m_mpuIntStatus = g_mpu.getIntStatus();

	// get current FIFO count
	m_fifoCount = g_mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((m_mpuIntStatus & 0x10) || m_fifoCount == 1024)
	{
		// reset so we can continue cleanly
		g_mpu.resetFIFO();
#ifdef DEBUG_IMU
		Serial.println(F("FIFO overflow!"));
		Serial.println(m_fifoCount);
#endif
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (m_mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (m_fifoCount < m_packetSize)
		{
			m_fifoCount = g_mpu.getFIFOCount();
		}

		// read a packet from FIFO
		g_mpu.getFIFOBytes(m_fifoBuffer, m_packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		m_fifoCount -= m_packetSize;

		#ifdef OUTPUT_READABLE_QUATERNION

		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		Serial.print("quat\t");
		Serial.print(q.w);
		Serial.print("\t");
		Serial.print(q.x);
		Serial.print("\t");
		Serial.print(q.y);
		Serial.print("\t");
		Serial.println(q.z);
		#endif

		#ifdef OUTPUT_READABLE_EULER

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		Serial.print("euler\t");
		Serial.print(euler[0] * 180 / M_PI);
		Serial.print("\t");
		Serial.print(euler[1] * 180 / M_PI);
		Serial.print("\t");
		Serial.println(euler[2] * 180 / M_PI);
		#endif

		#ifdef OUTPUT_READABLE_YAWPITCHROLL

		// display Euler angles in degrees
		g_mpu.dmpGetQuaternion(&m_q, m_fifoBuffer);
		g_mpu.dmpGetGravity(&m_gravity, &m_q);
		g_mpu.dmpGetYawPitchRoll(m_ypr, &m_q, &m_gravity);
		m_ypr[0] *= const180ovPI;
		m_ypr[1] *= const180ovPI;
		m_ypr[2] *= const180ovPI;
		#endif

		#ifdef OUTPUT_READABLE_REALACCEL

		// display real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.print(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);
		#endif

		#ifdef OUTPUT_READABLE_WORLDACCEL

		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);
		#endif

		#ifdef OUTPUT_TEAPOT

		// display quaternion values in InvenSense Teapot demo format:
		teapotPacket[2] = fifoBuffer[0];
		teapotPacket[3] = fifoBuffer[1];
		teapotPacket[4] = fifoBuffer[4];
		teapotPacket[5] = fifoBuffer[5];
		teapotPacket[6] = fifoBuffer[8];
		teapotPacket[7] = fifoBuffer[9];
		teapotPacket[8] = fifoBuffer[12];
		teapotPacket[9] = fifoBuffer[13];
		Serial.write(teapotPacket, 14);
		teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		#endif

	}

#endif
}
