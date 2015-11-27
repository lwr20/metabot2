#include "testmode.h"
#include "Motors.h"
#include "MPU9250.h"

#define CONFIGSPEED 10
#define CONFIGCLICKS 3200

#define XOFFSET 539
#define YOFFSET 1060

void Testmode::start()
{
	SerialUSB.println("Start Test Mode");
	motors.setCurrentPosition(0, 0);
	motors.setAcceleration(100, 20);
	m_currentSpeed = 0;
	m_currentDirection = 0;
	m_running = false;

}

void Testmode::stop()
{
	SerialUSB.println("Stop Test Mode");
}

void Testmode::loop()
{
	static int last_time = 0;
	int time = millis() / 500;
	int16_t accelCount[3];

	if (time != last_time && MPU9250.isDataReady())
	{
		last_time = time;
		MPU9250.readAccelData(accelCount);
		int ax = accelCount[0];
		int ay = accelCount[1]; 
		int az = accelCount[2];
		SerialUSB.print(ax); SerialUSB.print(", "); SerialUSB.print(ay); SerialUSB.print(", "); SerialUSB.println(az);
	}
}

void Testmode::cmd(int arg_cnt, char **args)
{
}

void Testmode::setdmh(bool setting)
{
	if (setting)
	{
		motors.setSpeedDirection(m_currentSpeed, m_currentDirection);
		motors.setEnableOutputs(true);
		m_running = true;
	}
	else
	{
		motors.stop();
		m_running = false;
	}

}


Testmode testmode;

