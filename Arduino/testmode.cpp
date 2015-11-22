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
}

void Testmode::cmd(int arg_cnt, char **args)
{
	// Check for Dead Man's Handle
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'D':
		// Dead Man's Handle
		if (args[1][0] == '1')
		{
			motors.setSpeed(m_currentSpeed, m_currentDirection);
			motors.setEnableOutputs(true);
			m_running = true;
		}
		else
		{
			motors.stop();
			m_running = false;
		}
		break;

	}
}

Testmode testmode;

