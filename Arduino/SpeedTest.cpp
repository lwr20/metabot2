#include "SpeedTest.h"
#include "Motors.h"

void SpeedTest::start()
{
	SerialUSB.println("Start Speed Test Mode");
	motors.setCurrentPosition(0, 0);
	motors.setAcceleration(100, 20);
	m_currentSpeed = 0;
	m_currentDirection = 0;
	m_running = false;

}

void SpeedTest::stop()
{
	SerialUSB.println("Stop Speed Test Mode");
}

void SpeedTest::loop()
{
}

void SpeedTest::cmd(int arg_cnt, char **args)
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




SpeedTest speedTest;

