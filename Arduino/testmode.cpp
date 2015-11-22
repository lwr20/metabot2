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
			motors.setSpeed(0.0, CONFIGSPEED);
			motors.setEnableOutputs(true);
		}
		else
		{
			motors.stop();
		}
		break;

	}
}

Testmode testmode;

