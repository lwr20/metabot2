#include "testmode.h"
#include "Motors.h"
#include "MPU9250.h"

#define CONFIGSPEED 10
#define CONFIGCLICKS 6400

void Testmode::start()
{
	SerialUSB.println("Start Test Mode");

	motors.stop();
	motors.setEnableOutputs(false);
	motors.L->setSpeed(CONFIGSPEED);
	motors.R->setSpeed(-CONFIGSPEED);
	mLCount = 0;
	mRCount = 0;
	SerialUSB.println("mx, my, mz, left_wheel, right_wheel");
	running = true;
}

void Testmode::stop()
{
	SerialUSB.println("Stop Test Mode");
}

void Testmode::loop()
{
	int16_t data[3];
	int i;

	if (!running)
		return;

	if (mLCount > CONFIGCLICKS || mRCount > CONFIGCLICKS)
	{
		motors.L->setSpeed(0);
		motors.R->setSpeed(0);
		motors.setEnableOutputs(false);
		running = false;
		SerialUSB.println("Data Complete");
		return;
	}

	if (MPU9250.readMagData(data))
	{
		for (i = 0; i < 3; i++)
		{
			SerialUSB.print(data[i]); 
			SerialUSB.print(", ");
		}
		SerialUSB.print(mLCount);
		SerialUSB.print(", ");
		SerialUSB.println(mRCount);
	}
}

void Testmode::cmd(int arg_cnt, char **args)
{
	// Check for Dead Man's Handle
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'D':
		// Dead Man's Handle
		motors.setEnableOutputs(args[1][0] == '1');
		break;

	}
}

Testmode testmode;

