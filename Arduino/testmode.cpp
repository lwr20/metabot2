#include "testmode.h"
#include "Motors.h"
#include "MPU9250.h"

#define CONFIGSPEED 5
#define CONFIGCLICKS 1600

#define XOFFSET 539
#define YOFFSET 1060

void Testmode::start()
{
	SerialUSB.println("Start Test Mode");

	motors.stop();
	mLCount = 0;
	mRCount = 0;
	SerialUSB.println("mx, my, mz, left_wheel, right_wheel");
	state = paused;

	xmin = 10000;
	xmax = -10000;
	ymin = 10000;
	ymax = -10000;

	xoffset = XOFFSET;
	yoffset = YOFFSET;
}

void Testmode::stop()
{
	SerialUSB.println("Stop Test Mode");
}

void Testmode::loop()
{
	int16_t data[3];
	int i;

	if (state == stopped)
		return;

	if (mLCount > CONFIGCLICKS || mRCount > CONFIGCLICKS)
	{
		motors.L->setSpeed(0);
		motors.R->setSpeed(0);
		motors.setEnableOutputs(false);
		state = stopped;
		SerialUSB.println("Data Complete");
		SerialUSB.print("xmin : "); SerialUSB.println(xmin);
		SerialUSB.print("xmax : "); SerialUSB.println(xmax);
		SerialUSB.print("ymin : "); SerialUSB.println(ymin);
		SerialUSB.print("ymax : "); SerialUSB.println(ymax);

		xoffset = (xmax + xmin) / 2;
		yoffset = (ymax + ymin) / 2;

		SerialUSB.print("x offset : "); SerialUSB.println(xoffset);
		SerialUSB.print("y offset : "); SerialUSB.println(yoffset);

		return;
	}

	if (state == running && MPU9250.readMagData(data))
	{
		for (i = 0; i < 3; i++)
		{
			SerialUSB.print(data[i]); 
			SerialUSB.print(", ");
		}
		SerialUSB.print(mLCount);
		SerialUSB.print(", ");
		SerialUSB.println(mRCount);

		if (data[0] < xmin)
			xmin = data[0];
		if (data[0] > xmax)
			xmax = data[0];
		if (data[1] < ymin)
			ymin = data[1];
		if (data[1] > ymax)
			ymax = data[1];
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
		if (args[1][0] == '1')
		{
			motors.L->setSpeed(CONFIGSPEED);
			motors.R->setSpeed(-CONFIGSPEED);
			motors.setEnableOutputs(true);
			if (state != stopped)
				state = running;
		}
		else
		{
			motors.stop();
			if (state != stopped)
				state = paused;
		}
		break;

	}
}

Testmode testmode;

