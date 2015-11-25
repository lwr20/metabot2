#include "Config.h"
#include "Motors.h"
#include "MPU9250.h"

#define CONFIGSPEED 10
#define CONFIGCLICKS 5100

#define XOFFSET 949
#define YOFFSET 1876
#define ZOFFSET 1230

void Config::start()
{
	SerialUSB.println("Start Test Mode");

	motors.stop();
	motors.setCurrentPosition(0, 0);
	SerialUSB.println("mx, my, mz, left_wheel, right_wheel");
	state = paused;
	rotspeed = CONFIGSPEED;

	xmin = 10000;
	xmax = -10000;
	ymin = 10000;
	ymax = -10000;
	zmin = 10000;
	zmax = -10000;

	xoffset = XOFFSET;
	yoffset = YOFFSET;
	zoffset = ZOFFSET;

	MPU9250.setMagBias(0, 0, 0);
}

void Config::stop()
{
	SerialUSB.println("Stop Test Mode");
}

void Config::loop()
{
	int16_t data[3];
	double angle;

	int i;

	if (state == stopped)
	{
		if (MPU9250.readMagData(data))
		{
			angle = atan2(data[0], data[1]) * 180 / PI;
			SerialUSB.print("Angle : "); SerialUSB.println(angle);
			SerialUSB.print("\x1b[1F");  // Scroll back 1 line
		}
		return;
	}

	if (motors.currentPositionL() >= CONFIGCLICKS || motors.currentPositionL() >= CONFIGCLICKS)
	{
		motors.stop();
		state = stopped;
		SerialUSB.println("Data Complete");
		SerialUSB.print("xmin : "); SerialUSB.println(xmin);
		SerialUSB.print("xmax : "); SerialUSB.println(xmax);
		SerialUSB.print("ymin : "); SerialUSB.println(ymin);
		SerialUSB.print("ymax : "); SerialUSB.println(ymax);
		SerialUSB.print("zmin : "); SerialUSB.println(zmin);
		SerialUSB.print("zmax : "); SerialUSB.println(zmax);

		xoffset = (xmax + xmin) / 2;
		yoffset = (ymax + ymin) / 2;
		zoffset = (zmax + zmin) / 2;

		SerialUSB.print("x offset : "); SerialUSB.println(xoffset);
		SerialUSB.print("y offset : "); SerialUSB.println(yoffset);
		SerialUSB.print("z offset : "); SerialUSB.println(zoffset);

		MPU9250.setMagBias(xoffset, yoffset, zoffset);

		return;
	}

	if (state == running && MPU9250.readMagData(data))
	{
		for (i = 0; i < 3; i++)
		{
			SerialUSB.print(data[i]);
			SerialUSB.print(", ");
		}
		SerialUSB.print(motors.currentPositionL());
		SerialUSB.print(", ");
		SerialUSB.println(motors.currentPositionR());

		if (data[0] < xmin)
			xmin = data[0];
		if (data[0] > xmax)
			xmax = data[0];
		if (data[1] < ymin)
			ymin = data[1];
		if (data[1] > ymax)
			ymax = data[1];
		if (data[2] < zmin)
			zmin = data[2];
		if (data[2] > zmax)
			zmax = data[2];
	}
}

void Config::cmd(int arg_cnt, char **args)
{
	// Check for Dead Man's Handle
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'D':
		// Dead Man's Handle
		if (args[1][0] == '1')
		{
			motors.setSpeedDirection(0.0, rotspeed);
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


Config config;

