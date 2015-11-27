#include "SpeedTest.h"
#include "Motors.h"
#include "Lights.h"

#define MAXSPEED 20000
#define ERRTHRESHOLD 20

// Configure the curve of allowable accelerations
int speedthreshold[] = {1000,	1300,	1600,	2000,	100000};
int accelrate[]		 = {2000,	500,	100,	10,		10};

void SpeedTest::start()
{
	SerialUSB.println("Start Speed Test Mode");
	motors.setCurrentPosition(0, 0);
	motors.setAcceleration(1000);
	m_accelindex = 0;
	m_running = false;

	// Enable lights
	lights.setEnabled(true);

}

void SpeedTest::stop()
{
	SerialUSB.println("Stop Speed Test Mode");
	// Disable lights
	lights.setEnabled(true);
}

void SpeedTest::loop()
{
	// Get current speed and check whether we need to move to the next level of acceleration
	int speed = motors.currentSpeed();

	int i = 0;
	while (speed > speedthreshold[i])
		i++;

	if (i != m_accelindex)
	{
		m_accelindex = i;
		motors.setAcceleration(accelrate[i]);
		SerialUSB.print("Shifting gear to : ");
		SerialUSB.println(i+1);
	}

	// Check side sensors
	lights.getvals();
	int leftlight = lights.lightval[0];
	int rightlight = lights.lightval[4];
	int error = leftlight - rightlight;     // Look at the difference between the left and right detectors
	if (abs(error) > ERRTHRESHOLD)
	{
		int newdir = (error * speed) / 5000;
		if (newdir != motors.currentDirection())
		{
			//SerialUSB.print("Set Direction : ");
			//SerialUSB.println(newdir);
			motors.setDirection(newdir);
		}
	}

	// Print out some trace
	static int last_time = 0;
	int time = millis() / 500;

	if (time != last_time)
	{
		last_time = time;
		if (m_running)
		{
			SerialUSB.print("Current Speed : "); SerialUSB.print(speed);
			SerialUSB.print("   Lights : "); SerialUSB.print(leftlight); SerialUSB.print(", "); SerialUSB.print(rightlight);
			SerialUSB.print("   Error : "); SerialUSB.println(error);
		}
	}
}

void SpeedTest::cmd(int arg_cnt, char **args)
{
}

void SpeedTest::setdmh(bool setting)
{
	if (setting)
	{
		motors.setAcceleration(accelrate[0]);
		motors.setSpeedDirection(MAXSPEED, 0);
		motors.setEnableOutputs(true);
		m_running = true;
	}
	else
	{
		int speed = motors.currentSpeed();
		SerialUSB.print("Stopping at speed : ");
		SerialUSB.println(speed);
		motors.stop();
		m_running = false;
		m_accelindex = 0;
	}
}



SpeedTest speedTest;

