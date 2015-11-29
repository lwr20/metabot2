#include "SpeedTest.h"
#include "Motors.h"
#include "Lights.h"
#include "CmdUSB.h"

#define MAXSPEED 20000
#define ERRTHRESHOLD 20

// Configure the curve of allowable accelerations
// Speedthreshold is a set of speed points on the curve
// Accelrate is the acceleration to use up to that speed
// Note that 0 accel has a special meaning, which is to use the m_topaccel value that can be set through the command line
int speedthreshold[] = {1000,	1300,	1600, 2000, 100000};
int accelrate[]		  = {1000,	800,	500,  300,    0};

void SpeedTest::start()
{
	SerialUSB.println("Start Speed Test Mode");
	motors.setCurrentPosition(0, 0);
	motors.setAcceleration(2000,0);
  motors.setDirection(0);
	m_accelindex = 0;
	m_topaccel = 10;
	m_running = false;

	// Enable lights
	lights.setEnabled(true);
}

void SpeedTest::stop()
{
	SerialUSB.println("Stop Speed Test Mode");
	// Disable lights
	lights.setEnabled(false);
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
		int newrate;
		m_accelindex = i;
		if (accelrate[i] > 0)
			newrate = accelrate[i];
		else
			newrate = m_topaccel;
		SerialUSB.print("Speed = ");
		SerialUSB.print(speed);
		SerialUSB.print("  Shifting gear to : ");
		SerialUSB.print(i+1);
		SerialUSB.print("  New acceleration =  ");
		SerialUSB.println(newrate);
		motors.setAcceleration(newrate,0);
	}

	// Check side sensors
	lights.getvals();
	int leftlight = lights.lightval[0];
	int rightlight = lights.lightval[4];
	int error = leftlight - rightlight;     // Look at the difference between the left and right detectors
	if (abs(error) > ERRTHRESHOLD)
	{
		int newdir = ((error - 20) * speed) / 2500;
		if (newdir != motors.currentDirection())
		{
			//SerialUSB.print("Set Direction : ");
			//SerialUSB.println(newdir);
			motors.setDirection(newdir);
		}
	}

	// Print out some trace
	static int last_time = 0;
	int time = millis() / 100;

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
	// Configure Acceleration for testing mode
	// Used when A = 0 in the table at the top of this file
	if (args[0][0] == 'C' && arg_cnt >= 2)
	{
		m_topaccel = cmdStr2Num(args[1], 10);
    SerialUSB.print("Set Top acceleration : ");
    SerialUSB.println(m_topaccel);
	}
}

void SpeedTest::setdmh(bool setting)
{
	if (setting)
	{
		motors.setAcceleration(accelrate[0],0);
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

