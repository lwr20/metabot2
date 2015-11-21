/* Joystick control mode
*
*  This receives speed commands from the RPi and translates them into motor
*  controls. On the trace channel it prints out the number of interrupts seen
*  per second = pulses/sec to the stepper motor controller
*/

#include "Joystick.h"
#include "Motors.h"
#include "CmdUSB.h"

void Joystick::start()
{
	SerialUSB.println("Start Joystick Mode");
	motors.stop();
	motors.setAcceleration(ACCELERATION);
}

void Joystick::stop()
{
	SerialUSB.println("Stop Joystick Mode");
}

void Joystick::loop()
{
	static int last_secs = 0;
	static int loopcount = 0;

	loopcount++;
	int secs = millis() / 1000;
	if (secs != last_secs)
	{
		noInterrupts();
		unsigned long mLC = mLCount;
		mLCount = 0;
		unsigned long mRC = mRCount;
		mRCount = 0;
		interrupts();
		last_secs = secs;
		SerialUSB.print(mLC);
		SerialUSB.print(" (");
		SerialUSB.print(mLC / MICROSTEP);
		SerialUSB.print(")\t");
		SerialUSB.print(mRC);
		SerialUSB.print(" (");
		SerialUSB.print(mRC / MICROSTEP);
		SerialUSB.print(")\t loops : ");
		SerialUSB.println(loopcount);
		loopcount = 0;
	}
}

void Joystick::cmd(int arg_cnt, char **args)
{
	if (args[0][0] == 'F')
		setForward(arg_cnt, args);
}

void Joystick::setDirection(bool direction)
{
	_normdir = direction;
}

void Joystick::setForward(int arg_cnt, char **args) {

	float speed;
	float direction;

	SerialUSB.print("F : ");
	if (arg_cnt == 1)
	{
		// if no args, stop
		// and disable steppers - this saves power, but allows stepper to freewheel
		motors.stop();
		SerialUSB.println("Stop");
	}
	else
	{
		if (arg_cnt == 2)
		{
			speed = float(cmdStr2Num(args[1], 10));
			direction = 0;
			SerialUSB.println(speed);
		}
		else if (arg_cnt == 3)
		{
			speed = float(cmdStr2Num(args[1], 10));
			direction = float(cmdStr2Num(args[2], 10));
			SerialUSB.print(speed);
			SerialUSB.print("  ");
			SerialUSB.println(direction);
		}
		else
			return;
		
		if (_normdir)
			motors.setSpeed(speed, direction);
		else
			motors.setSpeed(-speed, -direction);

		motors.setEnableOutputs(true);
	}
}

Joystick joystick;

