// Joystick control mode

#include "Joystick.h"
#include "Motors.h"
#include "CmdUSB.h"

void Joystick::start()
{
	SerialUSB.println("Start Joystick Mode");

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
		SerialUSB.print(mLC / L_MICROSTEP);
		SerialUSB.print(")\t");
		SerialUSB.print(mRC);
		SerialUSB.print(" (");
		SerialUSB.print(mRC / R_MICROSTEP);
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

void Joystick::setForward(int arg_cnt, char **args) {

	float lspeed;
	float rspeed;

	SerialUSB.print("F : ");
	if (arg_cnt == 1)
	{
		// if no args, stop
		// and disable steppers - this saves power, but allows stepper to freewheel
		motors.stop();
		motors.setEnableOutputs(false);
		SerialUSB.println("Stop");
	}
	else
	{
		if (arg_cnt == 2)
		{
			lspeed = float(cmdStr2Num(args[1], 10));
			rspeed = lspeed;
			SerialUSB.println(lspeed);
		}
		else if (arg_cnt == 3)
		{
			lspeed = float(cmdStr2Num(args[1], 10));
			rspeed = float(cmdStr2Num(args[2], 10));
			SerialUSB.print(lspeed);
			SerialUSB.print("  ");
			SerialUSB.println(rspeed);
		}
		else
			return;
		motors.L->setSpeed(lspeed);
		motors.R->setSpeed(rspeed);
		motors.setEnableOutputs(true);
	}
}

Joystick joystick;

