/* Joystick control mode
*
*  This receives speed commands from the RPi and translates them into motor
*  controls. On the trace channel it prints out the number of interrupts seen
*  per second = pulses/sec to the stepper motor controller
*/

#include "Joystick.h"
#include "Motors.h"
#include "CmdUSB.h"

Joystick::Joystick()
{
	sensitivity = 100; 
	_normdir = true;
	servoon = SERVOON;
	servooff = SERVOOFF;
}

void Joystick::start()
{
	SerialUSB.println("Start Joystick Mode");
	servo.attach(SERVOPIN);
	servo.write(servooff);
	motors.stop();
	motors.setAcceleration(JOYACCELERATION, JOYROTACCELERATION);
}

void Joystick::stop()
{
	SerialUSB.println("Stop Joystick Mode");
	servo.detach();
}

void Joystick::loop()
{
	static int last_secs = 0;
	static int loopcount = 0;

	loopcount++;
	int secs = millis() / 1000;
	if (secs != last_secs)
	{
		uint32_t mLC = motors.currentPositionL();
		uint32_t mRC = motors.currentPositionR();
		motors.setCurrentPosition(0, 0);

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
	else if (args[0][0] == 'V' && arg_cnt >= 2)
	{
		setSensitivity(cmdStr2Num(args[1], 10));
	}
	else if (args[0][0] == 'C' && arg_cnt >= 3)
	{
		servooff = cmdStr2Num(args[1], 10);
		servoon = cmdStr2Num(args[2], 10);
		SerialUSB.print("Set Servo Off & On angle to : ");
		SerialUSB.print(servooff);
		SerialUSB.print(", ");
		SerialUSB.println(servoon);
	}
}

void Joystick::setdmh(bool setting)
{
	if (setting)
	{
		servo.write(servoon);
		SerialUSB.println("Servo on");
	}
	else
	{
		servo.write(servooff);
		SerialUSB.println("Servo off");
	}
}

void Joystick::setDirection(bool direction)
{
	_normdir = direction;
}

void Joystick::setSensitivity(int s)
{
	sensitivity = s;
	SerialUSB.print("Set sensitivity to ");
	SerialUSB.println(sensitivity);
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
			speed = cmdStr2Num(args[1], 10) * sensitivity / 100;
			direction = 0;
			SerialUSB.println(speed);
		}
		else if (arg_cnt == 3)
		{
			speed = cmdStr2Num(args[1], 10) * sensitivity / 100;
			direction = cmdStr2Num(args[2], 10)  * sensitivity / 100;
			SerialUSB.print(speed);
			SerialUSB.print("  ");
			SerialUSB.println(direction);
		}
		else
			return;
		
		if (_normdir)
			motors.setSpeedDirection(speed, direction);
		else
			motors.setSpeedDirection(-speed, direction);

		motors.setEnableOutputs(true);
	}
}

Joystick joystick;

