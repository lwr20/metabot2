/*  Skittles.cpp
*
*   Mode control for the skittles event.  Basic Joystick function + servo control
*   for the firing mechanism
*/

#include "Skittles.h"

#define SERVOPIN 13

void Skittles::start()
{
	SerialUSB.println("Start Skittles Mode");
	servo.attach(SERVOPIN);
}

void Skittles::stop()
{
	SerialUSB.println("Stop Skittles Mode");
	servo.detach();
}

void Skittles::loop()
{
	//ToDo
}

void Skittles::cmd(int arg_cnt, char **args)
{
	// Check for Servo command
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'D':
		// Servo
		servocmd(arg_cnt, args);
		break;
	}
}

void Skittles::servocmd(int arg_cnt, char **args)
{

	if (arg_cnt < 2)
		return;

	if (args[1][0] == '1')
	{
		servo.write(30);
		SerialUSB.println("Servo on");
	}
	else
	{
		servo.write(120);
		SerialUSB.println("Servo off");
	}
}

Skittles skittles;

