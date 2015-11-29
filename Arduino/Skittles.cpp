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
}

void Skittles::setdmh(bool setting)
{

	if (setting)
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

