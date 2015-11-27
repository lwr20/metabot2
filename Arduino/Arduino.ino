/* Arduino.ino - Metabot Arduino driver
*
*  This code runs on the Arduino in Metabot and controls all of the real time functions.  It communicates with the Raspberry
*  PI over two serial connections:  
*  - Serial3 used for receiving commands
*  - SerialUSB used for trace.
*
*  The program comprises the following components.
*
*	-  A set of service routines to handle various peripherals 
*		- Motors.cpp to drive the motors, which in turn uses InterruptStepper to drive the two Stepper motors
*		- MPU9250.cpp to drive the motion controller (3 axes for each of acceleration, gyro and magnetometer)
*		- CmdUSB.cpp to process commands coming over the serial interface.
*
*	-  A set of mode controllers.  These are the brains of the program and control the operation of the robot
*      in the different events. There will be mode controllers for each event, but at present these are only
*	   present for the manual Joystick mode and the LineFollower mode, with stub modes for the Skittles and for
*      testing.
*
*  The Arduino goes into a new mode when it receives a mode command from the RPi.  This causes the previous mode to be stopped
*  (call to its stop method) and the new mode to be started (call to the start method).  Thereafter, the loop method of the mode
*  is called from the main Arduino loop, and any command received from the RPi are passed to the mode's cmd method.
*
*  A few commands are trapped at the top level before being passed to the mode, specifically the emergency Stop and
*  some configuration commands (at present just to set acceleration).
*/


#include "SpeedTest.h"
#include "Lights.h"
#include "Proximity.h"
#include "ThreePointTurn.h"
#include "Config.h"
#include <Servo.h>
#include <SPI.h>

#include "MPU9250.h"
#include "Motors.h"
#include "CmdUSB.h"

#include "Skittles.h"
#include "LineFollower.h"
#include "Joystick.h"
#include "testmode.h"

ModeBase * mode = &joystick;

void setup()
{
	SerialUSB.begin(9600);
	SerialUSB.println("Initialising...");

	// CMD Setup - uses Serial3
	cmdInit(115200);

	// Start-up the MPU9250
	MPU9250.init();

	// Initialise the lights
	lights.init();

	// Start your engine
	motors.init();

	// Start the mode
	mode->start();

	SerialUSB.println("Ready to roll...");
}

void loop()
{
	// Run the loop for the current mode
	mode->loop();

	// Check for serial commands
	cmdPoll();

	// Update motor params
	motors.loop();

}

// process_cmd is called from CmdUSB.cpp when a command is received.
void process_cmd(int arg_cnt, char **args)
{
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'M':
		// Set Mode
		if (arg_cnt >= 2)
			set_mode(args[1][0]);
		break;

	case 'S':
		// Emergency stop
		motors.stop();
		SerialUSB.println("Emergency Stop");
		break;

	case 'A':
		// Set Acceleration
		setAcceleration(arg_cnt, args);
		break;

	case 'D':
		// Dead Man's Handle
		if (arg_cnt > 1)
			mode->setdmh(args[1][0] == '1');
		break;

		// Add cases for any generic commands not associated with a specific mode

	default:
		mode->cmd(arg_cnt, args);
		break;
	}
}

void set_mode(char modechar)
{
	mode->stop();
	switch (modechar)
	{
	case 'J':
	case 'R':
		mode = &joystick;
		joystick.setDirection(modechar == 'J');
		break;

	case 'L':
		mode = &lineFollower;
		break;

	case 'B':
		mode = &skittles;
		break;

	case 'T':
		mode = &testmode;
		break;

	case 'C':
		mode = &config;
		break;

	case '3':
		mode = &threePointTurn;
		break;

	case 'P':
		mode = &proximity;
		break;

	case 'S':
		mode = &speedTest;
		break;


	}
	mode->start();
}


void setAcceleration(int arg_cnt, char **args)
{
	if (arg_cnt > 1)
	{
		int acc = cmdStr2Num(args[1], 10);
		if (arg_cnt > 1)
		{
			int racc = cmdStr2Num(args[2], 10);
			motors.setAcceleration(float(acc), float(racc));
			SerialUSB.print("A : ");
			SerialUSB.print(acc);
			SerialUSB.print(", ");
			SerialUSB.println(racc);
		}
		else
		{
			motors.setAcceleration(float(acc));
			SerialUSB.print("A : ");
			SerialUSB.println(acc);
		}
	}
}



