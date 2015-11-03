#include "MPU9250.h"
#include <Servo.h>
#include <SPI.h>

#include "Skittles.h"
#include "LineFollower.h"
#include "Motors.h"
#include "Joystick.h"
#include "CmdUSB.h"
#include "modebase.h"


ModeBase * mode = &joystick;

void setup()
{
  SerialUSB.begin(9600);
  SerialUSB.println("Initialising...");

  // CMD Setup - uses Serial3
  cmdInit(115200);
  
  // Start-up the MPU9250
  MPU9250.init();

  // Start your engine
  motors.init();

  // Start the mode
  mode->start();

  SerialUSB.println("Ready to roll...");
}

void loop() {

  // Run the loop for the current mode
  mode->loop();

  // Check for serial commands
  cmdPoll();

  // Update motor params
  motors.run();
}

void process_cmd(int arg_cnt, char **args) {

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
		mode = &joystick;
		break;

	case 'L':
		mode = &lineFollower;
		break;

	case 'S':
		mode = &skittles;
		break;
	}
	mode->start();
}


void setAcceleration(int arg_cnt, char **args) {
	if (arg_cnt > 1)
	{
		int acc = cmdStr2Num(args[1], 10);
		motors.setAcceleration(float(acc));

		SerialUSB.print("A : ");
		SerialUSB.println(args[1]);
	}
}



