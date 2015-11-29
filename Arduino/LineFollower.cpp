/*  LineFollower.cpp
*
*	This mode reads the array of 5 light detectors on the base of the metabot and uses them
*   to work out where the robot is in relation to the line.  It then controls the motors to turn the 
*   robot back towards the line - the further the line is from the center the harder it tries to turn.
*   If we lose track of the line altogether, then we reverse straight back until we find the line again.
*
*   The code includes auto-configuration.  It does this be detecting when we are on the board (combination
*   of the light level and the reaction to the illuminating LED being turned on and off). It then detects
*   the change in light levels between being on and off the line and uses that to scale subsequent readings.
*
*   Finally we've got some funky bar graph tracing, so that we can see the reaction of the light detectors
*   easily in the trace.
*/

#include "LineFollower.h"
#include "CmdUSB.h"
#include "Motors.h"

extern int analogpins[NOPINS];

void LineFollower::start()
{
	// Initialise trace variables
	memset(bar, '|', BARLEN);

	// Init other variables
	setState(inactive);
	dmh = false;
	direrror = 0;
	motorspeed = 0;
	motordir = 0;
	setspeed = LFSPEED;

	// Turn on line follower light
	pinMode(LFENABLEPIN, OUTPUT);
	digitalWrite(LFENABLEPIN, 1);

	// Set fast acceleration
	motors.stop();
	motors.setAcceleration(2000,0);

	SerialUSB.println("Start Line Follower Mode");
}

void LineFollower::stop()
{
	digitalWrite(LFENABLEPIN, 0);
	SerialUSB.println("Stop Line Follower Mode");
}

void LineFollower::loop()
{
	int i;
	int norm;
	float direction;

	int first = 0;
	int firsti = 0;
	int second = 0;
	int secondi = 0;
	int meannorm;

	if (state == inactive)
	{
		// Check if we are on the board and ready to auto-configure
		if (reflection() && steadyReadings())
		{
			setState(config);
		} 
		else
		{
			printbars();
			return;
		}
	}

	// Read the light levels from the LED array
	meannorm = 0;
	for (i = 0; i < NOPINS; i++)
	{
		pinval[i] = analogRead(analogpins[i]);
		if (state == config)
		{
			if (pinval[i] > pinmax[i])
				pinmax[i] = pinval[i];
			if (pinval[i] < pinmin[i])
				pinmin[i] = pinval[i];
		}
		// if we have previous max and min values then calculate a normalised
		// reading that falls between these two.
		if (pinmax[i] == pinmin[i])
			norm = 0;
		else
			norm = (pinval[i] - pinmin[i]) * 1000 / (pinmax[i] - pinmin[i]);
		// if we are not in config state, then the actual reading could lie
		// outside the previous max and min values
		if (norm < 0)
			norm = 0;
		if (norm > 1000)
			norm = 1000;
		pinnrm[i] = norm;
		meannorm += norm;

		// keep track of the detector with the highest and second highest readings
		if (norm > first)
		{
			second = first;
			secondi = firsti;
			first = norm;
			firsti = i;
		}
		else if (norm > second)
		{
			second = norm;
			secondi = i;
		}
	}
	meannorm /= 5;


	// Use the mean of the norms to decide whether to go into inactive mode or not
	if (meannorm > INACTIVETHRESH  && !reflection())
	{
		setState(inactive);
	}

	// Calculate Direction using the highest and second highest readings
	direction = firsti;

	// If the second highest reading is next to the highest reading
	// then assume the line lies between them.  Use linear interpolation
	// to figure out where. 
	if (secondi == (firsti + 1))
	{
		direction += (float)second / (first * 2);
	}
	else if (secondi == (firsti - 1))
	{
		direction -= (float)second / (first * 2);
	}
	direrror = direction - 2;

	// set motor speed and direction

	if (direrror >= -1.0 && direrror <= 1.0)
	{
		// if direrror is between -1 and +1 then
		// continue forward at max speed but add direction proportional to the error
		motorspeed = setspeed;
		motordir = setspeed * direrror;
	}
	else
	{
		// if direrror is less than -1 or greater than +1 then
		// set maximum direction and reduce forward speed proportionally
		motorspeed = (2 - abs(direrror)) * setspeed;
		motordir = copysign(setspeed, direrror);
	}

	if (first + second < OFFLINE)
	{
		// We've lost the line, go into reverse
		motorspeed = -setspeed;
		motordir = 0;
	}

	if (state == active && dmh)
	{
		motors.setSpeed(motorspeed);
		motors.setDirection(motordir);
	}

	printbars();
}

void LineFollower::cmd(int arg_cnt, char **args)
{
	// Check for Dead Man's Handle (DMH) and Config commands
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'P':
		// Set Speed
		speedcmd(arg_cnt, args);
		break;
	}
}

void LineFollower::setdmh(bool dmhset)
{
	dmh = dmhset;

	if (dmhset)
	{
		motors.setEnableOutputs(true);
		if (state == config)
			setState(active);
	}
	else
	{
		motors.stop();
	}
}

void LineFollower::speedcmd(int arg_cnt, char **args)
{
	if (arg_cnt < 2)
		return;

	setspeed = cmdStr2Num(args[1], 10);
}

void LineFollower::printbars()
{
	int i;

	static int last_secs = 0;
	static int loopcount = 0;

	loopcount++;

	int secs = millis() / 1000;
	if (secs != last_secs)
	{
		SerialUSB.print("Loops per second: ");
		SerialUSB.println(loopcount);
		last_secs = secs;
		loopcount = 0;
	}

	if (dmh)
		// we are running, so don't put out any bar graph diags because these reduce
		// the response time by a factor of 40!
		return;


	SerialUSB.println("\x1b[2K");
	// Draw out the actual readings
	for (i = 0; i < NOPINS; i++)
	{
		bar[pinval[i] / 12] = 0;
		SerialUSB.print("\x1b[2K");  // Delete Line
		SerialUSB.print(pinval[i]);
		SerialUSB.print("\t");
		SerialUSB.print(pinmin[i]);
		SerialUSB.print("\t");
		SerialUSB.print(pinmax[i]);
		SerialUSB.print("\t");
		SerialUSB.print(bar);
		SerialUSB.println();
		bar[pinval[i] / 12] = '|';
	}
	SerialUSB.println("\x1b[2K");

	// Draw the normalised readings
	for (i = 0; i < NOPINS; i++)
	{
		int endbar = round(pinnrm[i] / 12);
		bar[endbar] = 0;
		SerialUSB.print("\x1b[2K");  // Delete Line
		if (state != inactive)
		{
			SerialUSB.print(pinnrm[i]);
			SerialUSB.print("\t");
			SerialUSB.print(pinmin[i]);
			SerialUSB.print("\t");
			SerialUSB.print(pinmax[i]);
			SerialUSB.print("\t");
			SerialUSB.print(bar);
		}
		SerialUSB.println();
		bar[endbar] = '|';
	}
	// Write out some other interesting values
	SerialUSB.println("\x1b[2K");
	SerialUSB.print("\x1b[2Kstate: ");
	SerialUSB.print(state);
	SerialUSB.print("\t direrror : ");
	SerialUSB.print(direrror);
	SerialUSB.print("\t speed : ");
	SerialUSB.print(motorspeed);
	SerialUSB.print("\t direction : ");
	SerialUSB.println(motordir);
	SerialUSB.print("\x1b[14F");  // Scroll back 14 lines
}

bool LineFollower::reflection()
{
	// Check if we are away from the surface.
	// Test is that blinking the light makes a difference to all 5 LEDS

	int i;
	int darkpinval;
	bool retval = true;

	for (i = 0; i < NOPINS; i++)
	{
		pinval[i] = analogRead(analogpins[i]);
	}

	digitalWrite(LFENABLEPIN, 0);
	delayMicroseconds(500);            // Need to wait a bit for the LED to turn off

	for (i = 0; i < NOPINS; i++)
	{
		//  We've turned off the LED.  If read values don't go up by more than
		// The error margin, then we aren't getting a reflection
		darkpinval = analogRead(analogpins[i]);
		if ((darkpinval - REFLECTMARGIN) < pinval[i])
		{
			retval = false;
		}
	}
	digitalWrite(LFENABLEPIN, 1);
	delayMicroseconds(500);            // Wait a bit for the LED to turn back on
	return retval;
}

bool LineFollower::steadyReadings()
{
	// Check if the readings all stay relatively constant for a period

	int i;
	int j;

	static int startMillis;
	static int startpinval[NOPINS];

	for (i = 0; i < NOPINS; i++)
	{
		if (abs(pinval[i] - startpinval[i]) > STEADYMARGIN)
		{
			// Pin is not within range of the starting position
			// Reset the clock and the starting position
			startMillis = millis();
			for (j = 0; j < NOPINS; j++)
				startpinval[j] = pinval[j];
			return false;
		}
	}

	// All the pins are within ERRORMARGIN of the starting value
	// Now check for how long
	if ((millis() - startMillis) < STEADYTIME)
		return false;

	// Hooray, we have found the table, so ready to start configuring
	return true;

}

void LineFollower::setState(State newstate)
{
	int i;
	state = newstate;

	if (state == inactive || state == config)
	{
		for (i = 0; i < NOPINS; i++)
		{
			pinmin[i] = 1024;
			pinmax[i] = 0;
		}
	}

	if (state == inactive)
	{
		dmh = false;
		motors.stop();
	}
}


LineFollower lineFollower;

