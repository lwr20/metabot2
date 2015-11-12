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

int analogpins[NOPINS] = { 4, 3, 2, 1, 0 };    // pin numbers of the line follower analog inputs, in order

void LineFollower::start()
{
	int i;

	// Initialise trace variables
	memset(bar, '|', BARLEN);

	// Init other variables
	state = inactive;
	dmh = false;
	for (i = 0; i < NOPINS; i++)
	{
		pinmin[i] = 1024;
		pinmax[i] = 0;
	}
	direrror = 0;
	speed = LFSPEED;

	// Turn on line follower light
	pinMode(LFENABLEPIN, OUTPUT);
	digitalWrite(LFENABLEPIN, 1);

	// Set instant acceleration
	motors.stop();
	motors.setAcceleration(0.0);

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
	float lspeed;
	float rspeed;

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
			state = config;
			for (i = 0; i < NOPINS; i++)
			{
				pinmax[i] = 0;
				pinmin[i] = 1024;
			}
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
		if (pinmax[i] == pinmin[i])
			norm = 0;
		else
			norm = (pinval[i] - pinmin[i]) * 1000 / (pinmax[i] - pinmin[i]);
		if (norm < 0)
			norm = 0;
		if (norm > 1000)
			norm = 1000;
		pinnrm[i] = norm;
		meannorm += norm;

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


	// Use the mean of the norms to decide whether to go into config mode or not
	if (meannorm > INACTIVETHRESH  && !reflection())
	{
		state = inactive;
		dmh = false;
		motors.stop();
	}

	// Calculate Direction from readings
	direction = firsti;
	if (secondi == (firsti + 1))
	{
		direction += (float)second / (first * 2);
	}
	else if (secondi == (firsti - 1))
	{
		direction -= (float)second / (first * 2);
	}
	direrror = direction - 2;

	lspeed = speed;
	rspeed = speed;
	if (direrror > 0)
	{
		rspeed = speed * (1.0 - direrror);
		lspeed = speed * min(1, 2 - direrror);
	}
	else if (direrror < 0)
	{
		lspeed = speed * (1.0 + direrror);
		rspeed = speed * min(1, 2 + direrror);
	}

	if (first + second < 0.1)
	{
		// We've lost the line
		lspeed = -speed;
		rspeed = -speed;
	}

	if (dmh)
	{
		motors.L->setSpeed(lspeed);
		motors.R->setSpeed(rspeed);
	}

	printbars();
}

void LineFollower::cmd(int arg_cnt, char **args)
{
	// Check for Dead Man's Handle (DMH) and Config commands
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'D':
		// Dead Man's Handle
		dmhcmd(arg_cnt, args);
		break;

	case 'P':
		// Set Speed
		speedcmd(arg_cnt, args);
		break;
	}
}

void LineFollower::dmhcmd(int arg_cnt, char **args)
{
	if (arg_cnt < 2)
		return;

	dmh = (args[1][0] == '1');

	if (!dmh)
	{
		motors.stop();
	}
	else
	{
		motors.setEnableOutputs(true);
		if (state == config)
			state = active;
	}
}

void LineFollower::speedcmd(int arg_cnt, char **args)
{
	if (arg_cnt < 2)
		return;

	speed = float(cmdStr2Num(args[1], 10));
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

	for (i = 0; i < NOPINS; i++)
	{
		int endbar = round(pinnrm[i] / 12);
		bar[endbar] = 0;
		SerialUSB.print("\x1b[2K");  // Delete Line
		if (config != inactive)
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
	SerialUSB.println("\x1b[2K");
	SerialUSB.print("\x1b[2Kstate: ");
	SerialUSB.print(state);
	SerialUSB.print("\t dmh: ");
	SerialUSB.print(dmh);
	SerialUSB.print("\t direrror : ");
	SerialUSB.println(direrror);
	SerialUSB.print("\x1b[14F");  // Scroll back 14 lines
}

bool LineFollower::reflection()
{
	// Check if we are away from the surface.
	// Test is that blinking the light makes a difference to all 5 LEDS

	int i;
	uint32_t darkpinval;

	for (i = 0; i < NOPINS; i++)
	{
		pinval[i] = analogRead(analogpins[i]);
	}

	digitalWrite(LFENABLEPIN, 0);
	delayMicroseconds(200);            // Need to wait a bit for the LED to turn off

	for (i = 0; i < NOPINS; i++)
	{
		//  We've turned off the LED.  If read values don't go up by more than
		// The error margin, we'll assume that we are not close enough to the board to detect it
		darkpinval = analogRead(analogpins[i]);
		if ((darkpinval - ERRORMARGIN) < pinval[i])
		{
			digitalWrite(LFENABLEPIN, 1);
			return false;
		}
	}
	digitalWrite(LFENABLEPIN, 1);
	return true;
}

bool LineFollower::steadyReadings()
{
	// Check if the readings all stay relatively constant for a period

	int i;
	int j;

	static int startMillis;
	static uint32_t startpinval[NOPINS];

	for (i = 0; i < NOPINS; i++)
	{
		if (abs(pinval[i] - startpinval[i]) > ERRORMARGIN)
		{
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

LineFollower lineFollower;

