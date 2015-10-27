#include "LineFollower.h"
#include "Motors.h"

#define NOPINS 5
int analogpins[NOPINS] = { 0, 1, 2, 3, 4 };    // pin numbers of the line follower analog inputs, in order

#define DEFAULTMIN 300
#define DEFAULTMAX 500

int pinmin[NOPINS];
int pinmax[NOPINS];
int pinval[NOPINS];
int pinnrm[NOPINS];
float direrror;

#define BARLEN 110
char bar[BARLEN];

void LineFollower::start()
{
	int i;

	// Initialise trace variables
	memset(bar, '|', BARLEN);

	// Init other variables
	config = false;
	dmh = false;
	for (i = 0; i < NOPINS; i++)
	{
		pinmin[i] = DEFAULTMIN;
		pinmax[i] = DEFAULTMAX;
	}
	direrror = 0;

	// Turn on line follower light
	pinMode(LFENABLEPIN, OUTPUT);
	digitalWrite(LFENABLEPIN, 1);

	SerialUSB.println("Start Line Follower Mode");
}

void LineFollower::stop()
{
	digitalWrite(33, 0);
	digitalWrite(35, 0);

	SerialUSB.println("Stop Line Follower Mode");
}

void LineFollower::printbars()
{
	int i;
	int n;

	SerialUSB.print("\x1b[13F");  // Scroll back 5 lines
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
		bar[pinnrm[i]] = 0;
		SerialUSB.print("\x1b[2K");  // Delete Line
		SerialUSB.print(pinnrm[i]);
		SerialUSB.print("\t");
		SerialUSB.print(pinmin[i]);
		SerialUSB.print("\t");
		SerialUSB.print(pinmax[i]);
		SerialUSB.print("\t");
		SerialUSB.print(bar);
		SerialUSB.println();
		bar[pinnrm[i]] = '|';
	}
	SerialUSB.println("\x1b[2K");
	SerialUSB.print("\x1b[2K");
	SerialUSB.print(config);
	SerialUSB.print("\t");
	SerialUSB.print(dmh);
	SerialUSB.print("\t");
	SerialUSB.println(direrror);
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


	for (i = 0; i < NOPINS; i++)
	{
		pinval[i] = analogRead(analogpins[i]);
		if (config)
		{
			if (pinval[i] > pinmax[i])
				pinmax[i] = pinval[i];
			if (pinval[i] < pinmin[i])
				pinmin[i] = pinval[i];
		}
		norm = (pinval[i] - pinmin[i]) * 100 / (pinmax[i] - pinmin[i]);
		if (norm < 0)
			norm = 0;
		if (norm > 100)
			norm = 100;
		pinnrm[i] = norm;

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

	direction = firsti;
	if (secondi == (firsti + 1))
	{
		direction += (float)second / (first * 2);

	}
	else if (secondi == (firsti - 1))
	{
		direction -= (float)second / (first * 2);
		// Drifting to the left, slow down right motor
	}
	direrror = direction - 2;

	lspeed = LFSPEED;
	rspeed = LFSPEED;
	if (direrror > 0)
	{
		rspeed = LFSPEED * (1.0 - direrror);
	}
	else if (direrror < 0)
	{
		lspeed = LFSPEED * (1.0 + direrror);
	}

	motors.L->setSpeed(lspeed);
	motors.R->setSpeed(rspeed);

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

	case 'C':
		// Config Mode
		configcmd(arg_cnt, args);
		break;
	}
}

void LineFollower::dmhcmd(int arg_cnt, char **args)
{
	int i;

	if (arg_cnt < 2)
		return;
	
	dmh = (args[1][0] == '1');
	
	if (!dmh)
	{
		motors.stop();
		motors.setEnableOutputs(false);
	}
	else
	{
		motors.setEnableOutputs(true);
	}
}

void LineFollower::configcmd(int arg_cnt, char **args)
{
	int i;

	if (arg_cnt < 2)
		return;

	config = args[1][0] == '1';

	if (config)
	{
		for (i = 0; i < NOPINS; i++)
		{
			pinmin[i] = DEFAULTMIN;
			pinmax[i] = DEFAULTMAX;
		}
		// Spin the robot 360.
	}
	else
	{
		// Stop any spin that is happening and get ready to run.
	}
}

LineFollower lineFollower;

