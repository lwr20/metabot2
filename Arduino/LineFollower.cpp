#include "LineFollower.h"
#include "CmdUSB.h"
#include "Motors.h"

#define NOPINS 5
//int analogpins[NOPINS] = { 0, 1, 2, 3, 4 };    // pin numbers of the line follower analog inputs, in order
int analogpins[NOPINS] = { 4, 3, 2, 1, 0 };    // pin numbers of the line follower analog inputs, in order

#define DEFAULTMIN 300
#define DEFAULTMAX 500
#define CONFIGMAX  800
#define CONFIGMIN  600

int pinmin[NOPINS];
int pinmax[NOPINS];
int pinval[NOPINS];
int meanpinval;
float pinnrm[NOPINS];
float direrror;
float speed = LFSPEED;

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

	// Set instant acceleration
	motors.setAcceleration(0.0);

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
		int endbar = round(pinnrm[i] * 80);
		bar[endbar] = 0;
		SerialUSB.print("\x1b[2K");  // Delete Line
		SerialUSB.print(pinnrm[i]);
		SerialUSB.print("\t");
		SerialUSB.print(pinmin[i]);
		SerialUSB.print("\t");
		SerialUSB.print(pinmax[i]);
		SerialUSB.print("\t");
		SerialUSB.print(bar);
		SerialUSB.println();
		bar[endbar] = '|';
	}
	SerialUSB.println("\x1b[2K");
	SerialUSB.print("\x1b[2K");
	SerialUSB.print(config);
	SerialUSB.print("\t");
	SerialUSB.print(dmh);
	SerialUSB.print("\t");
	SerialUSB.print(meanpinval);
	SerialUSB.print("\t");
	SerialUSB.println(direrror);
  SerialUSB.print("\x1b[14F");  // Scroll back 14 lines

}

void LineFollower::loop()
{
	int i;
	float norm;
	float direction;
	float lspeed;
	float rspeed;

	float first = 0;
	int firsti = 0;
	float second = 0;
	int secondi = 0;
	meanpinval = 0;

	for (i = 0; i < NOPINS; i++)
	{
		pinval[i] = analogRead(analogpins[i]);
		meanpinval += pinval[i];
		if (config)
		{
			if (pinval[i] > pinmax[i])
				pinmax[i] = pinval[i];
			if (pinval[i] < pinmin[i])
				pinmin[i] = pinval[i];
		}
		norm = (float)(pinval[i] - pinmin[i]) / (pinmax[i] - pinmin[i]);
		if (norm < 0)
			norm = 0;
		if (norm > 1)
			norm = 1;
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

	meanpinval /= 5;
	// Use Meanvalue to decide whether to go into config mode or not
	// Greater than DEFAULTMAX we should reset (been lifted off board)
	// Less than DEFAULTMIN and we should go into config mode
	if (config && meanpinval > CONFIGMAX)
	{
		for (i = 0; i < NOPINS; i++)
		{
			pinmin[i] = DEFAULTMIN;
			pinmax[i] = DEFAULTMAX;
		}
		config = false;
	}
	else if (!config && meanpinval < CONFIGMIN)
	{
		config = true;
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
		// Drifting to the left, slow down right motor
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

	case 'P':
		// Set Speed
		speedcmd(arg_cnt, args);
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
    config = false;
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
	}
}

void LineFollower::speedcmd(int arg_cnt, char **args)
{
	if (arg_cnt < 2)
		return;

	speed = float(cmdStr2Num(args[1], 10));
}


LineFollower lineFollower;

