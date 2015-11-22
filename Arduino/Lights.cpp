#include "Lights.h"

int analogpins[NOPINS] = { 4, 3, 2, 1, 0 };    // pin numbers of the line follower analog inputs, in order

void Lights::init()
{
	pinMode(LFENABLEPIN, OUTPUT);
	setEnabled(false);

	// Initialise trace variables
	memset(bar, '|', BARLEN);
}

void Lights::setEnabled(bool enabled)
{
	m_enabled = enabled;
	if (m_enabled)
		digitalWrite(LFENABLEPIN, 1);
	else
		digitalWrite(LFENABLEPIN, 0);
}

void Lights::getvals()
{
	int i;

	setEnabled(true);
	delayMicroseconds(500);            // Need to wait a bit for the LED to turn on

	for (i = 0; i < NOPINS; i++)
	{
		lightval[i] = analogRead(analogpins[i]);
	}

	setEnabled(false);
	delayMicroseconds(500);            // Need to wait a bit for the LED to turn off

	for (i = 0; i < NOPINS; i++)
	{
		//  We've turned off the LED, so values should go up
		lightval[i] = analogRead(analogpins[i]) - lightval[i];
		if (lightval[i] < 0)
			lightval[i] = 0;
	}
	setEnabled(true);
}

void Lights::bartrace()
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

	SerialUSB.println("\x1b[2K");
	// Draw out the actual readings
	for (i = 0; i < NOPINS; i++)
	{
		bar[lightval[i] / 12] = 0;
		SerialUSB.print("\x1b[2K");  // Delete Line
		SerialUSB.print(lightval[i]);
		SerialUSB.print("\t");
		SerialUSB.print(bar);
		SerialUSB.println();
		bar[lightval[i] / 12] = '|';
	}
	SerialUSB.print("\x1b[6F");  // Scroll back 6 lines
}

Lights lights;

