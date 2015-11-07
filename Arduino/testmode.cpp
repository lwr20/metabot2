#include "testmode.h"

#define LFENABLEPIN  33 
#define SAMPLES 15

int LEDvals[SAMPLES];
uint32_t microvals[SAMPLES];

void Testmode::start()
{
	SerialUSB.println("Start Test Mode");
	// Turn on line follower light
	pinMode(LFENABLEPIN, OUTPUT);
	digitalWrite(LFENABLEPIN, 1);

}

void Testmode::stop()
{
	SerialUSB.println("Stop Test Mode");
	// Turn off line follower light
	digitalWrite(LFENABLEPIN, 0);
}

void Testmode::loop()
{
	int i;
	uint32_t start = micros();

	for (i = 0; i < SAMPLES; i++)
	{
		microvals[i] = micros() - start;
		LEDvals[i] = analogRead(0);
		if (i == 0)
			digitalWrite(LFENABLEPIN, 0);
		delayMicroseconds(10);
	}
	digitalWrite(LFENABLEPIN, 1);

	for (i = 0; i < SAMPLES; i++)
	{
		SerialUSB.print(microvals[i]);
		SerialUSB.print(": ");
		SerialUSB.print(LEDvals[i]);
		SerialUSB.print("  ");
	}
	SerialUSB.println();
	delay(500);
}

void Testmode::cmd(int arg_cnt, char **args)
{
}

Testmode testmode;

