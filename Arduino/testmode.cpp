#include "testmode.h"
#include "MPU9250.h"

void Testmode::start()
{
	SerialUSB.println("Start Test Mode");
	// Turn on line follower light

}

void Testmode::stop()
{
	SerialUSB.println("Stop Test Mode");
}

void Testmode::loop()
{
	// Update position
	MPU9250.loop();
}

void Testmode::cmd(int arg_cnt, char **args)
{
}

Testmode testmode;

