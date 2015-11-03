#include "MPU9250.h"
#include <SPI.h>

#define CSPIN 4
#define MPU9250_WHOAMI 0x75

void MPU9250Class::init()
{
	uint8_t readval;

	Serial3.println("Looking for MPU9250...");

	SPI.begin(CSPIN);
	SPI.setClockDivider(CSPIN, 84);
	readval = readByte(MPU9250_WHOAMI);

	if (readval == 0x71)
		Serial3.println("...MPU9250 found");
	else
		Serial3.println("...MPU9250 not found");
}

void MPU9250Class::writeByte(uint8_t reg, uint8_t data)
{
	SPI.transfer(CSPIN, reg, SPI_CONTINUE);
	data = SPI.transfer(CSPIN, data, SPI_LAST);
}

uint8_t MPU9250Class::readByte(uint8_t reg)
{
	uint8_t data;

	SPI.transfer(CSPIN, reg | 0x80, SPI_CONTINUE);
	data = SPI.transfer(CSPIN, 0, SPI_LAST);
	return data;
}

MPU9250Class MPU9250;

