// MPU9250.h

#ifndef _MPU9250_h
#define _MPU9250_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class MPU9250Class
{
 protected:


 public:
	void init();

 private:
	void writeByte(uint8_t reg, uint8_t data);
	uint8_t readByte(uint8_t reg);

};

extern MPU9250Class MPU9250;

#endif

