// Skittles.h

#ifndef _SKITTLES_h
#define _SKITTLES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Servo.h>
#include "modebase.h"

class Skittles : public ModeBase
{
 public:
	 void start();
	 void stop();
	 void loop();
	 void cmd(int arg_cnt, char **args);

private:
	void servocmd(int arg_cnt, char **args);

	Servo servo;
};

extern Skittles skittles;

#endif

