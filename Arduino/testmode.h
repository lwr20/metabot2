// testmode.h

#ifndef _TESTMODE_h
#define _TESTMODE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

class Testmode : public ModeBase
{
 public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);

private:
	bool running;
};

extern Testmode testmode;

#endif

