// modebase.h

#ifndef _MODEBASE_h
#define _MODEBASE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class ModeBase
{
 public:
	virtual void start();
	virtual void stop();
	virtual void loop();
	virtual void cmd(int arg_cnt, char **args);
	virtual void setdmh(bool);
};

#endif

