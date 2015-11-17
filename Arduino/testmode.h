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
	enum  State { stopped, paused, running } state;
	int16_t xmin;
	int16_t xmax;
	int16_t ymin;
	int16_t ymax;
	int16_t xoffset;
	int16_t yoffset;

};

extern Testmode testmode;

#endif

