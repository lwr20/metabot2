// Config.h

#ifndef _CONFIG_h
#define _CONFIG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

class Config : public ModeBase
{
public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);
	void setdmh(bool);

private:
	enum  State { stopped, paused, running } state;
	int16_t xmin;
	int16_t xmax;
	int16_t ymin;
	int16_t ymax;
	int16_t zmin;
	int16_t zmax;
	int16_t xoffset;
	int16_t yoffset;
	int16_t zoffset;

	int		rotspeed;

};

extern Config config;

#endif

