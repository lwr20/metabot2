// LineFollower.h

#ifndef _LINEFOLLOWER_h
#define _LINEFOLLOWER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

#define LFENABLEPIN  33 
#define LFSPEED 60.0


class LineFollower : public ModeBase
{
  public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);

  private:
	bool dmh;
	bool config;
	void configcmd(int arg_cnt, char **args);
	void dmhcmd(int arg_cnt, char **args);
	void speedcmd(int arg_cnt, char **args);
	void printbars();


};

extern LineFollower lineFollower;

#endif

