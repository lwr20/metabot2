// LineFollower.h

#ifndef _LINEFOLLOWER_h
#define _LINEFOLLOWER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"
#include "Lights.h"

#define LFSPEED 60.0
#define INACTIVETHRESH 800
#define STEADYTIME 1000
#define ERRORMARGIN 5
#define SPREADMARGIN 300

class LineFollower : public ModeBase
{
  public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);
	void setdmh(bool dmhset);

  private:
	bool dmh;				//Dead Man's Handle
	void speedcmd(int arg_cnt, char **args);
	bool steadyReadings();
	bool reflection();
	void printbars();

	enum State { inactive, config, active };
	State state;
	uint32_t pinmin[NOPINS];
	uint32_t pinmax[NOPINS];
	uint32_t pinval[NOPINS];
	float pinnrm[NOPINS];
	float direrror;
	float speed;

	char bar[BARLEN];

};

extern LineFollower lineFollower;

#endif

