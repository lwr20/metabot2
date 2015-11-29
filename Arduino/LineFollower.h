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

#define LFSPEED 150
#define INACTIVETHRESH 800
#define STEADYTIME 1000
#define REFLECTMARGIN 50
#define STEADYMARGIN 10
#define SPREADMARGIN 300
#define OFFLINE 400

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
	void setState(State);

	int pinmin[NOPINS];
	int pinmax[NOPINS];
	int pinval[NOPINS];
	int pinnrm[NOPINS];
	float direrror;
	int setspeed;
	int motorspeed;
	int motordir;

	char bar[BARLEN];

};

extern LineFollower lineFollower;

#endif

