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
#define BARLEN 110
#define NOPINS 5
#define DEFAULTMIN 300
#define DEFAULTMAX 500
#define CONFIGMAX  800
#define CONFIGMIN  600
#define ERRORMARGIN 10

class LineFollower : public ModeBase
{
  public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);

  private:
	bool dmh;
	void dmhcmd(int arg_cnt, char **args);
	void speedcmd(int arg_cnt, char **args);
	bool isInactive();
	void printbars();
	void readarray();

	enum State { inactive, config, active };

	State state;
	uint32_t pinmin[NOPINS];
	uint32_t pinmax[NOPINS];
	uint32_t pinval[NOPINS];
	uint32_t meanpinval;
	float pinnrm[NOPINS];
	float direrror;
	float speed;

	char bar[BARLEN];

};

extern LineFollower lineFollower;

#endif

