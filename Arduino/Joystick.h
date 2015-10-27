// Joystick.h

#ifndef _JOYSTICK_h
#define _JOYSTICK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

class Joystick : public ModeBase
{

  public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);

  private:
	void setForward(int arg_cnt, char **args);

};

extern Joystick joystick;

#endif

