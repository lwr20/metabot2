// Joystick.h

#ifndef _JOYSTICK_h
#define _JOYSTICK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

#define JOYACCELERATION			1000
#define JOYROTACCELERATION		1000


class Joystick : public ModeBase
{

  public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);
	void setDirection(bool fwd);

  private:
	void setForward(int arg_cnt, char **args);

	bool _normdir;

};

extern Joystick joystick;

#endif

