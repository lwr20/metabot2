// Joystick.h

#ifndef _JOYSTICK_h
#define _JOYSTICK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"
#include <Servo.h>

#define JOYACCELERATION			1000
#define JOYROTACCELERATION		1000
#define SERVOPIN 13
#define SERVOON 30
#define SERVOOFF 120

class Joystick : public ModeBase
{

  public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);
	void setdmh(bool setting);
	void setDirection(bool fwd);
	void setSensitivity(int sensitivity);

  private:
	void setForward(int arg_cnt, char **args);
	int servoon;
	int servooff;

	int sensitivity;   // scaling factor for joystick, 100 = 100%
	bool _normdir;
	Servo servo;

};

extern Joystick joystick;

#endif

