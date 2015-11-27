// ThreePointTurn.h

#ifndef _THREEPOINTTURN_h
#define _THREEPOINTTURN_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

class ThreePointTurn : public ModeBase
{

public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);
	void setdmh(bool dmhset);

private:
	void startStep(int);

	int m_step;
	uint32_t	m_currentSpeed;
	uint32_t	m_currentDirection;
	bool m_running;

};

extern ThreePointTurn threePointTurn;

#endif

