// SpeedTest.h

#ifndef _SPEEDTEST_h
#define _SPEEDTEST_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"

class SpeedTest : public ModeBase
{
public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);

private:
	uint32_t	m_currentSpeed;
	uint32_t	m_currentDirection;
	bool m_running;

};

extern SpeedTest speedTest;

#endif

