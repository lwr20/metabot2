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
	void setdmh(bool setting);

private:
	bool m_running;
	int  m_accelindex;
	int m_topaccel;

};

extern SpeedTest speedTest;

#endif

