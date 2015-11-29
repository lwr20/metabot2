// Proximity.h

#ifndef _PROXIMITY_h
#define _PROXIMITY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "modebase.h"
#include "Lights.h"

#define HIGHSPEED 200
#define SLOWSPEED 10
#define SLOWDISTANCE 30

#define STOPPINGDISTANCE 30   // number of clicks to continue after hitting the val70 point (which in theory is 70 clicks from the wall)

class Proximity : public ModeBase
{
public:
	void start();
	void stop();
	void loop();
	void cmd(int arg_cnt, char **args);
	void setdmh(bool setting);
		
private:
	void calcAverages(int position, int32_t* values);
	void updateSpeed(int32_t* values);
	float minDiff(float* values1, int32_t* values2);

	uint32_t m_currentSpeed;
	bool m_running;
	bool m_config;
	float lastAverage[3];
	int stoppingdistance;
	
	// Average values when 70 and 500 pulses away from the wall
	float val70[3];
	float val500[3];

};

extern Proximity proximity;

#endif

