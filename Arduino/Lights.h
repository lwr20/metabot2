// Lights.h

#ifndef _LIGHTS_h
#define _LIGHTS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define LFENABLEPIN  33 
#define NOPINS 5
#define BARLEN 110


class Lights
{
 
public:
	void init();
	void getvals();
	void setEnabled(bool);
	void bartrace();

	int32_t lightval[NOPINS];

private:
	bool m_enabled;
	char bar[BARLEN];

};

extern Lights lights;

#endif

