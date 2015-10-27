// Motors.h

#ifndef _MOTORS_h
#define _MOTORS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "InterruptStepper.h"

// Pin definitions from Arduino DUE variant.cpp file here:
// C:\Users\<user>\AppData\Roaming\Arduino15\packages\arduino\hardware\sam\1.6.4\variants\arduino_due_x
#define X_STEP_PIN         9   // PC21
#define X_DIR_PIN          10  // PC29
#define X_ENABLE_PIN       8   // PC22

#define Y_STEP_PIN         6   // PC24
#define Y_DIR_PIN          7   // PC23
#define Y_ENABLE_PIN       5   // PC26 (and PA29)

// Other constants
#define L_MICROSTEP        16
#define R_MICROSTEP        16
#define MAX_SPEED          6000
#define ACCELERATION       300

class Motors
{
 protected:


 public:
	void init();
	void run();
	void setAcceleration(float acc);
	void stop();
	void setEnableOutputs(bool);


	InterruptStepper* L;
	InterruptStepper* R;

};

extern Motors motors;
extern volatile unsigned long mLCount; 
extern volatile unsigned long mRCount;  


#endif

