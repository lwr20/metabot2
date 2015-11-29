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
#define MICROSTEP			16
#define STOPTIME			1000      // Number of milliseconds to hold motor when stopping.  After this time the motor is disabled (and so can freewheel);
#define STOPPINGSPEED		10      // Target speed when coming to an automatice stop
#define DFLTACCELERATION	1000
#define DFLTROTACCELERATION	1000
class Motors
{
  public:
	void init();
	void loop();
	void setAcceleration(float acceleration, float rotAcceleration);
	void setAcceleration(float acceleration);
	void setEnableOutputs(bool);
	void setSpeedDirection(int,int);
	void setSpeed(int);
	void setDirection(int);
	void stop();
	void moveTo(int, int);
	void move(int, int);
	void move(int);
	uint32_t currentPositionL();
	uint32_t currentPositionR();
	void setCurrentPosition(uint32_t positionL, uint32_t positionR);
	void resetTargetPosition();
	bool isStopped();
	bool atTargetPosition();
	float currentSpeed();
	float currentDirection();

	/*
	boolean runSpeed();
	float   speed();
	long    distanceToGo();
	long    targetPosition();
	long    currentPosition();
	void    setCurrentPosition(long position);
	void    runToPosition();
	boolean runSpeedToPosition();
	void    runToNewPosition(long position);
	*/

private:
	void        computeNewSpeed();
	void		setNewSpeed(uint16_t period);
	float       accelerate(float start, float current, float target, float acceleration);
	uint32_t	stoppingDistance();

	InterruptStepper* L;
	InterruptStepper* R;

	uint32_t	_targetL;			// PWM Pulses (interrupt count)
	uint32_t	_targetR;			// PWM Pulses (interrupt count)
	float       _currentSpeed;		// Steps per second
	float       _targetSpeed;		// Steps per second
	float       _currentDirection;  // Steps per second
	float       _targetDirection;   // Steps per second
	float       _acceleration;      // Steps per second per second
	float		_rotAcceleration;   // Steps per second per second
	bool		_stopping;
	uint32_t    _stopMillis;

	// Variables for acceleration calculation
	float		_startSpeed;
	float		_startDirection;
	uint32_t    _startMillis;

};

extern Motors motors;

#endif

