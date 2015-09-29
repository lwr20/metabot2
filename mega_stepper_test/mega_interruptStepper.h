// InterruptStepper.h

#ifndef InterruptStepper_h
#define AccelStepper_h
#include <stdlib.h>

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

#define CLKFREQ 16000000.0

class InterruptStepper
{

public:
    InterruptStepper(uint8_t timer, uint8_t dirPin, uint8_t enablePin);
    boolean run();
    void    setMaxSpeed(float speed);
    void    setAcceleration(float acceleration);
    void    setSpeed(float speed);
    void    setEnableOutputs(bool enabled);
	void	setMicrostep(int microstep);
    void    setPinsInverted(bool directionInvert, bool enableInvert);
    void    stop();
	
/*	void    moveTo(long absolute); 
    void    move(long relative);
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
    void           computeNewSpeed();
	void		   setTimer(uint16_t count, uint8_t cs, boolean direction);

	uint8_t        _timer;         // 3 or 4
    long           _currentPos;    // Steps
    long           _targetPos;     // Steps
	float          _currentSpeed;  // Steps per second
    float          _targetSpeed;   // Steps per second
    float          _maxSpeed;      // Steps per second
	float		   _absMaxSpeed;   // Steps per second
	float          _absMinSpeed;   // Steps per second
    float          _acceleration;  // Steps per second per second 
	int		       _microstep;
	uint8_t        _dirPin;
    uint8_t        _enablePin;
    bool           _dirInverted;
    bool           _enableInverted;
    boolean        _targetDirection;
	
	// Variables for acceleration calculation
	float		   _startSpeed;
	unsigned long  _startMillis;
};

#endif 
