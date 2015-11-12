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

#define CLKDIV 1024         // Starting point for Clock Divider. Note that microstepping will change what is actually programmed
#define CLKFREQ (VARIANT_MCK / CLKDIV)
#define ABSMAXSPEED 5000.0
#define ABSMINSPEED 1.282  // This is the slowest speed we can run without overflowing the PWM timer at the clock speed we are using.
#define STOPTIME 1000      // Number of milliseconds to hold motor when stopping.  After this time the motor is disabled (and so can freewheel);

class InterruptStepper
{

  public:
    InterruptStepper(uint32_t pwmPin, uint32_t dirPin, uint32_t enablePin);
    void run();
    void setMaxSpeed(float speed);
    void setAcceleration(float acceleration);
    void setSpeed(float speed);
    void setEnableOutputs(bool enabled);
    void setMicrostep(int microstep);
    void setPinsInverted(bool directionInvert, bool enableInvert);
    void stop();

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
    void		       setNewSpeed(uint16_t period);

    long           _currentPos;    // Steps
    long           _targetPos;     // Steps
    float          _currentSpeed;  // Steps per second
    float          _targetSpeed;   // Steps per second
    float          _maxSpeed;      // Steps per second
    float          _acceleration;  // Steps per second per second
    uint32_t       _pwmPin;
    uint32_t       _dirPin;
    uint32_t       _enablePin;
    uint32_t       _chan;
    bool           _dirInverted;
    bool           _enableInverted;
    bool           _targetDirection;
	bool		   _stopping;
	unsigned long  _stopMillis;

    // Variables for acceleration calculation
    float		   _startSpeed;
    unsigned long  _startMillis;
};

#endif
