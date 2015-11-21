/*  Motors.cpp
*
*   Simple class to control the left and right stepper motors.
*   Also includes an interrupt routine to count the pulses to each motor
*   which is used to calculate distance travelled.
*/

#include "Motors.h"

volatile unsigned long mLCount = 0;   // Count of pulses to left motor, Global variable as used in interrupt routine
volatile unsigned long mRCount = 0;   // Count of pulses to right motor
uint32_t mLChannel = g_APinDescription[X_STEP_PIN].ulPWMChannel;    // Left motor PWM channel
uint32_t mLChMask = 0x01 << mLChannel;
uint32_t mRChannel = g_APinDescription[Y_STEP_PIN].ulPWMChannel;    // Right motor PWM channel
uint32_t mRChMask = 0x01 << mRChannel;

// PWM Interrupt handler
void PWM_Handler()
{
	// Look at what events are being signalled
	uint32_t events = PWM->PWM_ISR1;

	if ((events & mLChMask) == mLChMask)
		mLCount++;
	else if ((events & mRChMask) == mRChMask)
		mRCount++;
}

void Motors::init()
{
	L = new InterruptStepper(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
	R = new InterruptStepper(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);

	// Initialise stepper motors
	L->setMicrostep(MICROSTEP);
	R->setMicrostep(MICROSTEP);

	L->setPinsInverted(false, false);
	R->setPinsInverted(false, false);

	L->setEnableOutputs(false);
	R->setEnableOutputs(false);

	setAcceleration(ACCELERATION, ROTACCELERATION);

	stop();

}

void Motors::loop()
{
	if (_stopping)
	{
		if (_stopMillis + STOPTIME < millis())
		{
			_stopping = false;
			setEnableOutputs(false);
		}
	}
	else
		computeNewSpeed();

}

void Motors::setEnableOutputs(bool enable)
{
	L->setEnableOutputs(enable);
	R->setEnableOutputs(enable);
}

void Motors::setAcceleration(float acceleration, float rotAcceleration)
{
	if (acceleration < 0)
		return;

	_acceleration = acceleration;
	_rotAcceleration = rotAcceleration;

	// Reset speed calculation
	_startSpeed = _currentSpeed;
	_startDirection = _currentDirection;
	_startMillis = millis();
}

void Motors::stop()
{
	_currentSpeed = 0.0;
	_targetSpeed = 0.0;
	_currentDirection = 0.0;
	_targetDirection = 0.0;
	_stopping = true;
	_stopMillis = millis();

	L->setSpeed(0.0);
	R->setSpeed(0.0);
}

/* SetSpeed has two parameters, speed and direction
  The final speed of the left motor is speed + direction and the 
  final speed of the right motor is speed - direction.
  
  When changing speeds, acceleration is taken into account and controls
  the maximum rate of change of speed on either motor.  If the speed of the motors
  is changing by different amounts, e.g. when turning and decelerating
  at the same time, then the larger change is constrained by the
  acceleration and the smaller change changes proportionally.
  */
void Motors::setSpeed(int speed, int direction)
{
	// Reset speed calculation
	_startSpeed = _currentSpeed;
	_startDirection = _currentDirection;
	_targetSpeed = speed;
	_targetDirection = direction;
	_startMillis = millis();
	_stopping = false;

	computeNewSpeed();
}

float Motors::accelerate(float start, float current, float target, float acceleration)
{
	float retspeed = target;

	if (acceleration > 0)
	{
		// Calculate new speed and direction based on acceleration
		if (current < target)
		{
			// Accelerating
			retspeed = start + (acceleration * (millis() - _startMillis) / 1000);
			if (retspeed > target)
				retspeed = target;
		}
		else if (current > target)
		{
			// Decelerating
			retspeed = start - (acceleration * (millis() - _startMillis) / 1000);
			if (retspeed < target)
				retspeed = target;
		}
	}

	return retspeed;
}


void Motors::computeNewSpeed()
{
	_currentSpeed = accelerate(_startSpeed, _currentSpeed, _targetSpeed, _acceleration);
	_currentDirection = accelerate(_startDirection, _currentDirection, _targetDirection, _rotAcceleration);

	L->setSpeed(_currentSpeed + _currentDirection);
	R->setSpeed(_currentSpeed - _currentDirection);
}

Motors motors;

