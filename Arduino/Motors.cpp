/*  Motors.cpp
*
*   Simple class to control the left and right stepper motors.
*   Also includes an interrupt routine to count the pulses to each motor
*   which is used to calculate distance travelled.
*/

#include "Motors.h"

volatile uint32_t mLCount = 0;   // Count of pulses to left motor, Global variable as used in interrupt routine
volatile uint32_t mRCount = 0;   // Count of pulses to right motor
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

	// Reset stop targets (0 means ignore)
	_targetL = 0;
	_targetR = 0;

	setAcceleration(DFLTACCELERATION, DFLTROTACCELERATION);

	stop();

}

void Motors::loop()
{
	// Check if we have a target position configured
	if ((_targetL > 0) || (_targetR > 0))
	{
		uint32_t LCount = currentPositionL();
		uint32_t RCount = currentPositionR();
		// Check if we have reached our target position, if so, stop.
		if ((_targetL <= mLCount) ||  (_targetR <= mRCount))
		{
			stop();
		}
		else
		{
			// Check if we are within stopping distance of our target position
			// If so decelerate towards the STOPPINGSPEED.  Don't go to zero
			// in case that leaves us short of the target position.
			uint32_t stopdistance = stoppingDistance();
			if (((_targetL - LCount) < stopdistance) ||
				((_targetR - RCount) < stopdistance))
			{
				// Work out what the target speed and direction should be
				int stopSpeed = copysign(min(abs(_currentSpeed), STOPPINGSPEED), _currentSpeed);
				int stopDir = copysign(min(abs(_currentDirection), STOPPINGSPEED), _currentDirection);
				setSpeed(stopSpeed);
				setDirection(stopDir);
			}
		}
	}

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

float Motors::currentSpeed()
{
	return _currentSpeed;
}

float Motors::currentDirection()
{
	return _currentDirection;
}


void Motors::setAcceleration(float acceleration)
{
	setAcceleration(acceleration, _rotAcceleration);
}

void Motors::setAcceleration(float acceleration, float rotAcceleration)
{
	if (acceleration < 0 || rotAcceleration < 0)
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
	resetTargetPosition();

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
void Motors::setSpeedDirection(int speed, int direction)
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

void Motors::setSpeed(int speed)
{
	if (speed != _targetSpeed)
		setSpeedDirection(speed, _targetDirection);
}

void Motors::setDirection(int direction)
{
	if (direction != _targetDirection)
		setSpeedDirection(_targetSpeed, direction);
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

uint32_t Motors::stoppingDistance()
{
	int stopDistance = 0;

	if (abs(_currentSpeed) > STOPPINGSPEED)
		stopDistance += MICROSTEP * ((_currentSpeed * _currentSpeed) - (STOPPINGSPEED * STOPPINGSPEED))
							/ (2 * _acceleration);

	if (abs(_currentDirection) > STOPPINGSPEED)
		stopDistance += MICROSTEP * ((_currentDirection * _currentDirection) - (STOPPINGSPEED * STOPPINGSPEED))
							/ (2 * _rotAcceleration);

	return stopDistance;
}

void Motors::moveTo(int lstop, int rstop)
{
	_targetL = lstop;
	_targetR = rstop;
}

void Motors::move(int lstop, int rstop)
{
	_targetL = mLCount + lstop;
	_targetR = mRCount + rstop;
}

void Motors::move(int lrstop)
{
	move(lrstop, lrstop);
}

uint32_t Motors::currentPositionL()
{
	return mLCount;
}

uint32_t Motors::currentPositionR()
{
	return mRCount;
}

void Motors::setCurrentPosition(uint32_t positionL, uint32_t positionR)
{
	mLCount = positionL;
	mRCount = positionR;
}

void Motors::resetTargetPosition()
{
	_targetL = 0;
	_targetR = 0;
}

bool Motors::atTargetPosition()
{
	return ((_targetL == 0) && (_targetR == 0));
}

bool Motors::isStopped()
{
	return (_currentDirection == 0.0) && (_currentSpeed == 0.0);
}



Motors motors;

