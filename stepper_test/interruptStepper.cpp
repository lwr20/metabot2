// InterruptStepper.cpp

#include "interruptStepper.h"

InterruptStepper::InterruptStepper(uint8_t timer, uint8_t dirPin, uint8_t enablePin)
{
	_timer = timer;
	_dirPin = dirPin;
	_enablePin = enablePin;

	pinMode(_dirPin, OUTPUT);
	pinMode(_enablePin, OUTPUT);
	
	if (_timer==3)
	{
		TCCR3A = 0x03;   // Set Mode 15 Fast pwm
		TCCR3B = 0x19;   // No prescaling on clock source
		TIMSK3 = 0x02;   // Enable interrupt for Channel A Output Compare
	} 
	else if (_timer==4)
	{
		TCCR4A = 0x03;   // Set Mode 15 Fast pwm
		TCCR4B = 0x19;   // No prescaling on clock source
		TIMSK4 = 0x02;   // Enable interrupt for Channel A Output Compare
	}
	else
		return;

	// Initialise variables and set speed
	_absMaxSpeed = CLKFREQ / 200;
	_absMinSpeed = 10;              // = 20Hz, which is fast enough to be responsive to new speeds, as speed only changes at end of a cycle.
    _maxSpeed = _absMaxSpeed;
    _acceleration = _absMaxSpeed;
	_dirInverted = false;
    _enableInverted = false;
	_microstep = 1;
	_startSpeed = 0.0;
	_startMillis = millis();
	stop();
	setEnableOutputs(false);
}

void InterruptStepper::stop()
{
    _currentPos = 0;
    _targetPos = 0;
	_currentSpeed = 0.0;
    _targetSpeed = 0.0;
	
	// Disable interrupt for Channel A Output Compare
	if (_timer==3)
		TIMSK3 = 0x00;
	else if (_timer==4)
		TIMSK4 = 0x00;
}

boolean InterruptStepper::run()
{
	computeNewSpeed();
}

void InterruptStepper::setMaxSpeed(float speed)
{
	if ((abs(speed) * _microstep) < _absMaxSpeed)
		_maxSpeed = abs(speed);
	else
		_maxSpeed = _absMaxSpeed / _microstep;
}

void InterruptStepper::setAcceleration(float acceleration)
{
	if (acceleration <= 0)
		return;
	
	_acceleration = acceleration;
	
	// Reset speed calculation
	_startSpeed = _currentSpeed;
	_startMillis = millis();
}

void InterruptStepper::setSpeed(float speed)
{
	if (abs(speed) < _maxSpeed)
		_targetSpeed = speed;
	else if (speed > 0)
		_targetSpeed = _maxSpeed;
	else
		_targetSpeed = -_maxSpeed;
	
	// Reset speed calculation
	_startSpeed = _currentSpeed;
	_startMillis = millis();
	
	computeNewSpeed();
}

void InterruptStepper::setEnableOutputs(bool enabled)
{
	if (enabled == _enableInverted)
		digitalWrite(_enablePin,LOW);
	else
		digitalWrite(_enablePin,HIGH);
}

void InterruptStepper::setMicrostep(int microstep)
{
	if (microstep <= 0)
		return;
	
	_microstep = microstep;
	
	if ((_maxSpeed * _microstep) >= _absMaxSpeed)
		_maxSpeed = _absMaxSpeed / _microstep;
	
	setSpeed(_targetSpeed);
}

void InterruptStepper::setPinsInverted(bool directionInvert, bool enableInvert)
{
	_dirInverted = directionInvert;
	_enableInverted = enableInvert;
}

void InterruptStepper::computeNewSpeed()
{
	float newSpeed;
	float newDirection;
	
	// Calculate new speed and direction based on acceleration
	if (_currentSpeed < _targetSpeed)
	{
		// Accelerating
		_currentSpeed = _startSpeed + (_acceleration * (millis() - _startMillis) / 1000);
		if (_currentSpeed > _targetSpeed)
			_currentSpeed = _targetSpeed;
	}
	else if (_currentSpeed > _targetSpeed)
	{
		// Decelerating
		_currentSpeed = _startSpeed - (_acceleration * (millis() - _startMillis) / 1000);
		if (_currentSpeed < _targetSpeed)
			_currentSpeed = _targetSpeed;
	}
	
	newSpeed = abs(_currentSpeed);
	newDirection = (_currentSpeed > 0);
	
	//Serial.print("New Speed = ");
	//Serial.println(newSpeed);
	
	// Calculate timer parameters
	long count = 0;   // default is zero speed
	uint8_t clockSource = 1;

	if ((newSpeed * _microstep) > _absMinSpeed)
	{
		count = long(CLKFREQ / (newSpeed * 2 * _microstep));
		//Serial.print("Count Start = ");
		//Serial.println(count, HEX);

		while (count > 0xFFFF)
		{
			count = count >> 3;  // this only works for clockSource -> 2,3 as they are x8 apart.  ->4,5 are only x4 apart
			clockSource++;
		}
		if (clockSource > 3)
		{
			Serial.print("Error, overflow");
			stop();
			return;
		}
	}

	if (newDirection == _dirInverted)
		digitalWrite(_dirPin,LOW);
	else
		digitalWrite(_dirPin,HIGH);
	
	//Serial.print("Count End = ");
	//Serial.println(count, HEX);
	//Serial.print("CS = ");
	//Serial.println(clockSource, HEX);
	
	setTimer((uint16_t)count, clockSource, newDirection);
}

void InterruptStepper::setTimer(uint16_t count, uint8_t clockSource, boolean direction)
{
	// clockSource can take the following values
	//      1 :  Clk / 1 (no prescaling)
	//      2 :  Clk / 8 (from prescaler)
	//      3 :  Clk / 64 (from prescaler)
	//      4 :  Clk / 256 (from prescaler)
	//      5 :  Clk / 1024 (from prescaler)
	
	if (_timer == 3)
	{
		if (count == 0)
		{
			// stopped, turn off timer interrupts
			TIMSK3 = 0x00; 
		}
		else
		{
			// make sure timer interrupts are on
			TIMSK3 = 0x02;
			// set prescaler
			TCCR3B = (TCCR3B & 0xFC) | (clockSource & 0x03); 
			// set count
			OCR3A = count;
		}
	}		
	else if (_timer == 4)
	{
		if (count == 0)
		{
			// stopped, turn off timer interrupts
			TIMSK4 = 0x00; 
		}
		else
		{
			// make sure timer interrupts are on
			TIMSK4 = 0x02;
			// set prescaler
			TCCR4B = (TCCR4B & 0xFC) | (clockSource & 0x03); 
			// set count
			OCR4A = count;
		}
	}		
}

/*
void InterruptStepper::moveTo(long absolute)
{
}

void InterruptStepper::move(long relative)
{
}

boolean InterruptStepper::runSpeed()
{
}

float InterruptStepper::speed()
{
}

long InterruptStepper::distanceToGo()
{
}

long InterruptStepper::targetPosition()
{
}

long InterruptStepper::currentPosition()
{
}

void InterruptStepper::setCurrentPosition(long position)
{
}

void InterruptStepper::runToPosition()
{
}

boolean InterruptStepper::runSpeedToPosition()
{
}

void InterruptStepper::runToNewPosition(long position)
{
}

*/
