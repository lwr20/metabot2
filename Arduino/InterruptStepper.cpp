// InterruptStepper.cpp

#include "interruptStepper.h"

InterruptStepper::InterruptStepper(uint32_t pwmPin, uint32_t dirPin, uint32_t enablePin)
{
	_pwmPin = pwmPin;
	_dirPin = dirPin;
	_enablePin = enablePin;

	pinMode(_pwmPin, OUTPUT);
	pinMode(_dirPin, OUTPUT);
	pinMode(_enablePin, OUTPUT);

	//Setup PWM
	pmc_enable_periph_clk(PWM_INTERFACE_ID);
	_chan = g_APinDescription[_pwmPin].ulPWMChannel;
	PIO_Configure(g_APinDescription[_pwmPin].pPort,
		g_APinDescription[_pwmPin].ulPinType,
		g_APinDescription[_pwmPin].ulPin,
		g_APinDescription[_pwmPin].ulPinConfiguration);
	PWMC_ConfigureChannel(PWM_INTERFACE, _chan, PWM_CMR_CPRE_MCK_DIV_1024, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, _chan, PWM_MAX_DUTY_CYCLE);
	PWMC_SetDutyCycle(PWM_INTERFACE, _chan, PWM_MAX_DUTY_CYCLE / 2);
	PWMC_EnableChannelIt(PWM_INTERFACE, _chan);
	NVIC_EnableIRQ(PWM_IRQn);
	PWMC_EnableChannel(PWM_INTERFACE, _chan);

	// Initialise variables and set speed
	_maxSpeed = ABSMAXSPEED;
	_acceleration = ABSMAXSPEED;
	_dirInverted = false;
	_enableInverted = false;
	_startSpeed = 0.0;
	_startMillis = millis();
	_stopping = false;
	_stopMillis = 0;

	stop();
	setEnableOutputs(false);
}

void InterruptStepper::stop()
{
	_currentPos = 0;
	_targetPos = 0;
	_currentSpeed = 0.0;
	_targetSpeed = 0.0;
	_stopping = true;
	_stopMillis = millis();

	// Disable the channel
	PWMC_DisableChannel(PWM_INTERFACE, _chan);
}

void InterruptStepper::run()
{
	if (_stopping)
	{
		if (_stopMillis + STOPTIME < millis())
		{
			_stopping = false;
			setEnableOutputs(false);
		}
	}
	else if (_acceleration > 0)
		computeNewSpeed();
}

void InterruptStepper::setMaxSpeed(float speed)
{
	if (abs(speed) < ABSMAXSPEED)
		_maxSpeed = abs(speed);
	else
		_maxSpeed = ABSMAXSPEED;
}

void InterruptStepper::setAcceleration(float acceleration)
{
	if (acceleration < 0)
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
	_stopping = false;

	computeNewSpeed();
}

void InterruptStepper::setEnableOutputs(bool enabled)
{
	if (enabled == _enableInverted)
		digitalWrite(_enablePin, LOW);
	else
		digitalWrite(_enablePin, HIGH);
}

// Note that this disables the channel
void InterruptStepper::setMicrostep(int microstep)
{
	if (microstep <= 0 || microstep > 128)
		return;

	uint32_t clock = PWM_CMR_CPRE_MCK_DIV_1024;

	while (microstep > 1)
	{
		microstep = microstep / 2;
		clock = clock - 1;
	}

	PWMC_ConfigureChannel(PWM_INTERFACE, _chan, clock, 0, 0);
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


	if (_acceleration == 0)
	{
		// Change speed instantly
		_currentSpeed = _targetSpeed;
	}
	else
	{
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
	}

	newSpeed = abs(_currentSpeed);
	newDirection = (_currentSpeed > 0);

	// Set Direction
	if (newDirection == _dirInverted)
		digitalWrite(_dirPin, LOW);
	else
		digitalWrite(_dirPin, HIGH);

	// Set Speed
	uint16_t period = 0;   // default is zero speed

	if (newSpeed > ABSMINSPEED)
		period = uint32_t(CLKFREQ / newSpeed);

	setNewSpeed(period);
}

void InterruptStepper::setNewSpeed(uint16_t period)
{
	//Serial.println(period);
	if (period == 0)
	{
		// stopped, turn off the channel
		PWMC_DisableChannel(PWM_INTERFACE, _chan);
	}
	else
	{
		// make sure the channel is enabled
		PWMC_EnableChannel(PWM_INTERFACE, _chan);
		// set period
		PWM->PWM_CH_NUM[_chan].PWM_CPRDUPD = period;
		PWM->PWM_CH_NUM[_chan].PWM_CDTYUPD = period / 2;
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
