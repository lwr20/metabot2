/*  InterruptStepper.cpp
*
*   This library sets up the PWM controller on the Arduino to drive the stepper motor controller.
*   It includes acceleration logic to control the rate of change of the speed to stop
*   the stepper motor losing sync.
*
*	Plan is to add logic to get the motor to run a certain distance (accounting for acceleration and
*   deceleration), but that hasn't been written yet.
*/

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
}

void InterruptStepper::setPinsInverted(bool directionInvert, bool enableInvert)
{
	_dirInverted = directionInvert;
	_enableInverted = enableInvert;
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

void InterruptStepper::setSpeed(float speed)
{
	float absSpeed;
	bool direction;

	absSpeed = abs(speed);
	direction = (speed > 0);

	// Set Direction
	if (direction == _dirInverted)
		digitalWrite(_dirPin, LOW);
	else
		digitalWrite(_dirPin, HIGH);

	// Calculate period of PWM output
	uint32_t period = MAXPERIODTICKS;

	if (absSpeed >= 1)
		period = CLKFREQ / absSpeed;

	if (period >= MAXPERIODTICKS)
		period = 0;					// use 0  length period to mean stop
	else if (period < MINPERIODTICKS)
		period = MINPERIODTICKS;   // Fastest we can go

	setPWMPeriod((uint16_t)period);
}

void InterruptStepper::setPWMPeriod(uint16_t period)
{
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

