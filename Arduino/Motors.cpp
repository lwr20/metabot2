#include "Motors.h"

volatile unsigned long mLCount = 0;   // Count of pulses to left motor
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
	{
		mLCount++;
	}

	else if ((events & mRChMask) == mRChMask)
	{
		mRCount++;
	}
}


void Motors::init()
{
	L = new InterruptStepper(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
	R = new InterruptStepper(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);

	// Initialise stepper motors
	L->setMaxSpeed(MAX_SPEED);
	L->setAcceleration(ACCELERATION);
	L->setPinsInverted(true, false);
	L->setMicrostep(L_MICROSTEP);

	R->setMaxSpeed(MAX_SPEED);
	R->setAcceleration(ACCELERATION);
	R->setPinsInverted(true, false);
	R->setMicrostep(R_MICROSTEP);

	// Set these just to be sure...
	L->setSpeed(0);
	R->setSpeed(0);
	L->setEnableOutputs(false);
	R->setEnableOutputs(false);
}

void Motors::run()
{
	L->run();
	R->run();
}

void Motors::setAcceleration(float acc)
{
	L->setAcceleration(acc);
	R->setAcceleration(acc);
}

void Motors::stop()
{
	L->stop();
	R->stop();
}

void Motors::setEnableOutputs(bool enable)
{
	L->setEnableOutputs(enable);
	R->setEnableOutputs(enable);
}



Motors motors;

