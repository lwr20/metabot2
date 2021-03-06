#include "ThreePointTurn.h"
#include "Motors.h"

#define MODESPEED	500    // 200
#define MODEDIR  	100     // 40
#define TURN90COUNT   1350

enum Actions {Forward, Backward, TurnLeft, TurnRight, Stop};

struct Action {
	Actions action;
	int     distance;
};

#define STEPS 8
Action sequence[STEPS]= {
	{ Forward, 15500 },
	{ TurnLeft, TURN90COUNT },
	{ Forward, 3000 },
	{ Backward, 7000 },
	{ Forward, 4000 },
	{ TurnLeft, TURN90COUNT },
	{ Forward, 14000 },
	{ Stop, 1000 },
};


void ThreePointTurn::start()
{
	SerialUSB.println("Start Three Point Turn Mode");

	motors.setCurrentPosition(0, 0);
	motors.setAcceleration(300, 100);
	m_step = 0;
	m_running = false;
	SerialUSB.print("Position : "); SerialUSB.print(motors.currentPositionL()); SerialUSB.print(", "); SerialUSB.println(motors.currentPositionR());
	startStep(m_step);
}

void ThreePointTurn::stop()
{
	SerialUSB.println("Stop Three Point Turn Mode");
}

void ThreePointTurn::loop()
{
	if (m_running && motors.atTargetPosition() && motors.isStopped())
	{
		SerialUSB.print("Position : "); SerialUSB.print(motors.currentPositionL()); SerialUSB.print(", "); SerialUSB.println(motors.currentPositionR());
		m_step++;
		if (m_step < STEPS)
			startStep(m_step);
	}
}

void ThreePointTurn::cmd(int arg_cnt, char **args)
{
}

void ThreePointTurn::setdmh(bool dmhset)
{
	if (dmhset)
	{
		m_running = true;
		motors.setEnableOutputs(true);
		motors.setSpeedDirection(m_currentSpeed, m_currentDirection);
		if (sequence[m_step].action == Stop)
		{
			// Reset to the beginning of the sequence
			m_step = 0;
			startStep(m_step);
		}
	}
	else
	{
		motors.stop();
		m_running = false;
	}
}

void ThreePointTurn::startStep(int step)
{
	switch (sequence[step].action)
	{
	case Forward:
		SerialUSB.print("Forward : ");
		m_currentSpeed = MODESPEED;
		m_currentDirection = 0;
		break;

	case Backward:
		SerialUSB.print("Backward : ");
		m_currentSpeed = -MODESPEED;
		m_currentDirection = 0;
		break;

	case TurnLeft:
		SerialUSB.print("Turn Left : ");
		m_currentSpeed = 0;
		m_currentDirection = -MODEDIR;
		break;

	case TurnRight:
		SerialUSB.print("Turn Right : ");
		m_currentSpeed = 0;
		m_currentDirection = MODEDIR;
		break;

	case Stop:
		SerialUSB.print("Stop : ");
		m_currentSpeed = 0;
		m_currentDirection = 0;
		motors.setEnableOutputs(false);
		break;
	}
	SerialUSB.println(sequence[step].distance);
	motors.move(sequence[step].distance);
	if (m_running)
		motors.setSpeedDirection(m_currentSpeed, m_currentDirection);

}

ThreePointTurn threePointTurn;

