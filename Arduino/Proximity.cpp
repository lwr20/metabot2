#include "Proximity.h"
#include "Motors.h"
#include "CmdUSB.h"


Proximity::Proximity()
{
	stoppingdistance = STOPPINGDISTANCE;

	// Assign some default thresholds in case we can't do a config run
	val70[0] = 770.0;
	val70[1] = 634.0;
	val70[2] = 710.0;
	val500[0] = 48.0;
	val500[1] = 36.0;
	val500[2] = 43.0;

	for (int i = 0; i < 3; i++)
		lastAverage[i] = 0;

}

void Proximity::start()
{
	SerialUSB.println("Proximity Alert Mode");
	motors.setCurrentPosition(0, 0);
	motors.setAcceleration(1000, 0);  // fast acceleration
	m_currentSpeed = 0;
	m_running = false;
	m_config = false;

	// Enable lights
	lights.setEnabled(true);
}

void Proximity::stop()
{
	SerialUSB.println("Proximity Alert Mode");
	// Disable lights
	lights.setEnabled(false);

}

void Proximity::loop()
{
	lights.getvals();

	if (m_config)
	{
		if (motors.atTargetPosition())
		{
			m_config = false;
			SerialUSB.print("val70 = ");
			SerialUSB.print(val70[0]);
			SerialUSB.print(", ");
			SerialUSB.print(val70[1]);
			SerialUSB.print(", ");
			SerialUSB.println(val70[2]);
			SerialUSB.print("val500 = ");
			SerialUSB.print(val500[0]);
			SerialUSB.print(", ");
			SerialUSB.print(val500[1]);
			SerialUSB.print(", ");
			SerialUSB.println(val500[2]);
		}

		for (int i = 1; i < 4; i++)
		{
			SerialUSB.print(lights.lightval[i]);
			SerialUSB.print(", ");
		}
		SerialUSB.print(motors.currentPositionL());
		SerialUSB.print(", ");
		SerialUSB.println(motors.currentPositionR());
		calcAverages(motors.currentPositionL(), &lights.lightval[1]);
	}

	if (m_running)
		updateSpeed(&lights.lightval[1]);
}

void Proximity::cmd(int arg_cnt, char **args)
{
	char cmd = args[0][0];

	switch (cmd)
	{
	case 'F':
		// Look for back joystick for calibration run
		if (arg_cnt >= 2)
		{
			int32_t speed = cmdStr2Num(args[1], 10);
			if (!m_running)
			{
				if (speed <= -500)
				{
					if (!m_config)
					{
						m_config = true;
						SerialUSB.println("left, middle, right, left_wheel, right_wheel");
						motors.setCurrentPosition(0, 0);
						motors.move(600);
						motors.setSpeedDirection(-SLOWSPEED, 0);
						motors.setEnableOutputs(true);
					}
				}
				else
				{
					m_config = false;
					motors.stop();
				}
			}
		}
		break;

	case 'C':
		// Configure the final stopping distance.
		// This is how far metabot runs after hitting the val70 point
		if (arg_cnt >= 2)
		{
			stoppingdistance = cmdStr2Num(args[1], 10);
			SerialUSB.print("Set stopping distance to : ");
			SerialUSB.println(stoppingdistance);
		}
	}
}

void Proximity::setdmh(bool setting)
{
	if (setting)
	{
		motors.setSpeedDirection(m_currentSpeed, 0);
		motors.setEnableOutputs(true);
		//motors.moveTo(0,0);  // Cancel effect of move in config mode
		m_running = true;
		m_config = false;
	}
	else
	{
		motors.stop();
		m_running = false;
	}
}

void Proximity::calcAverages(int position, int32_t* values)
{
	static int32_t sum[3] = { 0, 0, 0 };
	static int32_t count[3] = { 0, 0, 0 };
	static int32_t lastpos = 0;
	int i;

	if (lastpos == position)
	{
		for (i = 0; i < 3; i++)
		{
			sum[i] += values[i];
			count[i]++;
		}
	}
	else
	{
		for (i = 0; i < 3; i++)
		{
			if (count[i] > 0)
				lastAverage[i] = sum[i] / count[i];
			else
				lastAverage[i] = 0;
			sum[i] = 0;
			count[i] = 0;
			if (m_config)
			{
				if (lastpos == 70)
					val70[i] = lastAverage[i];
				else if (lastpos == 500)
					val500[i] = lastAverage[i];
			}
		}
		lastpos = position;
	}
}

float Proximity::minDiff(float* values1, int32_t* values2)
{
	int i;
	float diff;
	float retval = 1000;

	for (i = 0; i < 3; i++)
	{
		diff = values1[i] - values2[i];
		if (diff < retval)
			retval = diff;
	}
	return retval;
}

void Proximity::updateSpeed(int32_t* values)
{
	// Compare the last average with the val70 and val500 thresholds
	// if the averages are greater than the val500 threshold then continue at high speed
	// if the average is less than the val70 threshold then stop
	// in between run at slow speed.

	float diff500 = minDiff(val500, values);
	float diff70 = minDiff(val70, values);
	SerialUSB.print("Diff500 = "); SerialUSB.print(diff500); SerialUSB.print(" Diff70 = "); SerialUSB.println(diff70);

	if (diff500 > 0)
	{
		m_currentSpeed = HIGHSPEED;
		motors.setSpeed(m_currentSpeed);
		return;
	}

	if (diff70 > 0)
	{
		m_currentSpeed = SLOWSPEED;
		motors.setSpeed(m_currentSpeed);
		return;
	}

	motors.move(stoppingdistance);
	m_running = false;
}

Proximity proximity;

