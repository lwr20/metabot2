// Pin definitions from Arduino MEGA section of 
// https://github.com/kliment/Sprinter/blob/master/Sprinter/pins.h

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define MAX_SPEED       100000
#define ACCELERATION     100

// Serial CLI library from https://github.com/fakufaku/CmdArduino
#include <Cmd.h>
#include "interruptStepper.h"

InterruptStepper stepperL(3, X_DIR_PIN, X_ENABLE_PIN);
InterruptStepper stepperR(4, Y_DIR_PIN, Y_ENABLE_PIN);

volatile unsigned long t3Count = 0;
volatile unsigned long t4Count = 0;

ISR(TIMER3_COMPA_vect)
{
  PINF = 0x01;	//Toggle Pin PF0 = Mega Pin 54, A0 = X_STEP_PIN
  t3Count++;
}

ISR(TIMER4_COMPA_vect)
{
  PINF = 0x40;	//Toggle Pin PF6 = Mega Pin 60, A6 = Y_STEP_PIN
  t4Count++;
}

void setup()
{  
  // CMD Setup
  cmdInit(9600);
  cmdAdd("L", setLeft);
  cmdAdd("R", setRight);
  cmdAdd("F", setForward);
  cmdAdd("A", setAcceleration);
  
  pinMode(X_STEP_PIN,OUTPUT);
  pinMode(Y_STEP_PIN,OUTPUT);
  
  // Initialise stepper motors
  stepperL.setMaxSpeed(MAX_SPEED); 
  stepperL.setAcceleration(ACCELERATION);
  stepperL.setPinsInverted(false, true);

  stepperR.setMaxSpeed(MAX_SPEED);
  stepperR.setAcceleration(ACCELERATION);
  stepperR.setPinsInverted(false, true);
  stepperR.setMicrostep(16);  
  
  // Set these just to be sure...
  stepperL.setSpeed(0);    
  stepperR.setSpeed(0);    
  stepperL.setEnableOutputs(false);
  stepperR.setEnableOutputs(false);
}

void loop() {
  static int count = 0;
  static int last_secs = 0;
  
  int secs = millis() / 1000;
  if (secs != last_secs)
  {
	noInterrupts();
	unsigned long t3C = t3Count;
	t3Count = 0;
	unsigned long t4C = t4Count;
	t4Count = 0;
	interrupts();
	last_secs = secs;
	Serial.print(t3C); 
	Serial.print("\t");
	Serial.print(t4C);
	Serial.print("\t");
	Serial.println(t4C/16);
  }

  count++;
  if (count > 1000) {
    cmdPoll();
    count = 0;    
  }
  
  stepperL.run();
  stepperR.run();
}

void setAcceleration(int arg_cnt, char **args) {
	if (arg_cnt > 1)
	{
		int acc = cmdStr2Num(args[1], 10);
		stepperL.setAcceleration(float(acc));
		stepperR.setAcceleration(float(acc));
	}
}

void setLeft(int arg_cnt, char **args) {
  setSpeed(&stepperL, arg_cnt, args);
}

void setRight(int arg_cnt, char **args) {
  setSpeed(&stepperR, arg_cnt, args);
}

void setForward(int arg_cnt, char **args) {
	setRight(arg_cnt, args);
	setLeft(arg_cnt, args);
}

void setSpeed(InterruptStepper* stepper, int arg_cnt, char **args) {

  if (arg_cnt > 1)
  {
    // if args are present, then use the first arg as the speed
    int speed = cmdStr2Num(args[1], 10);
    stepper->setSpeed(float(speed));
    stepper->setEnableOutputs(true);
  }
  else
  {
    // if no args, stop 
    // and disable steppers - this saves power, but allows stepper to freewheel
    stepper->stop();
    stepper->setEnableOutputs(false);
  }
}

