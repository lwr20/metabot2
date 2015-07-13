// Based on the tutorial from:
// http://www.schmalzhaus.com/EasyDriver/Examples/EasyDriverExamples.html

// Pin definitions from Arduino MEGA section of 
// https://github.com/kliment/Sprinter/blob/master/Sprinter/pins.h

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define E_1_STEP_PIN         36
#define E_1_DIR_PIN          34
#define E_1_ENABLE_PIN       30

// AccelStepper library from http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>
// Serial CLI library from https://github.com/fakufaku/CmdArduino
#include <Cmd.h>

// Define a stepper and the pins it will use
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
int count = 0;

void setup()
{  
  cmdInit(9600);
  cmdAdd("L", setLeft);
  cmdAdd("R", setRight);

  // Speeds are in steps/sec.
  // Motors are 200 steps/rev.
  // Library docs say that speeds above 1K are unreliable...
  stepper1.setMaxSpeed(10000);  // 10K step/s = 50 rps = 30000 rpm.  
  stepper1.setAcceleration(1000);
  stepper1.setEnablePin(X_ENABLE_PIN);
  stepper1.setPinsInverted(false, false, true);
  
  stepper2.setMaxSpeed(10000);
  stepper2.setAcceleration(1000);
  stepper2.setEnablePin(Y_ENABLE_PIN);
  stepper2.setPinsInverted(false, false, true);

  // Set these just to be sure...
  stepper1.setSpeed(0);    
  stepper2.setSpeed(0);    
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}

void loop() {
  count++;
  if (count > 1000) {
    cmdPoll();
    count = 0;    
  }
  stepper1.runSpeed();
  stepper2.runSpeed();
}

void setLeft(int arg_cnt, char **args) {
  int speed;
  if (arg_cnt > 1)
  {
    // if args are present, then use the first arg as the speed
    speed = cmdStr2Num(args[1], 10);
    stepper1.setSpeed(float(speed));
    stepper1.enableOutputs();
  }
  else
  {
    // if no args, stop
    // and disable steppers - this saves power, but allows stepper to freewheel
    stepper1.setSpeed(0);
    stepper1.disableOutputs();
  }
}

void setRight(int arg_cnt, char **args) {
  int speed;
  if (arg_cnt > 1)
  {
    // if args are present, then use the first arg as the speed
    speed = cmdStr2Num(args[1], 10);
    stepper2.setSpeed(float(speed));
    stepper2.enableOutputs();
  }
  else
  {
    // if no args, stop 
    // and disable steppers - this saves power, but allows stepper to freewheel
    stepper2.setSpeed(0);
    stepper2.disableOutputs();
  }
}


