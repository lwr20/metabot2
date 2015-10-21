// Pin definitions from Arduino DUE variant.cpp file here:
// C:\Users\<user>\AppData\Roaming\Arduino15\packages\arduino\hardware\sam\1.6.4\variants\arduino_due_x
#define X_STEP_PIN         9   // PC21
#define X_DIR_PIN          8   // PC22
#define X_ENABLE_PIN       7   // PC23

#define Y_STEP_PIN         6   // PC24
#define Y_DIR_PIN          5   // PC25
#define Y_ENABLE_PIN       4   // PC26 (and PA29)

// Other constants
#define L_MICROSTEP        16
#define R_MICROSTEP        16
#define MAX_SPEED          6000
#define ACCELERATION       300

// Serial CLI library from https://github.com/fakufaku/CmdArduino
//#include <Cmd.h>
// CmdUSB is a local version for the Arduino DUE that uses the other USB port (i.e. SerialUSB rather than Serial)
#include "CmdUSB.h"
#include "interruptStepper.h"

InterruptStepper stepperL(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
InterruptStepper stepperR(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);

volatile unsigned long mLCount = 0;   // Count of pulses to left motor
volatile unsigned long mRCount = 0;   // Count of pulses to right motor
uint32_t mLChannel = g_APinDescription[X_STEP_PIN].ulPWMChannel;    // Left motor PWM channel
uint32_t mRChannel = g_APinDescription[Y_STEP_PIN].ulPWMChannel;    // Right motor PWM channel

// PWM Interrupt handler
void PWM_Handler()
{
  // This interrupt handler isn't actually needed, but it is useful to monitor
  // pulses sent to each motor and could be used to twiddle different pins if wanted.

  // Look at what events are being signalled
  uint32_t events = PWM->PWM_ISR1;

  if ((events & (1 << mLChannel)) == (1 << mLChannel))
  {
    mLCount++;
  }
    
  else if ((events & (1 << mRChannel)) == (1 << mRChannel))
  {
    mRCount++;
  }
}

void setup()
{
  SerialUSB.begin(9600);
    
  SerialUSB.println("Initialising...");
  SerialUSB.println("Initialising USB...");

  // CMD Setup
  cmdInit(115200);
  cmdAdd("F", setForward);
  cmdAdd("f", setForward);
  cmdAdd("A", setAcceleration);
  cmdAdd("a", setAcceleration);
  cmdAdd("M", setMode);
  cmdAdd("m", setMode);
  
  // Initialise stepper motors
  stepperL.setMaxSpeed(MAX_SPEED);
  stepperL.setAcceleration(ACCELERATION);
  stepperL.setPinsInverted(true, false);
  stepperL.setMicrostep(L_MICROSTEP);

  stepperR.setMaxSpeed(MAX_SPEED);
  stepperR.setAcceleration(ACCELERATION);
  stepperR.setPinsInverted(true, false);
  stepperR.setMicrostep(R_MICROSTEP);

  // Set these just to be sure...
  stepperL.setSpeed(0);
  stepperR.setSpeed(0);
  stepperL.setEnableOutputs(false);
  stepperR.setEnableOutputs(false);

  SerialUSB.println("Ready to roll...");
}

void loop() {
  static int count = 0;
  static int last_secs = 0;

  int secs = millis() / 1000;
  if (secs != last_secs)
  {
    noInterrupts();
    unsigned long mLC = mLCount;
    mLCount = 0;
    unsigned long mRC = mRCount;
    mRCount = 0;
    interrupts();
    last_secs = secs;
    SerialUSB.print(mLC);
    SerialUSB.print(" (");
    SerialUSB.print(mLC / L_MICROSTEP);
    SerialUSB.print(")\t");
    SerialUSB.print(mRC);
    SerialUSB.print(" (");
    SerialUSB.print(mRC / R_MICROSTEP);
    SerialUSB.println(")");
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

    SerialUSB.print("A : ");
    SerialUSB.println(args[1]);
  }
}

void setMode(int arg_cnt, char **args) {
  Serial3.print("<< Setting Mode : ");
  Serial3.println(args[1]);
  SerialUSB.print("M : ");
  SerialUSB.println(args[1]);
}

void setForward(int arg_cnt, char **args) {

  float lspeed;
  float rspeed;

  SerialUSB.print("F : ");
  if (arg_cnt == 1)
  {
    // if no args, stop
    // and disable steppers - this saves power, but allows stepper to freewheel
    stepperL.stop();
    stepperL.setEnableOutputs(false);
    stepperR.stop();
    stepperR.setEnableOutputs(false);
    SerialUSB.println("Stop");
  }
  else
  {
    if (arg_cnt == 2)
    {
      lspeed = float(cmdStr2Num(args[1], 10));
      rspeed = lspeed;
      SerialUSB.println(lspeed);
    }
    else if (arg_cnt == 3)
    {
      lspeed = float(cmdStr2Num(args[1], 10));
      rspeed = float(cmdStr2Num(args[2], 10));
      SerialUSB.print(lspeed);
      SerialUSB.print("  ");
      SerialUSB.println(rspeed);
    }
    else
      return;
    stepperL.setSpeed(lspeed);
    stepperL.setEnableOutputs(true);
    stepperR.setSpeed(rspeed);
    stepperR.setEnableOutputs(true);
  }
}


