// InterruptStepper.h

#ifndef InterruptStepper_h
#define AccelStepper_h
#include <stdlib.h>

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

#define CLKDIV 1024         // Starting point for Clock Divider. Note that microstepping will change what is actually programmed
#define CLKFREQ (VARIANT_MCK / CLKDIV)
#define MAXPERIODTICKS 65535
#define MINPERIODTICKS 10

class InterruptStepper
{
  public:
    InterruptStepper(uint32_t pwmPin, uint32_t dirPin, uint32_t enablePin);
    void setSpeed(float speed);
    void setEnableOutputs(bool enabled);
    void setMicrostep(int microstep);
    void setPinsInverted(bool directionInvert, bool enableInvert);

  private:
    void		   setPWMPeriod(uint16_t period);

    uint32_t       _pwmPin;
    uint32_t       _dirPin;
    uint32_t       _enablePin;
    uint32_t       _chan;
    bool           _dirInverted;
    bool           _enableInverted;
};

#endif
