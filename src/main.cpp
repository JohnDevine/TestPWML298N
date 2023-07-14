#include "JDGlobal.h"
#include <elapsedMillis.h>
#include "BTS7960.h"

elapsedMillis wipTimer;

/*
 pwmPin1 = pin for pwm1 analog output to H-Bridge
 pwmPin2 = pin for pwm2 analog output to H-Bridge
 pwmFrequency = frequency for pwm1 and pwm2 in Hz. Usually 2000 Hz.
 minDutyCycle = minimum duty cycle for pwm1 and pwm2. This is the lowest duty cycle before the motor stalls under normal load.
 stopSwitch1Pin = pin for stop switch 1
 stopSwitch2Pin = pin for stop switch 2
 maxCurrent = (float)maximum current in mA. Over this and system will stop.
 */
BTS7960 motor;

void setup()
{

  Serial.begin(115200);
  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    // do nothing
  }
  TRACE();
  /*
pwmPin1 = pin for pwm1 analog output to H-Bridge
pwmPin2 = pin for pwm2 analog output to H-Bridge
pwmFrequency = frequency for pwm1 and pwm2 in Hz. Usually 2000 Hz.
minDutyCycle = minimum duty cycle for pwm1 and pwm2. This is the lowest duty cycle before the motor stalls under normal load.
stopSwitch1Pin = pin for stop switch 1
stopSwitch2Pin = pin for stop switch 2
maxCurrent = (float)maximum current in mA. Over this and system will stop.
*/
  motor.begin(PIN_D3, PIN_D4, 2000, 50, PIN_D5, PIN_D6, (float)4000.0);
  wipTimer = 0;

  // Go Forward
  /*
   forward paramaters are:
   requestedDutyCycle = duty cycle for pwm1 and pwm2. 0 to 255.
   time = duration in ms to go forward
   accelDuration = duration in ms for acceleration (must be less than time)
   typeOfAcceletation is one of = none,rampUp,rampDown,rampUpDown
   */

  motor.forward(255, 10000, 3000, rampUpDown);
  wipTimer = 0;
}

void loop()
{

  motor.run();
}
