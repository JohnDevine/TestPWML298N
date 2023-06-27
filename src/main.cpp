#include "JDGlobal.h"
#include <elapsedMillis.h>
#include "BTS7960.h"

BTS7960 motor(PIN_D7, PIN_D8, 2000, 150, 100);

// Valid values for duty cycle are 0 - 255. 0 is off, 255 is fully on
int dutyCycle1 = 0; // PWM duty cycle
int dutyCycle2 = 0; // PWM duty cycle
int swapCycle;      // used to switch dutyCycle1 and dutyCycle2

int loopsRequired = 10;

bool forward = true; // true = forward, false = backward
// elapsedMillis millisTimer; // timer for loop

void setup()
{

  Serial.begin(115200);
  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    // do nothing
  }

  // Test inching
  motor.inchForward();

  /*

    // Valid values are from 100Hz to 40000Hz. Default is 1000Hz
    analogWriteFreq(2000);

    // Valid values for duty cycle are 0 - 255. 0 is off, 255 is fully on
    analogWrite(outPin1, dutyCycle1);
    analogWrite(outPin2, dutyCycle2);

    */
}

void loop()
{

  if (motor.millisTimer >= motor.msToMove and motor.moving == true)
  {
    motor.stop();
    if ( loopsRequired-- > 0)
    {
      motor.inchForward();
    }
  }

  

  /**
  if (millisTimer >= 100)
  {
    millisTimer = 0;
    TRACE();
    if (forward)
    {
      dutyCycle1 += 1;
      dutyCycle2 = 0;
      if (dutyCycle1 == 255)
      {
        forward = false;
      }
    }
    else
    {
      dutyCycle1 = 0;
      dutyCycle2 += 1;
      if (dutyCycle2 == 255)
      {
        forward = true;
      }
    }
    analogWrite(outPin1, dutyCycle1);
    analogWrite(outPin2, dutyCycle2);


  }
  */
}