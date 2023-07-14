#ifndef BTS7960_H // Include guard to prevent multiple declarations
#define BTS7960_H
#include "JDGlobal.h"      // Include the JDGlobal header file
#include <elapsedMillis.h> // Include the elapsedMillis library
#include <Bounce2.h>
#include "Adafruit_INA219.h" // INA219 current sensor library

enum typeOfAcceleration
{
  none,
  rampUp,
  rampDown,
  rampUpDown
};

enum motorDirection
{
  unknown,
  forward,
  backward
};

class BTS7960
{
private: // Access specifier for private members
  // Data members
  int _pwmPin1;
  int _pwmPin2;
  int _pwmFrequency;
  int _minDutyCycle;
  int _beginSwitchPin;
  int _endSwitchPin;
  float _maxCurrent;

  unsigned long _motorRequestedDuration;
  motorDirection _motorDirection;
  elapsedMillis _motorRunTimeTillNow; // Keep track of how long the motor has been running
  int _currentDutyCycle;
  int _requestedDutyCycle;
  elapsedMillis _waitForCurrentSensor; // Timer used in waiting for current sensor.

  Bounce _beginSwitch;
  Bounce _endSwitch;

  Adafruit_INA219 _currentSensor;

  unsigned long _accelDuration;
  typeOfAcceleration _accelType;
  elapsedMillis _accelTimer;

public: // Access specifier for public members
  /*
  pwmPin1 = pin for pwm1 analog output to H-Bridge
  pwmPin2 = pin for pwm2 analog output to H-Bridge
  pwmFrequency = frequency for pwm1 and pwm2 in Hz. Usually 2000 Hz.
  minDutyCycle = minimum duty cycle for pwm1 and pwm2. This is the lowest duty cycle before the motor stalls under normal load.
  beginSwitchPin = pin for stop switch 1
  endSwitchPin = pin for stop switch 2
  currentPin = pin for current sensor
  maxCurrent = (float)maximum current in mA. Over this and system will stop.
  */
  BTS7960();
  // Member functions
  void begin(int pwmPin1, int pwmPin2, int pwmFrequency, int minDutyCycle, int stopSwitch1Pin, int stopSwitch2Pin, float maxCurrent);
  /*
  forward paramaters are:
  requestedDutyCycle = duty cycle for pwm1 and pwm2. 0 to 255.
  time = duration in ms to go forward
  accelDuration = duration in ms for acceleration (must be less than time)
  typeOfAcceletation is one of = none,rampUp,rampDown,rampUpDown
  */
  void forward(int requestedDutyCycle, long time, long accelDuration, typeOfAcceleration accelType); // Drive forward with a given speed for time (ms). How long for acceleration in ms and type of acceleration (None, RampUp, RampDown, Both)
  void backward(int requestedDutyCycle, long time, long accelDuration, typeOfAcceleration accelType); // Drive backward with a given speed for time (ms). How long for acceleration in ms and type of acceleration (None, RampUp, RampDown, Both)
  void run();                                                                           // Run the motor doing acceleration, deacceleration and stopping at requested time.
  void stop();                                                                          // Stop the motor
  float getCurrent();                                                                   // Get the current reading from the current pin
  void displayCurrent();                                                                // Display the current reading from the current pin
  void setPwmFrequency(int frequency);                                                  // Set the PWM frequency
  bool setAcceleration(long accelDuration, typeOfAcceleration accelType); // Drive backward with a given speed
  void resetAcceleration();
  int calculateDutyCycle();
  int calculateBeginningDutyCycle();
  int calculateEndingDutyCycle();
  bool motorIsRunning();
};

#endif
