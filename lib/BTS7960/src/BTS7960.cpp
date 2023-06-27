#include "BTS7960.h"

/* Constructor */
BTS7960::BTS7960(int outPin1, int outPin2, int frequency, int inchDutyCycle, unsigned long inchTimeMs) {
  _outPin1 = outPin1;
  _outPin2 = outPin2;
  _frequency = frequency;
  _inchDutyCycle = inchDutyCycle;
  _inchTimeMs = inchTimeMs;
  moving = false;

  pinMode(_outPin1, OUTPUT);
  pinMode(_outPin2, OUTPUT);  
  // Set the frequency
  // Valid values are from 100Hz to 40000Hz. Default is 1000Hz
  analogWriteFreq(_frequency);
}

/* Move Forward a small amount. */
void BTS7960::inchForward() {

  analogWrite(_outPin2, _inchDutyCycle);
  digitalWrite(_outPin1, LOW);
  
  millisTimer = 0;
  if (_inchTimeMs > 0) {
    moving = true;
    msToMove = _inchTimeMs;
  }
  else {
    moving = false;
    msToMove = 0;
  }

}

/* Move Backward a small amount */
bool BTS7960::inchBack() {
  digitalWrite(_outPin1, LOW);
  analogWrite(_outPin2, 255);
  delay(1000/_frequency);
  digitalWrite(_outPin2, LOW);
  return true;
}
void BTS7960::stop() {
  digitalWrite(_outPin1, LOW);
  digitalWrite(_outPin2, LOW);
  moving = false;
  msToMove = 0;
}
