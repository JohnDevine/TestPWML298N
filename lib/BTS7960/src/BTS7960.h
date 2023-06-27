#ifndef BTS7960_H
#define BTS7960_H

#include <JDGlobal.h>
#include <elapsedMillis.h>

class BTS7960 {
  public:
  /*
  Duty Cycle is from 0 to 255
  inch time is in ms
  */
    BTS7960(int outPin1, int outPin2, int frequency, int inchDutyCycle, unsigned long inchTimeMs);
    void inchForward(); // returns the number of ms to run. 0 means it's done
    bool inchBack();
    void stop();
    bool moving;
    unsigned long msToMove;
    elapsedMillis millisTimer;

  private:
    int _outPin1;
    int _outPin2;
    int _frequency;
    int _inchDutyCycle;
    unsigned long _inchTimeMs;
};

#endif
