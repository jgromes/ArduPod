#ifndef _AP_Utilities_H
#define _AP_Utilities_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#define DEBUG

#define SERVOMIN    150
#define SERVOMAX    600

#define HORIZ_MAX   50
#define HORIZ_DEF   90
#define HORIZ_MIN   130
#define VERT_MAX    50 
#define VERT_DEF    90
#define VERT_MIN    130

class AP_Utils {
 public:
  AP_Utils(void);
  void reset(void);
  void stretchAll(void);
  void walk(int dir);
 private:
  //uint8_t _horiz_max, _horiz_def, _horiz_min, _vert_max, _vert_def, _vert_min;
  uint8_t horizontal[6] = {0, 2, 4, 6, 8, 10};
  uint8_t vertical[6] = {1, 3, 5, 7, 9, 11};
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

  int pulseLength(int deg);
  void legUp(uint8_t leg, bool smooth);
  void legDown(uint8_t leg, bool smooth);
  void legStretch(uint8_t leg, bool smooth);

  void moveServo(uint8_t servo, int deg, bool smooth);
  //void smoothMove(uint8_t servo, int deg);
};

#endif
