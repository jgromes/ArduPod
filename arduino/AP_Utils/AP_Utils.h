#ifndef _AP_Utilities_H
#define _AP_Utilities_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_PWMServoDriver.h>

//#define DEBUG

#define PWM_FREQ 60
#define M  1
#define CM 100
#define MM 1000

#define SERVOMIN    150
#define SERVOMAX    560

#define HORIZONTAL  0
#define VERTICAL    1
#define SENSOR      2

#define HORIZ_MAX   50
#define HORIZ_DEF   90
#define HORIZ_MIN   130
#define VERT_MAX    50 
#define VERT_DEF    90
#define VERT_MIN    130

#define DELAY       100

struct servo {
  uint8_t number;
  uint8_t type;
  int position;
};

class AP_Utils {
 public:
  AP_Utils(void);
  void begin(void);
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  servo servos[16];
  void moveServo(uint8_t servo, int deg, bool smooth);
  void reset(void);
  void stretchAll(void);
  void walk(int dir);
  
  float sr04(uint8_t trig, uint8_t echo, int unit);
  float sr04_average(uint8_t trig, uint8_t echo, int unit, int samples, int time);
  //float sr04_median(uint8_t trig, uint8_t echo, int unit, int samples, int time); //TODO because #fuckaverage
 private:
  uint8_t horizontal[6] = {0, 2, 4, 6, 8, 10};
  uint8_t vertical[6] = {1, 3, 5, 7, 9, 11};

  int pulseLength(int deg);
  void legUp(uint8_t leg, bool smooth);
  void legDown(uint8_t leg, bool smooth);
  void legStretch(uint8_t leg, bool smooth);
  void pwmove(uint8_t i, int deg);
};

#endif
