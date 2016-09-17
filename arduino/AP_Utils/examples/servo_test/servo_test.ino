/*
 * Servo test sketch
 * 
 * Used to calibrate the maximum and minimum boundaries for the servos.
 * It is absolutely necessary to run this example and calibrate the servos you will be using.
 * 
 * Calibration guide:
 * After uploading this example, the first servo (number 0) will sweep from the minimum to the maximum position.
 * You have to check whether it is reaching its physical limit.
 * The easiest way to do that is to disconnect power to the servo shield when the servo is in one of the limit positions.
 * With disconnected power, try to move the servo further in the direction it was moving before.
 * If it can still be moved, try to tweak the 'SERVOMIN' (or 'SERVOMAX') a little bit (about 10 at a time).
 * Do NOT try to move the servo beyond its physical limit, this might destroy your servo!
 * 
 * Brought into existence and debugged out of it by Gipsonek, 2016
 */

#include <Adafruit_PWMServoDriver.h>

//default SERVOMIN and SERVOMAX values
//be extremely careful when changing these, only in small incremets (about 10) at a time
//too big change (100 and more) can cause your servos gears to break!
#define SERVOMIN 150
#define SERVOMAX 530

//create Adafruit PWM Servo Driver object to directly control the PWM outputs
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  //move from max position to min
  for(int pulse=SERVOMAX; pulse>=SERVOMIN; pulse--) {   
    pwm.setPWM(15, 0, pulse);
    Serial.println(pulse);
    delay(10);
  }
  delay(3000);

  //move from min position to max
  for(int pulse=SERVOMIN; pulse<=SERVOMAX; pulse++) {   
    pwm.setPWM(15, 0, pulse);
    Serial.println(pulse);
    delay(10);
  }
  delay(3000);
}
