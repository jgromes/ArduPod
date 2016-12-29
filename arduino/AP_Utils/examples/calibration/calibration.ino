/*
 * ArduPod calibration sketch
 * 
 * Used to calibrate servo offsets so the servos 90 degrees position corresponds with the true middle position.
 * It is highly recommended to run this example and calibrate the servos you will be using.
 * 
 * Calibration guide:
 * The 'offsets' array stores the offsets (in degrees). Usually, this number is between -10 and 10.
 * If you think you need to offset the servo more than that, you should try to unscrew the servo from the axis and move it.
 * Start with 0 offset and increase/decrease the offset and upload again. Repeat until the servo is in the middle position.
 * When you have all the servos in the middle, save the offsets. You can use these in any other sketches.
 * Just initialize the array with the offsets you measured here before calling 'begin()' function in the 'setup()' routine,
 * the same way it is done in servo_test.ino example.
 * 
 * Brought into existence and debugged out of it by Gipsonek, 2016
 */
 
#include <AP_Utils.h>

//create AP_Utils object to access the functions
AP_Utils ardupod;

void setup() {
  //initialize the offset array, you can change the numbers and observe the effect on the servos
  int offsets[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  //start ArduPod with the offsets
  ardupod.begin(offsets);
}

void loop() {}
