# ArduPod Utilities Library

DISCLAIMER: This library is provided 'AS IS'. See ```license.txt``` for details.

IMPORTANT: Please note that this library requires [Adafruit PWM driver library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/license.txt) to be properly installed in your ```libraries``` folder. Adafruit PWM library is currently not provided with AP_Utils and has to be installed separately!

This library allows even less Arduino-experienced users to program their ArduPod. The main part of this library is lower-level servo and sensor control, plus a higher-level API in Arduino style that allows fro easy ArduPod control. The following is a list of currently implemented or work-in-progress (WIP) features and API description, see included examples for details.


* `AP_Utils(void)`

  Default constructor, used to create AP\_utils object, ie. `AP_Utils ardupod;`


* `begin(int *offsets)`

  This method performs initial setup, similarly to calling `Serial.begin(9600);` you have to call `ardupod.begin(offsets);` before calling any other methods. The `offsets` is an integer array with 16 members. This is to calibrate all the servos to the middle position. See `examples/calibration.ino` for details.

  **Note**: It is necessary to adjust few settings in `AP_Utils.h` according to the physical boundaries of the servos used. On lines 19 and 20, both variables have to be adjusted so that the servos won't get damaged. The easiest approach to this is to set the `SERVOMIN` value higher and `SERVOMAX` lower and uploading the `examples/servo_test.ino`. The servo will try to move back and forth from one end to the other. If the servo isn't reaching the physical limit, adjust the settings back towards their former value a bit, then upload again and oserve the differnce. Repeat this process until the servo is moving to both its limits while not stepping over them! Setting the `SERVOMAX` too high (or `SERVOMIN` too low) might destroy your servo!

  Also in `AP_Utils.h`, on line 12 ther is an option for debug mode. When uncommented, debug data will be sent to the serial port. It is highly recommended to not use this option unless absolutely necessary, as it will slow down everything else considerably.


* `moveServo(uint8_t servo, int deg. bool smooth = true, float speed = 2.5)`

  Move servo number `servo` (numbering consistent with Adafruit PWM Shield from 0 to 15) to position `deg` (where 0 and 180 degrees are the servos physical boundaries). If `smooth` is set to `true`, then the movement will be smooth. The `speed` arguement is optional and defines the speed at which the servo will move, with higher value meaning slower movement.  The value for this can vary for each servo type, most common values are from 2.0 to 5.0.


* `reset(int *offsets)`

  Reset all servos to their middle position with supplied offsets. See `examples/calibration.ino` for details.


* `traceLeg(uint8_t leg, float phi, float z, int resolution)`

  This method calculates a path for specified `leg` to follow, where `phi` and `z` are the leg ending positions and `resolution` is the path resolution. Note that this function itself won't move the leg, it will only return the path as an array of `pointLeg` structures.


* `setLegs(pointLeg *legs, uint8_t *numLegs, uint8_t total, bool smooth = true, float speed = 2.5)`

  Move the `legs` whose numbers are specified in array `numLegs` (the total number of moving legs has to be supplied by the `total` argument). This function will effectively move legs to the positions specified by `legs`, while driving all of them at once. See `examples/walk.ino` for details. 
  
  **Note**: It is NOT possible to achieve the same effect by using multiple calls to `moveServo`, as this function will only move one servo at a time!


* `sr04(uint8_t trig, uint8_t echo, int unit)`

  Take a single measurement with HC-SR04 rangefinder. `trig` and `echo` are numbers of Arduino pins to which the SR04 TRIG and ECHO pins are connected. The result will be returned as a floating point number with unit specified by `unit`: currently supported constants are `MM` for millimters, `CM` for centimiters, and `M` for meters.


* `sr04_average(uint8_t trig, uint8_t echo, int unit, int samples, int time)`

  Take muber of measurements specified by `samples` over `time` milliseconds. The result will be an average of these measurements returned as floating point number with the specified unit.
  

* `sr04_median(uint8_t trig, uint8_t echo, int unit, int samples, int time)`

  Similar to the above, however, this method returns median of all the measurements. This function is recommended over `sr04_average` as it's result are much less prone to be affected by exceedingly wrong measurements.
