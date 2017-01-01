/*
 * ArduPod Remote Control sketch
 * 
 * This sketch is designed to work together with the ardupodRemote processing app.
 * For details, refer to the ArduPod articles at http://deviceplus.com
 * 
 * Brought into existence and debugged out of it by Gipsonek, 2017
 */

#include <AP_Utils.h>

AP_Utils ardupod;

//This array contains servo offsets from center position. See examples/calibration for details
int offsets[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  //Bluetooth module is connected to Serial1
  Serial1.begin(9600);
  while(!Serial1);

  //initiate ardupod
  ardupod.begin(offsets);

  //set default leg positions
  leg legs[6] = {{0, true, 0, -0.8}, {1, true, 0, -0.8}, {2, true, 0, -0.8}, {3, true, 0, -0.8}, {4, true, 0, -0.8}, {5, true, 0, -0.8}};
  ardupod.setLegs(legs);

  //let the user know everythin is finished
  Serial1.println("01Hello sir!");
}

String input;
void loop() {
  //if there are data available at the port, read them
  if(Serial1.available() > 0) {
    input = Serial1.readString();

    //based on the first character of the incoming string, choose proper action
    switch(input.charAt(0)) {
      
      //ultrasonic scan
      case 'n': { Serial1.println("40s");
                  for(int i=0; i<180; i++) {
                    Serial1.print("40");
                    ardupod.moveServo(15, i);
                    Serial1.println((int)ardupod.sr04(TRIG, ECHO, CM));
                  }
                  ardupod.moveServo(15, 90);
                } break;
      
      //measure distance
      case 'm': { Serial1.print("41");
                  Serial1.println((String)(int)ardupod.sr04_median(TRIG, ECHO, CM, 10, 10));
                } break;

      //turn ultrasonic sensor
      case 't': { input.remove(0, 1);
                  int angle = input.toInt();
                  if((angle >= 0) && (angle <= 180)) {
                    ardupod.moveServo(15, angle);
                  }
                } break;

      //step or walk forward
      case 'w': { if(input.charAt(1) == 's') {
                    ardupod.step(FORWARD);
                  } else {
                    //walk
                  }
                } break;
      
      //step or walk left
      case 'a': { if(input.charAt(1) == 's') {
                    ardupod.step(LEFT);
                  } else {
                    //walk
                  }
                } break;

      //step or walk backward
      case 's': { if(input.charAt(1) == 's') {
                    ardupod.step(BACKWARD);
                  } else {
                    //walk
                  }
                } break;

      ////step or walk right
      case 'd': { if(input.charAt(1) == 's') {
                    ardupod.step(RIGHT);
                  } else {
                    //walk
                  }
                } break;
    }
  }
}
