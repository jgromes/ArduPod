#include <AP_Utils.h>

AP_Utils::AP_Utils(void) {
 reset();
}

void AP_Utils::reset(void) {
 for(int i=0; i<11; i++) {
  pwm.setPWM(i, 0, pulseLength(90));
 }
}

void AP_Utils::stretchAll(void) {
 for(int i=0; i<2; i++) {
  legStretch(0, true);
  legStretch(2, true);
  legStretch(4, true);
  delay(100);

  legStretch(1, true);
  legStretch(3, true);
  legStretch(5, true);
  delay(100);
 }
}

void AP_Utils::walk(int dir) {

}

int AP_Utils::pulseLength(int deg) {
 return map(deg, 0, 180, 0, 4095);
}

void AP_Utils::legUp(uint8_t leg, bool smooth) {
 uint8_t servo = vertical[leg];
 //pwm.setPWM(servo, 0, pulseLength(VERT_MAX));
 moveServo(servo, pulseLength(VERT_MAX), smooth);
 #ifdef DEBUG
  Serial.print("[INF]Leg #");
  Serial.print(leg);
  Serial.print(" up ");
  if(smooth) {
   Serial.println("(smooth)");
  }
 #endif
}

void AP_Utils::legDown(uint8_t leg, bool smooth) {
 uint8_t servo = vertical[leg];
 //pwm.setPWM(servo, 0, pulseLength(VERT_MIN));
 moveServo(servo, pulseLength(VERT_MIN), smooth);
 #ifdef DEBUG
  Serial.print("[INF]Leg #");
  Serial.print(leg);
  Serial.print(" down ");
  if(smooth) {
   Serial.println("(smooth)");
  }
 #endif
}

void AP_Utils::moveServo(uint8_t servo, int deg, bool smooth) {
 if(smooth) {
  if(servo % 2 == 0) {
   //horizontal servos
   if((deg >= HORIZ_MIN) || (deg <= HORIZ_MAX)) {
    for(int i=0; i<=deg; i++) {
     pwm.setPWM(servo, 0, pulseLength(i));
     delay((cos(i*(PI/90)) + 1)*7.5);
    }
   } else {
    //out of bounds!
    #ifdef DEBUG
     int bound;
     if(deg >= HORIZ_MAX) {
      bound = HORIZ_MAX;
     } else {
      bound = HORIZ_MIN;
     }
     Serial.print("[ERR]Servo #");
     Serial.print(servo);
     Serial.println(" out of bounds!");
    
     Serial.print("     Command sent : ");
     Serial.print(deg);
     Serial.println(" deg");

     Serial.print("     Current bound: ");
     Serial.print(bound);
     Serial.println(" deg");
    #endif
   }
  } else {
   //vertical servos
   if((deg >= VERT_MIN) || (deg <= VERT_MAX)) {
    for(int i=0; i<=deg; i++) {
     pwm.setPWM(servo, 0, pulseLength(i));
     delay((cos(i*(PI/90)) + 1)*7.5);
    }
   } else {
    //out of bounds!
    #ifdef DEBUG
     int bound;
     if(deg >= VERT_MAX) {
      bound = VERT_MAX;
     } else {
      bound = VERT_MIN;
     }
     Serial.print("[ERR]Servo #");
     Serial.print(servo);
     Serial.println(" out of bounds!");
    
     Serial.print("     Command sent : ");
     Serial.print(deg);
     Serial.println(" deg");

     Serial.print("     Current bound: ");
     Serial.print(bound);
     Serial.println(" deg");
    #endif
   }
  }
 } else {
  if(servo % 2 == 0) {
   //horizontal servos
   if((deg >= HORIZ_MIN) || (deg <= HORIZ_MAX)) {
    pwm.setPWM(servo, 0, pulseLength(deg));
   } else {
    //out of bounds!
    #ifdef DEBUG
     int bound;
     if(deg >= HORIZ_MAX) {
      bound = HORIZ_MAX;
     } else {
      bound = HORIZ_MIN;
     }
     Serial.print("[ERR]Servo #");
     Serial.print(servo);
     Serial.println(" out of bounds!");
    
     Serial.print("     Command sent : ");
     Serial.print(deg);
     Serial.println(" deg");

     Serial.print("     Current bound: ");
     Serial.print(bound);
     Serial.println(" deg");
    #endif
   }
  } else {
   //vertical servos
   if((deg >= VERT_MIN) || (deg <= VERT_MAX)) {
    pwm.setPWM(servo, 0, pulseLength(deg));
   } else {
    //out of bounds!
    #ifdef DEBUG
     int bound;
     if(deg >= VERT_MAX) {
      bound = VERT_MAX;
     } else {
      bound = VERT_MIN;
     }
     Serial.print("[ERR]Servo #");
     Serial.print(servo);
     Serial.println(" out of bounds!");
    
     Serial.print("     Command sent : ");
     Serial.print(deg);
     Serial.println(" deg");

     Serial.print("     Current bound: ");
     Serial.print(bound);
     Serial.println(" deg");
    #endif
   }
  }
 }
}

void AP_Utils::legStretch(uint8_t leg, bool smooth) {
 legUp(leg, smooth);
 delay(50);
 legDown(leg, smooth);
 delay(50);
}
