#include <AP_Utils.h>

AP_Utils::AP_Utils(void) {
  for(int i=0; i<16; i++) {
    servos[i].number = i;
    if(i % 2 == 0) {
      servos[i].type = HORIZONTAL;
    } else {
      servos[i].type = VERTICAL;
    }
  }
  pwm.begin();
  pwm.setPWMFreq(60);
  pwm.setPWM(15, 0, pulseLength(90));
  reset();
}

void AP_Utils::reset(void) {
  for(int i=0; i<16; i++) {
    pwm.setPWM(i, 0, pulseLength(90));
    servos[i].position = 90;
  }
}

void AP_Utils::stretchAll(void) {
  for(int i=0; i<2; i++) {
    legStretch(0, true);
    legStretch(2, true);
    legStretch(4, true);
    delay(DELAY);

    legStretch(1, true);
    legStretch(3, true);
    legStretch(5, true);
    delay(DELAY);
  }
}

void AP_Utils::walk(int dir) {

}

float AP_Utils::sr04(uint8_t trig, uint8_t echo, int unit) {
  float duration, distance;
  digitalWrite(trig, LOW); 
  delayMicroseconds(2); 

  digitalWrite(trig, HIGH);
  delayMicroseconds(10); 
 
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
 
  distance = (346.3*duration*0.000001*unit)/2;
  
  if((distance >= 0.02*unit) && (distance <= 4*unit)) {
    return(distance);
  } else {
    return 0;
  }
}

float AP_Utils::sr04_average(uint8_t trig, uint8_t echo, int unit, int samples, int time) {
  float duration, distance, average;
  float total = 0;
  float pause = time/samples;
  for(int i=0; i<samples; i++) {
    digitalWrite(trig, LOW); 
    delayMicroseconds(2); 

    digitalWrite(trig, HIGH);
    delayMicroseconds(10); 
   
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
   
    distance = (346.3*duration*0.000001*unit)/2;
    
    if((distance >= 0.02*unit) && (distance <= 4*unit)) {
      Serial.println(distance);
      total += distance;
      delay(pause);
    } else {
      Serial.println("E");
      i--;
    }
  }
  average = total/samples;
  return average;
}

int AP_Utils::pulseLength(int deg) {
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void AP_Utils::pwmove(uint8_t i, int deg) {
  pwm.setPWM(i, 0, pulseLength(deg));
  servos[i].position = deg;
  delay(DELAY);
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

void AP_Utils::moveServo(uint8_t number, int deg, bool smooth) {
  servo moving = servos[number];
  if(smooth) {
    if(moving.type == HORIZONTAL) {
      if(moving.position > deg) {
        for(int i=moving.position; i<=deg; i++) {
          pwm.setPWM(moving.number, 0, pulseLength(i));
          delay((cos(i*(PI/90)) + 1)*5);
        }
      } else if(moving.position < deg) {
        for(int i=moving.position; i>=deg; i--) {
          pwm.setPWM(moving.position, 0, pulseLength(i));
          delay((cos(i*(PI/90)) + 1)*5);
        }
      }
    } else if(moving.type == VERTICAL) {
      if(moving.position > deg) {
        for(int i=moving.position; i<=deg; i++) {
          pwm.setPWM(moving.number, 0, pulseLength(i));
          delay((cos(i*(PI/90)) + 1)*5);
        }
      } else if(moving.position < deg) {
        for(int i=moving.position; i>=deg; i--) {
          pwm.setPWM(moving.number, 0, pulseLength(i));
          delay((cos(i*(PI/90)) + 1)*5);
        }
      }
    }
  } else {
    pwmove(number, deg);
  }
}

void AP_Utils::legStretch(uint8_t leg, bool smooth) {
  legUp(leg, smooth);
  delay(50);
  legDown(leg, smooth);
  delay(50);
}
