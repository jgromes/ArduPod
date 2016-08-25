#include <AP_Utils.h>

AP_Utils::AP_Utils(void) {
  
}

void AP_Utils::begin(void) {
  for(int i=0; i<16; i++) {
    servos[i].number = i;
    if((i % 2 == 0)&&(i<12)) {
      servos[i].type = HORIZONTAL;
    } else if(i==15) {
      servos[i].type = SENSOR;
    } else {
      servos[i].type = VERTICAL;
    }
  }
  #ifdef DEBUG
    Serial.print("\n[INF]\tStarting PWM ... ");
  #endif
  pwm.begin();
  #ifdef DEBUG
    Serial.println("Done!");
    Serial.print("[INF]\tSetting PWM frequency to ");
    Serial.print(PWM_FREQ);
    Serial.print(" Hz ... ");
  #endif
  pwm.setPWMFreq(PWM_FREQ);
  #ifdef DEBUG
    Serial.println("Done!");
    Serial.print("[INF]\tSetting all servos to default position ... ");
  #endif
  reset();
  #ifdef DEBUG
    Serial.println("Done!");
    Serial.println("[INF]\tSuccesfully started!");
  #endif
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
    #ifdef DEBUG
      Serial.print("[INF]\tSR04 distance: ");
      Serial.print(distance);
      switch(unit) {
        case MM:
          Serial.println(" mm");
          break;
        case CM:
          Serial.println(" cm");
          break;
        case M:
          Serial.println(" m");
          break;
      }
    #endif
    return(distance);
  } else {
    #ifdef DEBUG
      Serial.print("[ERROR]\tSR04 distance ");
      Serial.print(distance);
      switch(unit) {
        case MM:
          Serial.print(" mm ");
          break;
        case CM:
          Serial.print(" cm ");
          break;
        case M:
          Serial.print(" m ");
          break;
      }
      Serial.println("is out of bounds for SR04, ignoring.");
    #endif
    return 0;
  }
}

float AP_Utils::sr04_average(uint8_t trig, uint8_t echo, int unit, int samples, int time) {
  #ifdef DEBUG
    Serial.print("[INF]\tSR04 measuring average distance (");
    Serial.print(samples);
    Serial.print(" samples over ");
    Serial.print(time);
    Serial.println(" ms)");
  #endif
  float average, pause;
  float total = 0;
  if(time/samples < 12) {
  #ifdef DEBUG
      Serial.println("[WARN]\tSamples to time ratio too low, setting to default.");
  #endif
    pause = 0;
  } else {
    pause = time/samples - 12;
  }
  for(int i=0; i<samples; i++) {
    total += sr04(trig, echo, unit);
    delay(pause);
  }
  average = total/samples;
  #ifdef DEBUG
    Serial.print("[INF]\tSR04 calculated average distance: ");
    Serial.print(average);
    switch(unit) {
      case MM:
        Serial.println(" mm");
        break;
      case CM:
        Serial.println(" cm");
        break;
      case M:
        Serial.println(" m");
        break;
    }
  #endif
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
  moveServo(servo, pulseLength(VERT_MAX), smooth);
  #ifdef DEBUG
    Serial.print("[INF]\tLeg #");
    Serial.print(leg);
    Serial.print(" up ");
    if(smooth) {
      Serial.println("(smooth)");
    } else {
      Serial.println();
    }
  #endif
}

void AP_Utils::legDown(uint8_t leg, bool smooth) {
  uint8_t servo = vertical[leg];
  moveServo(servo, pulseLength(VERT_MIN), smooth);
  #ifdef DEBUG
    Serial.print("[INF]\tLeg #");
    Serial.print(leg);
    Serial.print(" down ");
    if(smooth) {
      Serial.println("(smooth)");
    } else {
      Serial.println();
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
  #ifdef DEBUG
    int bound = -1;
    if(moving.type == HORIZONTAL) {
      if(deg > HORIZ_MAX) {
        bound = HORIZ_MAX;
      } else if(deg < HORIZ_MIN) {
        bound = HORIZ_MIN;
      }
    } else if(moving.type == VERTICAL) {
      if(deg > VERT_MAX) {
        bound = VERT_MAX;
      } else if(deg < VERT_MIN) {
        bound = VERT_MIN;
      }
    }
    if(bound != -1) {
      Serial.print("[ERROR]\tServo #");
      Serial.print(moving.number);
      Serial.println(" out of bounds!");

      Serial.print("     Command sent : ");
      Serial.print(deg);
      Serial.println(" deg");

      Serial.print("     Current bound: ");
      Serial.print(bound);
      Serial.println(" deg");
    }
  #endif
}

void AP_Utils::legStretch(uint8_t leg, bool smooth) {
  legUp(leg, smooth);
  delay(50);
  legDown(leg, smooth);
  delay(50);
}
