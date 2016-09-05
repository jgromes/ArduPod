#include <AP_Utils.h>

AP_Utils::AP_Utils(void) {
  
}

void AP_Utils::begin(int *offsets) {
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
    Serial.println("\n[WARN]\t\tDebug mode is active, servos will probably move slowly and time-related functions might not work as intended!");
    Serial.println("\t\tTo disable debug mode, comment out line 12 in AP_Utils.h");
    Serial.print("\t\tResuming setup in ");
    for(int i=10; i>0; i--) {
      Serial.print(i);
      delay(250);
      for(int j=0; j<3; j++) {
        Serial.print('.');
        delay(250);
      }
    }
    Serial.print("\n[INF]\t[SETUP]\tStarting PWM ... ");
  #endif
  pwm.begin();
  #ifdef DEBUG
    Serial.println("Done!");
    Serial.print("[INF]\t[SETUP]\tSetting PWM frequency to ");
    Serial.print(PWM_FREQ);
    Serial.print(" Hz ... ");
  #endif
  pwm.setPWMFreq(PWM_FREQ);
  #ifdef DEBUG
    Serial.println("Done!");
    Serial.print("[INF]\t[SETUP]\tSetting all servos to default position ... ");
  #endif
  reset(offsets);
  #ifdef DEBUG
    Serial.println("Done!");
    Serial.println("[INF]\t[SETUP]\tSuccesfully started!");
  #endif
}

void AP_Utils::moveServo(uint8_t number, int deg, bool smooth, float speed) {
  int bound = -1;
  if(servos[number].type == HORIZONTAL) {
    if(deg > HORIZ_MAX) {
      bound = HORIZ_MAX;
    } else if(deg < HORIZ_MIN) {
      bound = HORIZ_MIN;
    }
  } else if(servos[number].type == VERTICAL) {
    if(deg > VERT_MAX) {
      bound = VERT_MAX;
    } else if(deg < VERT_MIN) {
      bound = VERT_MIN;
    }
  }
  
  if(bound == -1) {
    if(smooth) {
      float range = abs(servos[number].position - deg);
      if(servos[number].position > deg) {
        for(int i=servos[number].position; i>=deg; i--) {
          pwm.setPWM(servos[number].number, 0, pulseLength(i));
          float pause = (cos(((2.0*PI)/range)*(i-deg))+1.0)*speed;
          delayMicroseconds(pause*1000.0);
          //Serial.println(pause);
        }
      } else if(servos[number].position < deg) {
        for(int i=servos[number].position; i<=deg; i++) {
          pwm.setPWM(servos[number].number, 0, pulseLength(i));
          float pause = (cos(((2.0*PI)/range)*(i-servos[number].position))+1.0)*speed;
          delayMicroseconds(pause*1000.0);
          //Serial.println(pause);
        }
      }
      servos[number].position = deg;
    } else {
      pwmove(number, deg);
    }
  } else {
    #ifdef DEBUG
      Serial.print("[ERROR]\t[PWM]\tServo #");
      Serial.print(servos[number].number);
      Serial.println(" out of bounds!");

      Serial.print("\t\tCommand sent : ");
      Serial.print(deg);
      Serial.println(" deg");

      Serial.print("\t\tCurrent bound: ");
      Serial.print(bound);
      Serial.println(" deg");
    #endif
  }
}

pointLeg* AP_Utils::traceLeg(uint8_t leg, float phi, float z, int resolution) {
  pointLeg* path = new pointLeg[resolution];
  float stepPhi = (phi-legs[leg].phi)/(float)(resolution-1);
  float stepZ = (z-legs[leg].z)/(float)(resolution-1);
  for(int i=0; i<resolution; i++) {
    path[i].phi = stepPhi*(float)i + legs[leg].phi;
    path[i].z = stepZ*(float)i + legs[leg].z;
  }
  legs[leg].phi = phi;
  legs[leg].z = z;
  return path;
}

//TODO: remove numLegs parameter (no idea how, other than std::vector)
void AP_Utils::setLegs(pointLeg *legs, uint8_t *numLegs, uint8_t total, bool smooth, float speed) {
  int resolution = 50;
  pointLeg paths[total][resolution];
  uint8_t toMove[total];
  for(int i=0; i<total; i++) {
    toMove[i] = *(numLegs+i);
    pointLeg* tmp = traceLeg(toMove[i], legs[toMove[i]].phi, legs[toMove[i]].z, resolution);
    for(int j=0; j<resolution; j++) {
      paths[i][j] = *(tmp+j);
    }
    delete[] tmp;
  }
  
  #ifdef DEBUG
    for(int i=0; i<total; i++) {
      Serial.print("[INF]\t[LEG ");
      Serial.print(toMove[i]);
      Serial.print("]\tPhi:\t(raw)\t");
      for(int j=0; j<resolution; j++) {
        Serial.print(paths[i][j].phi, 3);
        Serial.print('\t');
      }
      Serial.print("\n\t\t\t(deg)\t");
      for(int j=0; j<resolution; j++) {
        Serial.print(40.0*paths[i][j].phi + 90.0);
        Serial.print('\t');
      }
      Serial.print("\n\t\tZ:\t(raw)\t");
      for(int j=0; j<resolution; j++) {
        Serial.print(paths[i][j].z, 3);
        Serial.print('\t');
      }
      Serial.print("\n\t\t\t(deg)\t");
      for(int j=0; j<resolution; j++) {
        Serial.print(40.0*paths[i][j].z + 90.0);
        Serial.print('\t');
      }
      Serial.println();
    }
  #endif
  
  for(int j=0; j<resolution; j++) {
    for(int i=0; i<total; i++) {
      pwmove(horizontal[toMove[i]], 40.0*paths[i][j].phi + 90.0);
      pwmove(vertical[toMove[i]], 40.0*paths[i][j].z + 90.0);
    }
  }
}

void AP_Utils::reset(int *offsets) {
  for(int i=0; i<16; i++) {
    pwmove(i, 90+offsets[i]);
  }
  delay(DELAY);
  
  body.x = 0;
  body.y = 0;
  body.z = 0;
  
  for(int i=0; i<6; i++) {
    legs[i].phi = 0;
    legs[i].z = 0;
  }
}

float AP_Utils::sr04(uint8_t trig, uint8_t echo, int unit) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
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
      Serial.print("[INF]\t[SR04]\tDistance: ");
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
      Serial.print("[ERROR]\t[SR04]\tDistance ");
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
    return -1;
  }
}

float AP_Utils::sr04_average(uint8_t trig, uint8_t echo, int unit, int samples, int time) {
  #ifdef DEBUG
    unsigned long timer = micros();
    Serial.print("[INF]\t[SR04]\tMeasuring average distance (");
    Serial.print(samples);
    Serial.print(" samples over ");
    Serial.print(time);
    Serial.println(" ms)");
  #endif
  float average, pause, value;
  float total = 0;
  if(time/samples <= 12) {
  #ifdef DEBUG
      Serial.println("[WARN]\t[SR04]\tSamples to time ratio too low, setting to default.");
  #endif
    pause = 0;
  } else {
    pause = time/samples - 12;
  }
  for(int i=0; i<samples; i++) {
    value = sr04(trig, echo, unit);
    if(value != -1) {
      total += value;
      delay(pause);
    } else {
      i--;
    }
  }
  average = total/samples;
  #ifdef DEBUG
    Serial.print("[INF]\t[SR04]\tCalculated average distance: ");
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
    Serial.print("[INF]\t[SR04]\tSampling took ");
    Serial.print((micros()-timer)/1000);
    Serial.println(" ms.");
  #endif
  return average;
}

float AP_Utils::sr04_median(uint8_t trig, uint8_t echo, int unit, int samples, int time) {
  #ifdef DEBUG
    unsigned long timer = micros();
    Serial.print("[INF]\t[SR04]\tMeasuring median distance (");
    Serial.print(samples);
    Serial.print(" samples over ");
    Serial.print(time);
    Serial.println(" ms)");
  #endif
  float med, pause, value;
  float *values;
  values = new float[samples];
  if(time/samples < 12) {
  #ifdef DEBUG
      Serial.println("[WARN]\t[SR04]\tSamples to time ratio too low, setting to default.");
  #endif
    pause = 0;
  } else {
    pause = time/samples - 12;
  }
  for(int i=0; i<samples; i++) {
    value = sr04(trig, echo, unit);
    if(value != -1) {
      values[i] = value;
      delay(pause);
    } else {
      i--;
    }
  }
  
  med = median(values, samples);
  delete [] values;
  #ifdef DEBUG
    Serial.print("[INF]\t[SR04]\tCalculated median distance: ");
    Serial.print(med);
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
    Serial.print("[INF]\t[SR04]\tSampling took ");
    Serial.print((micros()-timer)/1000);
    Serial.println(" ms.");
  #endif
  return(med);
}

int AP_Utils::pulseLength(int deg) {
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void AP_Utils::pwmove(uint8_t i, int deg) {
  pwm.setPWM(i, 0, pulseLength(deg));
  servos[i].position = deg;
}

void AP_Utils::legUp(uint8_t leg, bool smooth, float speed) {
  uint8_t servo = vertical[leg];
  moveServo(servo, pulseLength(VERT_MAX), smooth, speed);
  #ifdef DEBUG
    Serial.print("[INF]\t[PWM]\tLeg #");
    Serial.print(leg);
    Serial.print(" up ");
    if(smooth) {
      Serial.println("(smooth)");
    } else {
      Serial.println();
    }
  #endif
}

void AP_Utils::legDown(uint8_t leg, bool smooth, float speed) {
  uint8_t servo = vertical[leg];
  moveServo(servo, pulseLength(VERT_MIN), smooth, speed);
  #ifdef DEBUG
    Serial.print("[INF]\t[PWM]\tLeg #");
    Serial.print(leg);
    Serial.print(" down ");
    if(smooth) {
      Serial.println("(smooth)");
    } else {
      Serial.println();
    }
  #endif
}

float AP_Utils::median(float *values, int numValues) {
  for(int i=0; i<(numValues-1); i++) {
    for(int j=0; j<(numValues-i-1); j++) {
      if(values[j+1] < values[j]) {
        float swap = values[j];
        values[j] = values[j+1];
        values[j+1] = swap;
      }
    }
  }
  
  if(numValues%2 == 0) {
    return((values[numValues/2-1]+values[numValues/2])/2.0);
  }
  return(values[numValues/2]);
}