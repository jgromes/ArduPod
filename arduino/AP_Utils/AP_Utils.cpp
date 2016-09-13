#include <AP_Utils.h>

AP_Utils::AP_Utils() {
  
}

void AP_Utils::begin(int *offsets) {
  for(int i=0; i<16; i++) {
    _offsets[i] = *(offsets+i);
  }
  
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
  reset();
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
  if((legs[leg].phi != phi)||(legs[leg].z != z)) {
    pointLeg* path = new pointLeg[resolution];
    //Circular section parameters
    float s = sqrt(pow(phi - legs[leg].phi, 2) + pow(z - legs[leg].z, 2));
    float h = s/2.0; //TODO: adjust
    float r = h/2.0 + pow(s, 2)/(8.0*h);
    //linear function coefficients
    float A = (legs[leg].z - z)/(legs[leg].phi - phi);
    float B = legs[leg].z - A*legs[leg].phi;
    //vector rotation
    float m = sqrt(pow(r, 2) - pow(s/2.0, 2));
    float phi0 = 0.5*(phi - legs[leg].phi) + (m/s)*(z - legs[leg].z) + legs[leg].phi;
    float z0 = 0.5*(z - legs[leg].z) - (m/s)*(phi - legs[leg].phi) + legs[leg].z;
    if(z0 > (A*phi0 + B)) {
      phi0 = 0.5*(phi - legs[leg].phi) - (m/s)*(z - legs[leg].z) + legs[leg].phi;
      z0 = 0.5*(z - legs[leg].z) + (m/s)*(phi - legs[leg].phi) + legs[leg].z;
    }
    
    float u = sqrt(pow(legs[leg].phi - phi, 2) + pow(legs[leg].z - z, 2));
    float theta = acos((pow(u, 2) - 2.0*pow(r, 2))/(-2.0*pow(r, 2)));
    
    float v = sqrt(pow(phi - phi0 - 1.0, 2) + pow(z - z0, 2));
    float delta;
    if(phi > 0) {
      delta = acos((pow(v, 2) - pow(r, 2) - 1.0)/(-2.0*r));
    } else {
      delta = acos((pow(v, 2) - pow(r, 2) - 1.0)/(-2.0*r)) - theta;
    }
    float stepTheta = theta/(float)(resolution-1);
    
    /*Serial.print("s:\t"); Serial.println(s, 6);
    Serial.print("h:\t"); Serial.println(h, 6);
    Serial.print("r:\t"); Serial.println(r, 6);
    Serial.print("A:\t"); Serial.println(A, 6);
    Serial.print("B:\t"); Serial.println(B, 6);
    Serial.print("phi0:\t"); Serial.println(phi0, 6);
    Serial.print("z0:\t"); Serial.println(z0, 6);
    Serial.print("v:\t"); Serial.println(v, 6);
    Serial.print("u:\t"); Serial.println(u, 6);
    Serial.print("delta:\t"); Serial.println(delta*(180.0/PI));
    Serial.print("theta:\t"); Serial.println(theta*(180.0/PI));*/
    
    if(phi > legs[leg].phi) {
      int j = 0;
      for(int i=resolution-1; i>=0; i--) {
        path[i].phi = phi0 + r*cos(stepTheta*(float)j + delta);
        path[i].z = z0 + r*sin(stepTheta*(float)j + delta);
        j++;
      }
    } else if(phi == legs[leg].phi) {
      //TODO: vertical directions
    } else {
      for(int i=0; i<resolution; i++) {
        path[i].phi = phi0 + r*cos(stepTheta*(float)i + delta);
        path[i].z = z0 + r*sin(stepTheta*(float)i + delta);
      }
    }
    
    /*for(int i=0; i<resolution; i++) {
      Serial.print(path[i].phi, 6);
      Serial.print('\t');
      Serial.println(path[i].z, 6);
    }*/
    
    legs[leg].phi = phi;
    legs[leg].z = z;
    return path;
  } else {
    pointLeg* path = new pointLeg[0];
    return path;
  }
}

void AP_Utils::setLegs(leg *legs, bool smooth, float speed) {
  int resolution = 50;
  uint8_t total = 0;
  uint8_t toMove[6] = {255, 255, 255, 255, 255, 255};
  for(int i=0; i<6; i++) {
    if(legs[i].move) {
      toMove[total] = legs[i].number;
      total++;
    }
  }
  
  pointLeg paths[total][resolution];
  for(int i=0; i<total; i++) {
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
      //delayMicroseconds((700.0*(cos((float)j*((2.0*PI))/(float)resolution) + 1.0))*(6.0/total));
    }
    delayMicroseconds((6.0/(float)total)*1000.0);
  }
  
  for(int i=0; i<6; i++) {
    legs[i].move = false;
  }
}

void AP_Utils::walk(float distance, int direction, float speed) {
  float remainingDistance = distance;
  float remainingAngle = direction;
  //create target position
  body target;
  target.x = sin(direction)*distance; 
  target.y = cos(direction)*distance;
  target.z = 0; //TODO: height parameter
  target.facing = direction;
  //calculate path resolution
  float maxStepLength = 0.089; //step = approx. 8.9 cm max for legs 0, 2, 3 and 5
  float stepLength = maxStepLength/1.0;
  
  //TODO: add check for overstepping?
  int numSteps = distance/stepLength;
  while(remainingDistance > 0) {
    //calculate each step separately
    //in each step: turn by direction/numSteps and step forward
    
    
    //repeat 'till we get there!
    remainingDistance -= stepLength;
    if(remainingAngle > 0) {
      remainingAngle -= direction/numSteps;
    }
  }
}

void AP_Utils::step(float length, float speed) {
  
}

void AP_Utils::reset(void) {
  for(int i=0; i<16; i++) {
    pwmove(i, 90+_offsets[i]);
  }
  delay(1000);
  
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  
  for(int i=0; i<6; i++) {
    legs[i].number = i;
    legs[i].move = false;
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
  pwm.setPWM(i, 0, pulseLength(deg+_offsets[i]));
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