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
    Serial.println("\n[WARN]\t[SETUP]\tDebug mode is active, servos will probably move slowly and time-related functions might not work as intended!");
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

void AP_Utils::moveServo(uint8_t number, int deg, bool smooth, float speed) {
  if(checkBounds(number, deg) == -1) {
    if(smooth) {
      float range = abs(servos[number].position - deg);
      if(servos[number].position > deg) {
        for(int i=servos[number].position; i>=deg; i--) {
          pwm.setPWM(servos[number].number, 0, pulseLength(i));
          float pause = (cos(((2.0*PI)/range)*(i-deg))+1.0)*speed;
          delayMicroseconds(pause*1000.0);
        }
      } else if(servos[number].position < deg) {
        for(int i=servos[number].position; i<=deg; i++) {
          pwm.setPWM(servos[number].number, 0, pulseLength(i));
          float pause = (cos(((2.0*PI)/range)*(i-servos[number].position))+1.0)*speed;
          delayMicroseconds(pause*1000.0);
        }
      }
      servos[number].position = deg;
    } else {
      pwmove(number, deg);
    }
  }
}

pointLeg* AP_Utils::traceLeg(uint8_t leg, float phi, float z, int resolution, uint8_t shape) {
  if((legs[leg].phi != phi)||(legs[leg].z != z)) {
    pointLeg* path = new pointLeg[resolution];

    if(shape == LINEAR) {
      #ifdef DEBUG
        Serial.println("[INF]\t[TRACE]\tTracing leg #" + (String)leg + " from [" + (String)legs[leg].phi + "; " + (String)legs[leg].z + "] to [" + (String)phi + "; " + (String)z + "] (linear trace)");
      #endif
      float stepPhi = (phi-legs[leg].phi)/(float)(resolution-1);
      float stepZ = (z-legs[leg].z)/(float)(resolution-1);
      for(int i=0; i<resolution; i++) {
        path[i].phi = stepPhi*(float)i + legs[leg].phi;
        path[i].z = stepZ*(float)i + legs[leg].z;
      }
    }
      
    if(shape == CIRCULAR) {
      float s = sqrt(pow(phi - legs[leg].phi, 2) + pow(z - legs[leg].z, 2));
      float h = s/2.0; //TODO: adjust
      float r = h/2.0 + pow(s, 2)/(8.0*h);
      float A = (legs[leg].z - z)/(legs[leg].phi - phi);
      float B = legs[leg].z - A*legs[leg].phi;
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
      
      #ifdef DEBUG
        Serial.println("[INF]\t[TRACE]\tTracing leg #" + (String)leg + " from [" + (String)legs[leg].phi + "; " + (String)legs[leg].z + "] to [" + (String)phi + "; " + (String)z + "] (circular trace)");
        Serial.println("\t\tTrace parameters:\ts\th\tr\tA\tB\tphi0\tz0\tv\tu\tdelta\ttheta");
        Serial.println("\t\t\t\t\t" + (String)s + "\t" + (String)h + "\t" + (String)r + "\t" + (String)A + "\t" + (String)B + "\t" + (String)phi0 + "\t" + (String)z0 + "\t" + (String)v + "\t" + (String)u + "\t" + (String)(delta*(180.0/PI)) + "\t" + (String)(theta*(180.0/PI)));
      #endif
      
      if(phi > legs[leg].phi) {
        int j = 0;
        for(int i=resolution-1; i>=0; i--) {
          path[i].phi = phi0 + r*cos(stepTheta*(float)j + delta);
          path[i].z = z0 + r*sin(stepTheta*(float)j + delta);
          j++;
        }
      } else if(phi == legs[leg].phi) {
        //TODO: vertical directions?
      } else {
        for(int i=0; i<resolution; i++) {
          path[i].phi = phi0 + r*cos(stepTheta*(float)i + delta);
          path[i].z = z0 + r*sin(stepTheta*(float)i + delta);
        }
      }
    }
    
    if(shape == ELLIPTIC) {
      float phi0 = (legs[leg].phi + phi)/2.0;
      float z0 = legs[leg].z;
      float a = sqrt(pow(phi0 - legs[leg].phi, 2));
      float b = 1.0 - z0;
      
      float theta, delta;
      if(z == legs[leg].z) {
        theta = PI;
        delta = 0;
      } else {
        float v = sqrt(pow(phi - legs[leg].phi, 2) + pow(z - legs[leg].z, 2));
        float u = sqrt(pow(phi - phi0, 2) + pow(z - z0, 2));
        float t = sqrt(pow(phi0 - legs[leg].phi, 2) + pow(z0 - legs[leg].z, 2));
        theta = acos((pow(t, 2) - pow(t, 2) - pow(v, 2))/(-2.0*t*u));
        
        float w = sqrt(pow(phi - phi0 - 1.0, 2) + pow(z - z0, 2));
        if(phi > 0.0) {
          delta = acos((pow(w, 2) - pow(u, 2) - 1.0)/(-2.0*u));
        } else {
          delta = acos((pow(w, 2) - pow(u, 2) - 1.0)/(-2.0*u)) - theta;
        }
      }
      float stepTheta = theta/(float)(resolution-1);
      
      #ifdef DEBUG
        Serial.println("[INF]\t[TRACE]\tTracing leg #" + (String)leg + " from [" + (String)legs[leg].phi + "; " + (String)legs[leg].z + "] to [" + (String)phi + "; " + (String)z + "] (elliptic trace)");
        Serial.println("\t\tTrace parameters:\ta\tb\tphi0\tz0\tdelta\ttheta");
        Serial.println("\t\t\t\t\t" + (String)a + "\t" + (String)b + "\t" + (String)phi0 + "\t" + (String)z0 + "\t" + (String)(delta*(180.0/PI)) + "\t" + (String)(theta*(180.0/PI)));
      #endif
      
      if(phi > legs[leg].phi) {
        int j = 0;
        for(int i=resolution-1; i>=0; i--) {
          path[i].phi = phi0 + a*cos(stepTheta*(float)j + delta);
          path[i].z = z0 + b*sin(stepTheta*(float)j + delta);
          j++;
        }
      } else if(phi == legs[leg].phi) {
        //TODO: vertical directions?
      } else {
        for(int i=0; i<resolution; i++) {
          path[i].phi = phi0 + a*cos(stepTheta*(float)i + delta);
          path[i].z = z0 + b*sin(stepTheta*(float)i + delta);
        }
      }
    }

    #ifdef VERBOSE
      Serial.print("\n\t\tPhi trace:\t\t");
      for(int i=0; i<resolution; i++) {
        Serial.print(path[i].phi);
        Serial.print('\t');
      }
      Serial.print("\n\t\tZ trace:\t\t");
      for(int i=0; i<resolution; i++) {
        Serial.print(path[i].z);
        Serial.print('\t');
      }
      Serial.print("\n\n");
    #endif
    
    legs[leg].phi = phi;
    legs[leg].z = z;
    return path;
  } else {
    pointLeg* path = new pointLeg[0];
    return path;
  }
}

void AP_Utils::setLegs(leg *legs, int shape) {
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
    pointLeg* tmp = traceLeg(toMove[i], legs[toMove[i]].phi, legs[toMove[i]].z, resolution, shape);
    for(int j=0; j<resolution; j++) {
      paths[i][j] = *(tmp+j);
    }
    delete[] tmp;
  }
  
  for(int j=0; j<resolution; j++) {
    for(int i=0; i<total; i++) {
      pwmove(horizontal[toMove[i]], 40.0*paths[i][j].phi + 90.0);
      pwmove(vertical[toMove[i]], 40.0*paths[i][j].z + 90.0);
    }
    delayMicroseconds((6.0/(float)total)*1000.0);
    //TODO: parametric/sine speed
  }
  
  for(int i=0; i<6; i++) {
    legs[i].move = false;
  }
}

void AP_Utils::setLegs(leg *legs, int *shapes) {
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
    pointLeg* tmp = traceLeg(toMove[i], legs[toMove[i]].phi, legs[toMove[i]].z, resolution, shapes[i]);
    for(int j=0; j<resolution; j++) {
      paths[i][j] = *(tmp+j);
    }
    delete[] tmp;
  }
  
  #ifdef VERBOSE
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
    delayMicroseconds((6.0/(float)total)*1000.0);
    //TODO: parametric/sine speed
  }
  
  for(int i=0; i<6; i++) {
    legs[i].move = false;
  }
}

void AP_Utils::stretch(void) {
  leg legs[6] = {{0, false, 0, -0.8}, {1, false, 0, -0.8}, {2, false, 0, -0.8}, {3, false, 0, -0.8}, {4, false, 0, -0.8}, {5, false, 0, -0.8}};
  
  for(int i=0; i<6; i++) {
    legs[i].move = true;
    legs[i].phi = 0;
  }
  setLegs(legs);
  
  legs[0].move = true;
  legs[0].z = 0;
  legs[2].move = true;
  legs[2].z = 0;
  legs[4].move = true;
  legs[4].z = 0;
  setLegs(legs);
  
  legs[0].move = true;
  legs[0].z = -0.8;
  legs[2].move = true;
  legs[2].z = -0.8;
  legs[4].move = true;
  legs[4].z = -0.8;
  setLegs(legs);
  
  legs[1].move = true;
  legs[1].z = 0;
  legs[3].move = true;
  legs[3].z = 0;
  legs[5].move = true;
  legs[5].z = 0;
  setLegs(legs);
  
  legs[1].move = true;
  legs[1].z = -0.8;
  legs[3].move = true;
  legs[3].z = -0.8;
  legs[5].move = true;
  legs[5].z = -0.8;
  setLegs(legs);
}

void AP_Utils::step(int direction) {
  leg legs[6] = {{0, false, 0, -0.8}, {1, false, 0, -0.8}, {2, false, 0, -0.8}, {3, false, 0, -0.8}, {4, false, 0, -0.8}, {5, false, 0, -0.8}};
  
  switch(direction) {
    case FORWARD: { float front = 1.0;
                    float side = 0.6;
                    int A[6] = {ELLIPTIC, LINEAR, ELLIPTIC, LINEAR, ELLIPTIC, LINEAR};
                    legs[0].move = true;
                    legs[0].phi = front;
                    legs[2].move = true;
                    legs[2].phi = front;
                    legs[4].move = true;
                    legs[4].phi = -1.0*side;

                    legs[1].move = true;
                    legs[1].phi = -1.0*side;
                    legs[3].move = true;

                    legs[3].phi = front;
                    legs[5].move = true;
                    legs[5].phi = front;
                    setLegs(legs, A);

                    int B[6] = {LINEAR, ELLIPTIC, LINEAR, ELLIPTIC, LINEAR, ELLIPTIC};
                    legs[1].move = true;
                    legs[1].phi = side;
                    legs[3].move = true;
                    legs[3].phi = -1.0*front;
                    legs[5].move = true;
                    legs[5].phi = -1.0*front;

                    legs[0].move = true;
                    legs[0].phi = -1.0*front;
                    legs[2].move = true;
                    legs[2].phi = -1.0*front;
                    legs[4].move = true;
                    legs[4].phi = side;
                    setLegs(legs, B);
                    
                    stretch();
                  } break;
    case BACKWARD:  { float front = 1.0;
                      float side = 0.6;
                      int A[6] = {ELLIPTIC, LINEAR, ELLIPTIC, LINEAR, ELLIPTIC, LINEAR};
                      legs[0].move = true;
                      legs[0].phi = -1.0*front;
                      legs[2].move = true;
                      legs[2].phi = -1.0*front;
                      legs[4].move = true;
                      legs[4].phi = side;

                      legs[1].move = true;
                      legs[1].phi = side;
                      legs[3].move = true;

                      legs[3].phi = -1.0*front;
                      legs[5].move = true;
                      legs[5].phi = -1.0*front;
                      setLegs(legs, A);

                      int B[6] = {LINEAR, ELLIPTIC, LINEAR, ELLIPTIC, LINEAR, ELLIPTIC};
                      legs[1].move = true;
                      legs[1].phi = -1.0*side;
                      legs[3].move = true;
                      legs[3].phi = front;
                      legs[5].move = true;
                      legs[5].phi = front;

                      legs[0].move = true;
                      legs[0].phi = front;
                      legs[2].move = true;
                      legs[2].phi = front;
                      legs[4].move = true;
                      legs[4].phi = -1.0*side;
                      setLegs(legs, B);
                      
                      stretch();
                    } break;
    case LEFT:  { legs[0].move = true;
                  legs[0].phi = 1;
                  legs[2].move = true;
                  legs[2].phi = 1;
                  legs[4].move = true;
                  legs[4].phi = 1;
                  setLegs(legs, ELLIPTIC);
                  
                  legs[1].move = true;
                  legs[1].phi = 1;
                  legs[3].move = true;
                  legs[3].phi = 1;
                  legs[5].move = true;
                  legs[5].phi = 1;
                  setLegs(legs, ELLIPTIC);
                  
                  for(int i=0; i<6; i++) {
                    legs[i].move = true;
                    legs[i].phi = 0;
                  }
                  setLegs(legs);
                } break;
    case RIGHT: { legs[0].move = true;
                  legs[0].phi = -1;
                  legs[2].move = true;
                  legs[2].phi = -1;
                  legs[4].move = true;
                  legs[4].phi = -1;
                  setLegs(legs, ELLIPTIC);
                  
                  legs[1].move = true;
                  legs[1].phi = -1;
                  legs[3].move = true;
                  legs[3].phi = -1;
                  legs[5].move = true;
                  legs[5].phi = -1;
                  setLegs(legs, ELLIPTIC);
                  
                  for(int i=0; i<6; i++) {
                    legs[i].move = true;
                    legs[i].phi = 0;
                  }
                  setLegs(legs);
                } break;
  }
}

void AP_Utils::turn(int deg) {
  if(deg > 360) {
    deg -= 360;
  }
  int direction = 1;
  if(deg > 180) {
    direction = -1;
    deg -= 180;
  }
  
  leg legs[6] = {{0, true, 0, -0.8}, {1, true, 0, -0.8}, {2, true, 0, -0.8}, {3, true, 0, -0.8}, {4, true, 0, -0.8}, {5, true, 0, -0.8}};
  float size = deg;
  int numSteps = 0;
  while(size > 40.0) {
    numSteps++;
    size = ((float)deg)/((float)numSteps);
  }
  float stepPhi = size/60.0; //TODO: should be 80
  

  for(int i=0; i<(numSteps+1); i++) {
    for(int j=0; j<6; j+=2) {
      legs[j].move = true;
      legs[j].phi = stepPhi;
      legs[j+1].move = true;
      legs[j+1].phi = -2.0*stepPhi;
    }
    int A[6] = {ELLIPTIC, LINEAR, ELLIPTIC, LINEAR, ELLIPTIC, LINEAR};
    setLegs(legs, A);
    
    for(int j=0; j<6; j++) {
      legs[j].move = true;
      legs[j].phi = 0;
    }
    int B[6] = {LINEAR, ELLIPTIC, LINEAR, ELLIPTIC, LINEAR, ELLIPTIC};
    setLegs(legs, B);
  }
}

void AP_Utils::walk(int direction) {
  
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
    return 0;
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
    pause = time/samples - 12.0;
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

int AP_Utils::checkBounds(uint8_t number, int deg) {
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
  if(bound != -1) {
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
  return bound;
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