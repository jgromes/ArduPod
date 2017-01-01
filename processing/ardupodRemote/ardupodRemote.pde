import processing.serial.*;
import controlP5.*;

ControlP5 cp5;

Serial port;
String[] comList;
int portNum;
boolean selected = false, connected = false, firstScan = false;

int[] scanResults = new int[180];
boolean startScan = false;
int posScan = 0;
int posLine = 90;
int lastAngleSliderValue = 90;

void setup() {
  size(960, 480);
  background(#888888);
  frameRate(120);
  comList = Serial.list();
  guiSetup();
}

public void serialInput(String s) {
  output(11, 9, s);
  if(s.equals("version") || s.equals("v")) {
    output(11, 8, version);
  }
}

public void serialOutput(String s) {
  //tab size fix
  s = s.replaceAll("\t", "        ");
}

public void angleSlider(int angle) {
  posLine = angle;
}

public void portsList(int n) {
  selected = true;
  portNum = n;
}

void mouseReleased() {
  if(angleSlider.isMouseOver() && (lastAngleSliderValue != (int)angleSlider.getValue()) && !(startScan)) {
    lastAngleSliderValue = (int)angleSlider.getValue();
    output(11, 10, "Turning sensor: " + (int)angleSlider.getValue() + "°");
    if(connected) {
      port.write("t" + str((int)angleSlider.getValue()));
    }
  }
}

long lastKey;
void keyPressed() {
  switch(key) {
    case 'w':
      if(!inputField.isFocus()) {
        moveW.setColorBackground(color(0x00, 0xAA, 0xFF));
        moveW.lock();
        if(millis()-lastKey >= 1000) {
          lastKey = millis();
          output(11, 10, "Walk forward");
          if(connected){port.write("w");}
        }
      }
      break;
    case 'a':
      if(!inputField.isFocus()) {
        moveA.setColorBackground(color(0x00, 0xAA, 0xFF));
        moveA.lock();
        if(millis()-lastKey >= 1000) {
          lastKey = millis();
          moveA.setOn();
          output(11, 10, "Turn left");
          if(connected){port.write("a");}
        }
      }
      break;
    case 's':
      if(!inputField.isFocus()) {
        moveS.setColorBackground(color(0x00, 0xAA, 0xFF));
        moveS.lock();
        if(millis()-lastKey >= 1000) {
          lastKey = millis();
          moveS.setOn();
          output(11, 10, "Walk backward");
          if(connected){port.write("s");}
        }
      }
      break;
    case 'd':
      if(!inputField.isFocus()) {
        moveD.setColorBackground(color(0x00, 0xAA, 0xFF));
        moveD.lock();
        if(millis()-lastKey >= 1000) {
          lastKey = millis();
          moveD.setOn();
          output(11, 10, "Turn right");
          if(connected){port.write("d");}
        }
      }
      break;
  }
}

void keyReleased() {
  switch(key) {
    case 'w':
      moveW.setColorBackground(color(0x00, 0x2D, 0x5A));
      moveW.unlock();
      break;
    case 'a':
      moveA.setColorBackground(color(0x00, 0x2D, 0x5A));
      moveA.unlock();
      break;
    case 's':
      moveS.setColorBackground(color(0x00, 0x2D, 0x5A));
      moveS.unlock();
      break;
    case 'd':
      moveD.setColorBackground(color(0x00, 0x2D, 0x5A));
      moveD.unlock();
      break;
  }
}

long lastMouse;
public void controlEvent(ControlEvent theEvent) {
  switch(theEvent.getController().getName()) {
    case "scanButton":
      scanButton.setLabel("scanning...");
      if(connected){port.write("n");}
      posScan = 0;
      posLine = 0;
      break;
    case "measureButton":
      if(connected){port.write("m");}
      break;
    case "connectButton":
      if(selected) {
        try {
          if(!connected) {
            port = new Serial(this, Serial.list()[portNum], 9600);
            connected = true;
            output(11, 8, "Connected to " + Serial.list()[portNum]);
          }
        } catch(Exception e) {
          output(11, 6, "Unable to connect to " + Serial.list()[portNum]);
          output(11, 6, e.toString());
          connected = false;
        }
      } else {
        output(11, 6, "Select appropriate COM port");
      }
      break;
    case "disconnectButton":
      if(connected) {
        output(11, 8, "Disconnected from " + Serial.list()[portNum]);
        port.stop();
        connected = false;
      }
      break;
    case "W":
      if(millis()-lastMouse >= 1000) {
        lastMouse = millis();
        output(11, 10, "Step forward");
        if(connected){port.write("ws");}
      }
      break;
    case "A":
      if(millis()-lastMouse >= 1000) {
        lastMouse = millis();
        output(11, 10, "Step left");
        if(connected){port.write("as");}
      }
      break;
    case "S":
      if(millis()-lastMouse >= 1000) {
        lastMouse = millis();
        output(11, 10, "Step backward");
        if(connected){port.write("ss");}
      }
      break;
    case "D":
      if(millis()-lastMouse >= 1000) {
        lastMouse = millis();
        output(11, 10, "Step right");
        if(connected){port.write("ds");}
      }
      break;
  }
}

void receiveData() {
  String input = port.readStringUntil('\n');
  String payload;
  int code1, code2;
  if (input != null && input.charAt(0) != 0) {
    code1 = (int)input.charAt(0)-48;
    code2 = (int)input.charAt(1)-48;
    payload = trim(input.substring(2));
    if((code1 == 4) && (code2 == 0)) {
      if(payload.equals("s")) {
        startScan = true;
        scanButton.lock();
        angleSlider.lock();
      } else {
        int distance = Integer.parseInt(trim(payload));
        if((distance < 200) && (distance > 0)) {
          scanResults[posLine] = distance;
        } else {
          scanResults[posLine] = 200;
        }
        drawScan(posLine);
        angleSlider.setValueLabel(str(posLine));
        posLine++;
        if(posLine == 180) {
          firstScan = true;
          startScan = false;
          scanButton.unlock();
          scanButton.setLabel("scan");
          angleSlider.unlock();
        }
      }
    } else if((code1 == 4) && (code2 == 1)) {
      distanceArea.setText(payload + " cm");
    } else {
      output(code1, code2, payload);
    }
  }
}

void draw() {
  background(#666666);
  CP5AddOns();
  drawRefreshLine(posLine);
  
  if(connected) {
    receiveData();
  }
}