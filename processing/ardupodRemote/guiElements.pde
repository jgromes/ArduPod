Textarea outputArea, distanceArea;
Textlabel angle0Label, angle180Label, distanceLabel, m0Label, m1Label, m2Label, m3Label, m4Label;
Textfield inputField;
Button scanButton, measureButton, connectButton, disconnectButton, moveW, moveA, moveS, moveD;
Slider angleSlider;
ScrollableList portsList;
String version = "1.0";

public void guiSetup() {
  cp5 = new ControlP5(this);
  /*default ControlP5 colors
  borders:        0074D9
  fill/inactive:  002D5A
  active:         00AAFF*/
  
  //serial output: 70-12=58 chars wide
  outputArea = cp5.addTextarea("serialOutput")
                  .setPosition(20, 20)
                  .setSize(480, 440)
                  .setFont(createFont("consolas", 12))
                  .setLineHeight(14)
                  .setColor(#FFFFFF)
                  .setColorBackground(#444444)
                  .setScrollBackground(#222222);
  
  //serial input
  inputField = cp5.addTextfield("serialInput")
                  .setPosition(20, 440)
                  .setSize(480, 20)
                  .setAutoClear(true)
                  .setLabel("")
                  .setFocus(true);
  
  //distance output
  distanceArea = cp5.addTextarea("distance")
                    .setPosition(585, 282)
                    .setSize(130, 16)
                    .setFont(createFont("consolas", 12))
                    .hideScrollbar()
                    .setColor(#FFFFFF)
                    .setColorBackground(#444444)
                    .setScrollBackground(#222222)
                    .setText("0");
  
  //labels
  angle0Label = cp5.addTextlabel("angle0")
                   .setText("0")
                   .setPosition(505, 215)
                   .setColorValue(255);
  
  angle180Label = cp5.addTextlabel("angle180")
                     .setText("180")
                     .setPosition(920, 215)
                     .setColorValue(255);
  
  distanceLabel = cp5.addTextlabel("distanceLabel")
                     .setText(" DISTANCE")
                     .setPosition(525, 285)
                     .setColorValue(255);
                     
  m0Label = cp5.addTextlabel("m0Label")
               .setText("2 m")
               .setPosition(510, 242)
               .setColorValue(255);
  
  m1Label = cp5.addTextlabel("m1Label")
               .setText("1 m")
               .setPosition(610, 242)
               .setColorValue(255);
  
  m2Label = cp5.addTextlabel("m2Label")
               .setText("0 m")
               .setPosition(710, 242)
               .setColorValue(255);
  
  m3Label = cp5.addTextlabel("m3Label")
               .setText("1 m")
               .setPosition(810, 242)
               .setColorValue(255);
  
  m4Label = cp5.addTextlabel("m4Label")
               .setText("2 m")
               .setPosition(910, 242)
               .setColorValue(255);
  
  //scan
  scanButton = cp5.addButton("scanButton")
                  .setPosition(825, 280)
                  .setSize(90, 20)
                  .setLabel("scan");
  
  //sensor turning
  angleSlider = cp5.addSlider("angleSlider")
                   .setPosition(525, 255)
                   .setSize(390, 10)
                   .setRange(0, 180)
                   .setLabel("")
                   .setSliderMode(Slider.FLEXIBLE)
                   .setNumberOfTickMarks(181)
                   .showTickMarks(false)
                   .snapToTickMarks(true)
                   .setValue(92);
  angleSlider.getValueLabel().align(ControlP5.CENTER, ControlP5.TOP_OUTSIDE).setPaddingY(40);
  
  //measure distance
  measureButton = cp5.addButton("measureButton")
                     .setPosition(725, 280)
                     .setSize(90, 20)
                     .setLabel("measure");
  
  //COM port selection
  portsList = cp5.addScrollableList("portsList")
                 .setPosition(525, 310)
                 .setSize(190, 100)
                 .setBarHeight(20)
                 .setItemHeight(20)
                 .setItems(comList)
                 .setLabel(" COM ports")
                 .close();
  
  //COM port connect
  connectButton = cp5.addButton("connectButton")
                     .setPosition(725, 310)
                     .setSize(90, 20)
                     .setLabel("connect");
  
  //COM port disconnect
  disconnectButton = cp5.addButton("disconnectButton")
                        .setPosition(825, 310)
                        .setSize(90, 20)
                        .setLabel("disconnect");
  
  //movement buttons
  moveW = cp5.addButton("W")
             .setPosition(800, 350)
             .setSize(40, 40);
  moveW.getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  
  moveA = cp5.addButton("A")
             .setPosition(750, 400)
             .setSize(40, 40);
  moveA.getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  
  moveS = cp5.addButton("S")
             .setPosition(800, 400)
             .setSize(40, 40);
  moveS.getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
  
  moveD = cp5.addButton("D")
             .setPosition(850, 400)
             .setSize(40, 40);
  moveD.getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);
}

/*void output(String s) {
  outputArea.append(trim(s) + "\n");
}*/

void output(int code1, int code2, String s) {
  /*in packet structure:  char#   desc.
                            0      code1
                            1      code2
                            2->n   code-dependent payload
                          reserved codes: 40  continous scanning data
                                          41  single distance measurement
   out packet structure:  char#   desc.
                            0      command
                            1      payload*/
  /*commands:  t[deg] turn sensor to [deg]
               m      measure distance
               n      scan
               w      walk forward
               a      turn left
               s      walk backward
               d      turn right*/
  //codes               0          1          2          3          4          5          6          7          8          9          10         11
  String[] codes = {"       ", "[ECHO] ", "[SETUP]", "[TRACE]", "[SR04] ", "[PWM]  ", "[ERROR]", "[WARN] ", "[INFO] ", "     > ", "[CMD]  ", "[APP]  "};
  
  String[] splitted = split(s, ' ');
  int i = 0;
  int remaining = s.length();
  if(remaining < 58) {
    outputArea.append(codes[code1] + " " + codes[code2] + " " + trim(s) + "\n");
  } else {
    while(remaining > 58) {
      String line;
      if(i == 0) {
        line = codes[code1] + " " + codes[code2];
      } else {
        line = "                ";
      }
      while(line.length() <= 58) {
        line += " " + splitted[i++];
      }
      outputArea.append(line + "\n");
      remaining -= line.length() - 15;
    }
    String end = s.substring(s.length()-remaining);
    outputArea.append("                " + end + "\n");
  }
}

void CP5AddOns() {
  //auto-update available COM ports on every redraw
  comList = Serial.list();
  cp5.get(ScrollableList.class, "portsList").setItems(comList);
  
  //ultrasonic scan
  stroke(255);
  
  line(720, 230, 720-210, 230);
  line(720, 230, 720-182, 230-105);
  line(720, 230, 720-105, 230-182);
  line(720, 230, 720, 20);
  line(720, 230, 720+105, 230-182);
  line(720, 230, 720+182, 230-105);
  line(720, 230, 720+210, 230);
  
  int lineHeight = 2;
  for (int i=-200; i<=200; i+=10) {
    if(i % 100 == 0) {
      lineHeight = 8;
    } else if(i % 50 == 0) {
      lineHeight = 4;
    } else {
      lineHeight = 2;
    }
    line(720+i, 230, 720+i, 230+lineHeight);
  }
  
  fill(#002D5A);
  stroke(#0074D9);
  arc(720, 230, 400, 400, PI, TWO_PI, CHORD);
  arc(720, 230, 300, 300, PI, TWO_PI, CHORD);
  arc(720, 230, 200, 200, PI, TWO_PI, CHORD);
  arc(720, 230, 100, 100, PI, TWO_PI, CHORD);
  fill(#FFFFFF);
  stroke(#FFFFFF);
  arc(720, 230, 8, 8, PI, TWO_PI, CHORD);
  
  if(firstScan) {
    drawScan(180);
  }
  
  //distance output
  fill(#444444);
  noStroke();
  rect(583, 280, 132, 20);
  
  fill(#FFFFFF);
  stroke(#000000);
}

void drawScanLine(int distance, int angle) {
  stroke(#0074D9);
  line(720-distance*cos(angle*DEG_TO_RAD), 230-distance*sin(angle*DEG_TO_RAD), 720-199*cos(angle*DEG_TO_RAD), 230-199*sin(angle*DEG_TO_RAD));
}

void drawRefreshLine(int angle) {
  stroke(#FFFFFF);
  line(720, 230, 720-199*cos(angle*DEG_TO_RAD), 230-199*sin(angle*DEG_TO_RAD));
}

void drawScan(int i) {
  for(int j=0; j<i; j++) {
    drawScanLine(scanResults[j], j);
  }
}
