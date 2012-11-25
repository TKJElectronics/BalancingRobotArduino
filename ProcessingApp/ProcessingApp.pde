import processing.serial.*;  
import controlP5.*;
ControlP5 controlP5;

Textfield P;
Textfield I;
Textfield D;
Textfield targetAngle;

String stringP = "";
String stringI = "";
String stringD = "";
String stringTargetAngle = "";

PFont f;
//PImage redButton;
//PImage greenButton;

boolean useDropDownLists = true; // Set if you want to use the dropdownlist or not
byte defaultComPort = 0;
int defaultBaudrate = 115200;

//Dropdown lists
DropdownList COMports; // Define the variable ports as a Dropdownlist.
Serial serial; // Define the variable port as a Serial object.
int portNumber = -1; // The dropdown list will return a float value, which we will connvert into an int. We will use this int for that.

DropdownList baudrate;
int selectedBaudrate = -1; // Used to indicate which baudrate has been selected
String[] baudrates = {
  "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200" // Supported baudrates
};
boolean connectedSerial;
boolean aborted;
boolean initialized; // True if connected to the device

boolean upPressed;
boolean downPressed;
boolean leftPressed;
boolean rightPressed;
boolean sendData;

void setup() {
  controlP5 = new ControlP5(this);
  size(337, 330);

  //redButton = loadImage("Red_Button.gif");
  //greenButton = loadImage("Green_Button.gif");

  f = loadFont("EuphemiaUCAS-Bold-30.vlw");
  textFont(f, 30);

  /* For remote control */
  controlP5.addButton("Up", 0, width/2-20, 70, 40, 20);
  controlP5.addButton("Down", 0, width/2-20, 92, 40, 20);
  controlP5.addButton("Left", 0, width/2-62, 92, 40, 20);
  controlP5.addButton("Right", 0, width/2+22, 92, 40, 20);  

  /* For setting the PID values etc. */
  P = controlP5.addTextfield("P", 10, 165, 35, 20);
  P.setFocus(true);
  I = controlP5.addTextfield("I", 50, 165, 35, 20);
  D = controlP5.addTextfield("D", 90, 165, 35, 20);
  targetAngle = controlP5.addTextfield("TargetAngle", 130, 165, 35, 20);
  
  P.setInputFilter(ControlP5.FLOAT);
  I.setInputFilter(ControlP5.FLOAT);
  D.setInputFilter(ControlP5.FLOAT);
  targetAngle.setInputFilter(ControlP5.FLOAT);

  P.setAutoClear(false);
  I.setAutoClear(false);
  D.setAutoClear(false);
  targetAngle.setAutoClear(false);

  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();

  controlP5.addButton("Submit", 0, 202, 165, 60, 20);
  controlP5.addButton("Clear", 0, 267, 165, 60, 20);

  controlP5.addButton("Abort", 0, 10, 300, 40, 20);
  controlP5.addButton("Continue", 0, 55, 300, 50, 20);
  controlP5.addButton("StoreValues", 0, 267, 300, 60, 20);

  //println(Serial.list()); // Used for debugging
  if (useDropDownLists)
    initDropdownlist();
  else { // if useDropDownLists is false, it will connect automatically at startup
    try {
      serial = new Serial(this, Serial.list()[defaultComPort], defaultBaudrate);
    } catch (Exception e) {
      //e.printStackTrace();
      println("Couldn't open serial port");
    }
    if (serial != null) {
      serial.bufferUntil('\n');
      connectedSerial = true;
      delay(100);
      serial.write("G;"); // Go
    }
  }/*
  for (int i=0; i<Serial.list().length; i++) { // Automaticly connect to the Balancing Robot on Mac OS X and Linux 
    if (Serial.list()[i].indexOf("tty.BalancingRobot") != -1) {
      println(Serial.list()[i]);
      try {
        serial = new Serial(this, Serial.list()[i], 115200);
      } catch (Exception e) {
        //e.printStackTrace();
        println("Couldn't open serial port");
      }            
      if (serial != null) {
        serial.bufferUntil('\n');
        connectedSerial = true;
        delay(100);
        serial.write("G;"); // Go
      }
    }
  }*/
}

void draw() {
  background(0);

  /* Remote contol */
  fill(0, 102, 153);
  textSize(25); 
  textAlign(CENTER); 
  text("Press buttons to steer", width/2, 55);

  /* Set PID value etc. */
  fill(0, 102, 153);
  textSize(30); 
  textAlign(CENTER); 
  text("Set PID Values:", width/2, 155);
  text("Current PID Values:", width/2, 250);

  fill(255, 255, 255);
  textSize(10);  
  textAlign(LEFT);
  text("P: " + stringP + " I: " + stringI +  " D: " + stringD + " TargetAngle: " + stringTargetAngle, 10, 275);

  if (sendData) {
    if (upPressed) {
      if (leftPressed)
        serial.write("FL;");
      else if (rightPressed)
        serial.write("FR;");
      else
        serial.write("F;");
    }
    else if (downPressed) {
      if (leftPressed)
        serial.write("BL;");
      else if (rightPressed)
        serial.write("BR;");
      else
        serial.write("B;");
    } 
    else if (leftPressed)
      serial.write("L;");
    else if (rightPressed)
      serial.write("R;");
    else {
      serial.write("S;");
      println("Stop");
    }
    sendData = false;
  }

  /* Set images */
  /*if (initialized)
   image(greenButton, 300, 250, 30, 30);
   else
   image(redButton, 300, 250, 30, 30);*/
}
void Abort(int theValue) {
  if (connectedSerial) {
    serial.write("A;");
    println("Abort");
    aborted = true;
  } else
    println("Establish a serial connection first!");
}
void Continue(int theValue) {
  if (connectedSerial) {
    serial.write("C");
    println("Continue");
    aborted = false;
  } else
    println("Establish a serial connection first!");
}
void Submit(int theValue) {
  if (connectedSerial) {    
    println("PID values: " + P.getText() + " " + I.getText() + " " + D.getText() +  " TargetAnlge: " + targetAngle.getText());
    String output1 = "P," + P.getText() + ';';
    String output2 = "I," + I.getText() + ';';
    String output3 = "D," + D.getText() + ';';
    String output4 = "T," + targetAngle.getText() + ';';

    serial.write(output1);
    delay(10);
    serial.write(output2);
    delay(10);
    serial.write(output3);
    delay(10);
    serial.write(output4);
    delay(10);    
    serial.write("G;"); // Send values back to application
    delay(10);
  }
  else
    println("Establish a serial connection first!");
}
void Clear(int theValue) {
  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();
}
void StoreValues(int theValue) {
  //Don't set the text if the string is empty or it will crash
  if (stringP != null)
    P.setText(stringP);
  if (stringI != null)
    I.setText(stringI);
  if (stringD != null)
    D.setText(stringD);
  if (stringTargetAngle != null)
    targetAngle.setText(stringTargetAngle);
}
void serialEvent(Serial serial) {
  String[] input = trim(split(serial.readString(), ','));
  for (int i = 0; i<input.length;i++)
    println("Number: " + i + " Input: " + input[i]); // For debugging

  if (input[0].equals("P")) {    
    if (input[1].length() > 5)
      stringP = input[1].substring(0, 5);
    else
      stringP = input[1];
  } else if (input[0].equals("I")) {
    if (input[1].length() > 5)
      stringI = input[1].substring(0, 5);
    else
      stringI = input[1];
  } else if (input[0].equals("D")) {
    if (input[1].length() > 5)
      stringD = input[1].substring(0, 5);
    else
      stringD = input[1];
  } else if (input[0].equals("T")) {
    if (input[1].length() > 6)
      stringTargetAngle = input[1].substring(0, 6);
    else
      stringTargetAngle = input[1];
  }  
  serial.clear();  // Empty the buffer
}
void keyPressed() {
  if (key == 's' || key == 'S')
    StoreValues(0);
  if (key == TAB) { //'\t'
    if (P.isFocus()) {
      P.setFocus(false);
      I.setFocus(true);
    } else if (I.isFocus()) {
      I.setFocus(false);
      D.setFocus(true);
    } else if (D.isFocus()) {
      D.setFocus(false);
      targetAngle.setFocus(true);
    } else if (targetAngle.isFocus()) {
      targetAngle.setFocus(false);
      P.setFocus(true);
    } else
      P.setFocus(true);
  }
  else if (key == ENTER) // '\n'
    Submit(0);
  else if (key == ESC) {
    if (aborted)
      Continue(0);
    else
      Abort(0);      
    key = 0; // Disable Processing from quiting when pressing ESC
  } 
  else if (key == CODED) { 
    if (connectedSerial) {
      if (!P.isFocus() && !I.isFocus() && !D.isFocus() && !targetAngle.isFocus()) { 
        if (keyCode == LEFT || keyCode == UP || keyCode == DOWN || keyCode == RIGHT) {        
          if (keyCode == LEFT) {  
            leftPressed = true;
            println("Left pressed");
          }
          if (keyCode == UP) {
            upPressed = true;
            println("Forward pressed");
          }
          if (keyCode == DOWN) {
            downPressed = true;
            println("Backward pressed");
          }
          if (keyCode == RIGHT) {
            rightPressed = true;
            println("Right pressed");
          }
          sendData = true;
        }
      }
    }
    else
      println("Establish a serial connection first!");
  }
}
void keyReleased() {
  if (connectedSerial) {
    if (!P.isFocus() && !I.isFocus() && !D.isFocus() && !targetAngle.isFocus()) { 
      if (keyCode == LEFT || keyCode == UP || keyCode == DOWN || keyCode == RIGHT) {        
        if (keyCode == LEFT) {
          leftPressed = false;
          println("Left released");
        }
        if (keyCode == UP) {
          upPressed = false;
          println("Up released");
        }
        if (keyCode == DOWN) {
          downPressed = false;
          println("Down released");
        }
        if (keyCode == RIGHT) {
          rightPressed = false;
          println("Right released");
        }
        sendData = true;
      }
    }
  }
  else
    println("Establish a serial connection first!");
}
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    if (theEvent.group().name() == "COMPort")     
      portNumber = int(theEvent.group().value()); //Since the list returns a float, we need to convert it to an int. For that we us the int() function.    
    else if (theEvent.group().name() == "Baudrate")    
      selectedBaudrate = int(theEvent.group().value());
  }
}
void Connect(int value) {     
  if (connectedSerial) // Disconnect existing connection
    Disconnect(0);
  if (selectedBaudrate != -1 && portNumber != -1 && !connectedSerial) { // Check if com port and baudrate is set and if there is not already a connection established
    println("ConnectSerial");    
    try {
      serial = new Serial(this, Serial.list()[portNumber], Integer.parseInt(baudrates[selectedBaudrate]));
    } catch (Exception e) {
      //e.printStackTrace();
      println("Couldn't open serial port");
    }  
    if (serial != null) {      
      serial.bufferUntil('\n');
      connectedSerial = true;
      delay(100);
      serial.write("G;"); // Go
    }
  }
  else if (portNumber == -1)
    println("Select COM Port first!");
  else if (selectedBaudrate == -1)
    println("Select baudrate first!");
  else if (connectedSerial)
    println("Already connected to a port!");
}
void Disconnect(int value) {
  if (connectedSerial) { //Check if there is a connection established  
    println("DisconnectSerial");
    serial.stop();
    serial.clear(); // Empty the buffer
    connectedSerial = false;
    initialized = false;
  }
  else
    println("Couldn't disconnect");
}
