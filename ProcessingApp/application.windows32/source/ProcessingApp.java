import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import controlP5.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class ProcessingApp extends PApplet {

  

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

boolean useDropDownLists = true; // Set if you want to use the dropdownlist or not
byte defaultComPort = 0;
int defaultBaudrate = 115200;

//Dropdown lists
DropdownList COMports; // Define the variable ports as a Dropdownlist.
Serial serial; // Define the variable port as a Serial object.
int portNumber = -1; // The dropdown list will return a float value, which we will connvert into an int. We will use this int for that.

boolean connectedSerial;
boolean aborted;
boolean initialized; // True if connected to the device

boolean upPressed;
boolean downPressed;
boolean leftPressed;
boolean rightPressed;
boolean sendData;

String stringGyro;
String stringAcc;
String stringKalman;

float[] acc = new float[600];
float[] gyro = new float[600];
float[] kalman = new float[600];

public void setup() {
  controlP5 = new ControlP5(this);
  size(937, 330);

  f = loadFont("EuphemiaUCAS-Bold-30.vlw");
  textFont(f, 30);

  /* For remote control */
  controlP5.addButton("Up", 0, 337/2-20, 70, 40, 20);
  controlP5.addButton("Down", 0, 337/2-20, 92, 40, 20);
  controlP5.addButton("Left", 0, 337/2-62, 92, 40, 20);
  controlP5.addButton("Right", 0, 337/2+22, 92, 40, 20);  

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
  
  for (int i=0;i<600;i++) { // center all variables    
    gyro[i] = height/2;
    acc[i] = height/2; 
    kalman[i] = height/2;
  }

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
      serial.write("GP;"); // Go
      delay(10);
      serial.write("GB;"); // Send graph data
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
        serial.write("GP;"); // Go
        delay(10);
        serial.write("GB;"); // Send graph data
      }
    }
  }*/
}

public void draw() {
  /* Draw Graph */
  background(255); // white
  for (int i = 0;i<=width/10;i++) {      
    stroke(200); // gray
    line((-frameCount%10)+i*10, 0, (-frameCount%10)+i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(0); // black
  for (int i = 1; i <= 3; i++)
    line(337, height/4*i, width, height/4*i); // Draw line, indicating 90 deg, 180 deg, and 270 deg

  convert();
  drawAxis();    

  /* Remote contol */
  //background(0);
  fill(0);
  stroke(0);
  rect(0, 0, 337, height);
  fill(0, 102, 153);
  textSize(25); 
  textAlign(CENTER); 
  text("Press buttons to steer", 337/2, 55);

  /* Set PID value etc. */
  fill(0, 102, 153);
  textSize(30); 
  textAlign(CENTER); 
  text("Set PID Values:", 337/2, 155);
  text("Current PID Values:", 337/2, 250);

  fill(255, 255, 255);
  textSize(10);  
  textAlign(LEFT);
  text("P: " + stringP + " I: " + stringI +  " D: " + stringD + " TargetAngle: " + stringTargetAngle, 10, height-50);

  if (sendData) { // Data is send as a x,y-coordinates
    if (upPressed) {
      if (leftPressed)
        serial.write("J,-0.7,0.7;"); // Forward left
      else if (rightPressed)
        serial.write("J,0.7,0.7;"); // Forward right
      else
        serial.write("J,0,0.7;"); // Forward
    }
    else if (downPressed) {
      if (leftPressed)
        serial.write("J,-0.7,-0.7;"); // Backward left
      else if (rightPressed)
        serial.write("J,0.7,-0.7;"); // Backward right
      else
        serial.write("J,0,-0.7;"); // Backward
    } 
    else if (leftPressed)
      serial.write("J,-0.7,0;"); // Left
    else if (rightPressed)
      serial.write("J,0.7,0;"); // Right
    else {
      serial.write("S;");
      println("Stop");
    }
    sendData = false;
  }
}
public void Abort(int theValue) {
  if (connectedSerial) {
    serial.write("A;");
    println("Abort");
    aborted = true;
  } else
    println("Establish a serial connection first!");
}
public void Continue(int theValue) {
  if (connectedSerial) {
    serial.write("C");
    println("Continue");
    aborted = false;
  } else
    println("Establish a serial connection first!");
}
public void Submit(int theValue) {
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
    serial.write("GP;"); // Send values back to application
    delay(10);
  }
  else
    println("Establish a serial connection first!");
}
public void Clear(int theValue) {
  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();
}
public void StoreValues(int theValue) {
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
public void serialEvent(Serial serial) {
  String[] input = trim(split(serial.readString(), ','));
  for (int i = 0; i<input.length;i++)
    println("Number: " + i + " Input: " + input[i]); // For debugging

  if (input[0].equals("P")) {    
    if (input[1].length() > 5)
      stringP = input[1].substring(0, 5);
    else
      stringP = input[1];
      
    if (input[2].length() > 5)
      stringI = input[2].substring(0, 5);
    else
      stringI = input[2];
      
    if (input[3].length() > 5)
      stringD = input[3].substring(0, 5);
    else
      stringD = input[3];
      
    if (input[4].length() > 6)
      stringTargetAngle = input[4].substring(0, 6);
    else
      stringTargetAngle = input[4];
  }
  if(input[0].equals("V")) {
    stringAcc = input[1];
    stringGyro = input[2];
    stringKalman = input[3];
    
    /*print(stringAcc);
    print(stringGyro);
    print(stringKalman);*/
  }
  serial.clear();  // Empty the buffer  
}
public void keyPressed() {
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
public void keyReleased() {
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
public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    if (theEvent.group().name() == "COMPort")     
      portNumber = PApplet.parseInt(theEvent.group().value()); //Since the list returns a float, we need to convert it to an int. For that we us the int() function.
  }
}
public void Connect(int value) {     
  if (connectedSerial) // Disconnect existing connection
    Disconnect(0);
  if (portNumber != -1 && !connectedSerial) { // Check if com port and baudrate is set and if there is not already a connection established
    println("ConnectSerial");    
    try {
      serial = new Serial(this, Serial.list()[portNumber], 115200);
    } catch (Exception e) {
      //e.printStackTrace();
      println("Couldn't open serial port");
    }  
    if (serial != null) {      
      serial.bufferUntil('\n');
      connectedSerial = true;
      delay(100);
      serial.write("GP;"); // Go
      delay(10);
      serial.write("GB;"); // Request graph data
    }
  }
  else if (portNumber == -1)
    println("Select COM Port first!");
  else if (connectedSerial)
    println("Already connected to a port!");
}
public void Disconnect(int value) {
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
//convert all axis
int minAngle = 0;
int maxAngle = 360;

public void convert() {   
  /* convert the gyro x-axis */
  if (stringGyro != null) {
    // trim off any whitespace:
    stringGyro = trim(stringGyro);
    // convert to an float and map to the screen height, then save in buffer:    
    gyro[gyro.length-1] = map(PApplet.parseFloat(stringGyro), minAngle, maxAngle, 0, height);
  }  
  /* convert the accelerometer y-axis */
  if (stringAcc != null) {
    // trim off any whitespace:
    stringAcc = trim(stringAcc);
    // convert to an float and map to the screen height, then save in buffer:        
    acc[acc.length-1] = map(PApplet.parseFloat(stringAcc), minAngle, maxAngle, 0, height);
  }
  /* convert the kalman filter y-axis */
  if (stringKalman != null) {
    // trim off any whitespace:
    stringKalman = trim(stringKalman);
    // convert to an float and map to the screen height, then save in buffer:    
    kalman[kalman.length-1] = map(PApplet.parseFloat(stringKalman), minAngle, maxAngle, 0, height);
  }
}
public void drawAxis() { 
  /* draw acceleromter x-axis */
  noFill();
  stroke(255,0,0); //red
  // redraw everything
  beginShape();
  for(int i = 0; i<acc.length;i++)
    vertex(i+337,height-acc[i]);  
  endShape();
  // put all data one array back
  for(int i = 1; i<acc.length;i++)
    acc[i-1] = acc[i];  
   
  /* draw gyro x-axis */
  noFill();
  stroke(0,255,0); // green
  // redraw everything
  beginShape();
  for(int i = 0; i<gyro.length;i++)
    vertex(i+337,height-gyro[i]);
  endShape();
  // put all data one array back
  for(int i = 1; i<gyro.length;i++)
    gyro[i-1] = gyro[i];
    
  /* draw kalman filter x-axis */
  noFill();
  stroke(0,0,255); // blue  
  // redraw everything
  beginShape();
  for(int i = 0; i<kalman.length;i++)
    vertex(i+337,height-kalman[i]);  
  endShape();
  // put all data one array back
  for(int i = 1; i<kalman.length;i++)
    kalman[i-1] = kalman[i];
}
public void initDropdownlist() {
  COMports = controlP5.addDropdownList("COMPort", 10, 20, 210, 200); // Make a dropdown list with all comports
  customize(COMports); // Setup the dropdownlist by using a function

  controlP5.addButton("Connect", 0, 225, 3, 45, 15);
  controlP5.addButton("Disconnect", 0, 275, 3, 52, 15);
}

public void customize(DropdownList ddl) {
  ddl.setBackgroundColor(color(200));//Set the background color of the line between values
  ddl.setItemHeight(20);//Set the height of each item when the list is opened.
  ddl.setBarHeight(15);//Set the height of the bar itself.

  ddl.captionLabel().style().marginTop = 3;//Set the top margin of the lable.  
  ddl.captionLabel().style().marginLeft = 3;//Set the left margin of the lable.  
  ddl.valueLabel().style().marginTop = 3;//Set the top margin of the value selected.
  
  if (ddl.name() == "COMPort") {
    ddl.captionLabel().set("Select COM port");//Set the lable of the bar when nothing is selected. 
    //Now well add the ports to the list, we use a for loop for that.
    for (int i=0; i<Serial.list().length; i++)    
      ddl.addItem(Serial.list()[i], i);//This is the line doing the actual adding of items, we use the current loop we are in to determin what place in the char array to access and what item number to add it as.
  }
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "ProcessingApp" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
