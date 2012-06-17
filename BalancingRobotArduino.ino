/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus
 * This is the algorithm for my balancing robot/segway.
 * It is controlled by a PS3 Controller via bluetooth.
 * The PS3 Bluetooth Library can be found at the following link: https://github.com/TKJElectronics/USB_Host_Shield_2.0
 * For details, see http://blog.tkjelectronics.dk/2012/02/the-balancing-robot/
 */

#include "BalancingRobot.h"
#include <PS3BT.h> // SS is rerouted to 8 and INT is rerouted to 7 - see http://www.circuitsathome.com/usb-host-shield-hardware-manual at "5. Interface modifications"
USB Usb;
PS3BT PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // Also remember to uncomment DEBUG in "PS3BT.cpp" to save space

void setup() {
  /* Setup encoders */
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 
  attachInterrupt(0,leftEncoder,RISING); // pin 2
  attachInterrupt(1,rightEncoder,RISING); // pin 3

  /* Setup motor pins to output */
  sbi(leftPwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(rightPwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);  
    
  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);
    
  /* Enable PWM on pin 9 (OC1A) & pin 10 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  setPWM(leftPWM,0); // Turn off pwm on both pins
  setPWM(rightPWM,0);
  
  /* Setup pin for buzzer to beep when finished calibrating */
  pinMode(buzzer,OUTPUT);  
  
  /* Setup IMU Inputs */
  analogReference(EXTERNAL); // Set voltage reference to 3.3V by connecting AREF to 3.3V
  pinMode(gyroY,INPUT);
  pinMode(accY,INPUT);
  pinMode(accZ,INPUT);      
  
  if (Usb.Init() == -1) // Check if USB Host Shield is working
    while(1); // Halt
    
  /* Calibrate the gyro and accelerometer relative to ground */
  calibrateSensors();

  /* Setup timing */
  loopStartTime = micros();
  timer = loopStartTime;
}

void loop() {
  /* Calculate pitch */
  accYangle = getAccY();
  gyroYrate = getGyroYrate();
  // See my guide for more info about calculation the angles and the Kalman filter: http://arduino.cc/forum/index.php/topic,58048.0.htm
  pitch = kalman(accYangle, gyroYrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter
  timer = micros();  

  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < 170 || pitch > 190)) || (!layingDown && (pitch < 135 || pitch > 225))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } 
  else {
    layingDown = false; // It's no longer laying down
    PID(targetAngle,targetOffset,turningOffset);        
  }

  /* Update wheel velocity every 100ms */
  loopCounter++;
  if (loopCounter == 10) {
    loopCounter = 0; // Reset loop counter
    wheelPosition = readLeftEncoder() + readRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    if (abs(wheelVelocity) <= 20 && !stopped) { // Set new targetPosition if breaking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
  
  /* Read the PS3 Controller */
  receivePS3();
  
  /* Use a time fixed loop */
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME)
    delayMicroseconds(STD_LOOP_TIME - lastLoopUsefulTime);
  loopStartTime = micros();
}
void PID(double restAngle, double offset, double turning) {
  /* Steer robot */
  if (steerForward) {
    offset += (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed and scale up when reversing
    restAngle -= offset;
  } 
  else if (steerBackward) {
    offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed and scale up when reversing
    restAngle += offset;
  }
  /* Break */
  else if (steerStop) {
    long positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA) // Inside zone A
      restAngle -= (double)positionError/positionScaleA;
    else if (abs(positionError) > zoneB) // Inside zone B
      restAngle -= (double)positionError/positionScaleB;
    else // Inside zone C
      restAngle -= (double)positionError/positionScaleC;   
    restAngle -= (double)wheelVelocity/velocityScaleStop;
    if (restAngle < 160) // Limit rest Angle
      restAngle = 160;
    else if (restAngle > 200)
      restAngle = 200;
  }
  /* Update PID values */
  double error = (restAngle - pitch);
  double pTerm = Kp * error;
  iTerm += Ki * error;
  double dTerm = Kd * (error - lastError);
  lastError = error;
  double PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  double PIDLeft;
  double PIDRight;
  if (steerLeft) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }
  
  PIDLeft *= 0.95; // compensate for difference in the motors

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, PIDLeft * -1);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, PIDRight * -1);
}
void receivePS3() {
  // Set all false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;

  Usb.Task();  

  if(PS3.PS3Connected) {
    if(PS3.getButton(PS)) {
      steer(stop);
      PS3.disconnect();
    } 
    else if(PS3.getButton(SELECT)) {
      stopAndReset();
      while(!PS3.getButton(START))
        Usb.Task();        
    }
    if((PS3.getAnalogHat(LeftHatY) < 117) || (PS3.getAnalogHat(RightHatY) < 117) || (PS3.getAnalogHat(LeftHatY) > 137) || (PS3.getAnalogHat(RightHatY) > 137)) {
      steer(update);
    } else 
      steer(stop);      
  } 
  else if(PS3.PS3NavigationConnected) {
    if(PS3.getButton(PS)) {
      steer(stop);
      PS3.disconnect();
    } 
    if(PS3.getAnalogHat(LeftHatX) > 200 || PS3.getAnalogHat(LeftHatX) < 55 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117) {
      steer(update);
    } else 
      steer(stop);  
  } 
  else
    steer(stop);    
}
void steer(Command command) {
  if(PS3.PS3Connected) {
    if(PS3.getAnalogHat(LeftHatY) < 117 && PS3.getAnalogHat(RightHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),232,0,7); // Scale from 232-0 to 0-7
        steerForward = true;
      } else if(PS3.getAnalogHat(LeftHatY) > 137 && PS3.getAnalogHat(RightHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),276,510,7); // Scale from 276-510 to 0-7
        steerBackward = true;
      }
      if(((int)PS3.getAnalogHat(LeftHatY) - (int)PS3.getAnalogHat(RightHatY)) > 15) {
        turningOffset = scale(abs(PS3.getAnalogHat(LeftHatY) - PS3.getAnalogHat(RightHatY)),0,255,20); // Scale from 0-255 to 0-20
        steerLeft = true;      
      } else if (((int)PS3.getAnalogHat(RightHatY) - (int)PS3.getAnalogHat(LeftHatY)) > 15) {   
        turningOffset = scale(abs(PS3.getAnalogHat(LeftHatY) - PS3.getAnalogHat(RightHatY)),0,255,20); // Scale from 0-255 to 0-20  
        steerRight = true;  
      }  
  }
  if(PS3.PS3NavigationConnected) {
    if(PS3.getAnalogHat(LeftHatY) < 117) {
      targetOffset = scale(PS3.getAnalogHat(LeftHatY),116,0,7); // Scale from 116-0 to 0-7
      steerForward = true;
    } else if(PS3.getAnalogHat(LeftHatY) > 137) {
      targetOffset = scale(PS3.getAnalogHat(LeftHatY),138,255,7); // Scale from 138-255 to 0-7
      steerBackward = true;
    }
    if(PS3.getAnalogHat(LeftHatX) < 55) {
      turningOffset = scale(PS3.getAnalogHat(LeftHatX),54,0,20); // Scale from 54-0 to 0-20
      steerLeft = true;     
    } else if(PS3.getAnalogHat(LeftHatX) > 200) {
      turningOffset = scale(PS3.getAnalogHat(LeftHatX),201,255,20); // Scale from 201-255 to 0-20
      steerRight = true;
    }
  }
  if(command == stop) {
    steerStop = true;    
    if(lastCommand != stop) { // Set new stop position
      targetPosition = wheelPosition;
      stopped = false;
    }
  }
  lastCommand = command;
}
double scale(double input, double inputMin, double inputMax, double outputMax) { // Like map() just returns a double
  if(inputMin < inputMax)
    return (input-inputMin)/((inputMax-inputMin)/outputMax);              
  else
    return (inputMin-input)/((inputMin-inputMax)/outputMax);    
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);  
  lastError = 0;
  iTerm = 0;
  targetPosition = wheelPosition;
}
double kalman(double newAngle, double newRate, double dtime) {
  // KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418
  // See also http://www.x-firm.com/?page_id=145
  // with slightly modifications by Kristian Lauszus
  // See http://academic.csuohio.edu/simond/courses/eec644/kalman.pdf and 
  // http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information
  dt = dtime / 1000000; // Convert from microseconds to seconds

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  angle += dt * (newRate - bias);

  // Update estimation error covariance - Project the error covariance ahead
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += +Q_gyro * dt;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  // Calculate angle and resting rate - Update estimate with measurement zk
  y = newAngle - angle;
  angle += K_0 * y;
  bias += K_1 * y;

  // Calculate estimation error covariance - Update the error covariance
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return angle;
}
double getGyroYrate() {
  // (gyroAdc-gyroZero)/Sensitivity (In quids) - Sensitivity = 0.00333/3.3*1023=1.0323
  double gyroRate = -((double)((double)analogRead(gyroY) - zeroValues[0]) / 1.0323);
  return gyroRate;
}
double getAccY() {
  double accYval = ((double)analogRead(accY) - zeroValues[1]);  
  double accZval = ((double)analogRead(accZ) - zeroValues[2]);
  // Convert to 360 degrees resolution
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We are then convert it to 0 to 2π and then from radians to degrees
  return (atan2(-accYval,-accZval)+PI)*RAD_TO_DEG;
}
void calibrateSensors() {
  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
    zeroValues[0] += analogRead(gyroY);
    zeroValues[1] += analogRead(accY);
    zeroValues[2] += analogRead(accZ);
    delay(10);
  }
  zeroValues[0] /= 100; // Gyro X-axis
  zeroValues[1] /= 100; // Accelerometer Y-axis
  zeroValues[2] /= 100; // Accelerometer Z-axis
  
  if(zeroValues[1] > 500) { // Check which side is lying down - 1g is equal to 0.33V or 102.3 quids (0.33/3.3*1023=102.3)
    zeroValues[1] -= 102.3; // -1g when lying at one of the sides
    angle = 90; // It starts at 90 degress and 270 when facing the other way
  } else {
    zeroValues[1] += 102.3; // +1g when lying at the other side
    angle = 270;
  }
  
  digitalWrite(buzzer,HIGH);
  delay(100);  
  digitalWrite(buzzer,LOW);
}
void moveMotor(Command motor, Command direction, double speedRaw) { // speed is a value in percentage 0-100%
  if(speedRaw > 100)
    speedRaw = 100;
  int speed = speedRaw*((double)PWMVALUE)/100; // Scale from 100 to PWMVALUE
  if (motor == left) {
    setPWM(leftPWM,speed); // Left motor pwm
    if (direction == forward) {
      cbi(leftPort,leftA);
      sbi(leftPort,leftB);
    } 
    else if (direction == backward) {
      sbi(leftPort,leftA);
      cbi(leftPort,leftB);
    }
  } 
  else if (motor == right) {
    setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      cbi(rightPort,rightA);
      sbi(rightPort,rightB);
    } 
    else if (direction == backward) {
      sbi(rightPort,rightA);
      cbi(rightPort,rightB);
    }
  }
}
void stopMotor(Command motor) {  
  if (motor == left) {
    setPWM(leftPWM,PWMVALUE); // Set high
    sbi(leftPort,leftA);
    sbi(leftPort,leftB);
  } 
  else if (motor == right) {
    setPWM(rightPWM,PWMVALUE); // Set high
    sbi(rightPort,rightA);
    sbi(rightPort,rightB);
  }
}

void setPWM(uint8_t pin, int dutyCycle) { // dutyCycle is a value between 0-ICR
  if(pin == leftPWM) {
    OCR1AH = (dutyCycle >> 8); 
    OCR1AL = (dutyCycle & 0xFF);
  } else if (pin == rightPWM) {
    OCR1BH = (dutyCycle >> 8);
    OCR1BL = (dutyCycle & 0xFF);    
  }
}

/* Interrupt routine and encoder read functions - I read using the port registers for faster processing */
void leftEncoder() { 
  if(PIND & _BV(PIND4)) // read pin 4
    leftCounter++;
  else
    leftCounter--;    
}
void rightEncoder() {
  if(PIND & _BV(PIND5)) // read pin 5
    rightCounter++;
  else
    rightCounter--;  
}
long readLeftEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}
long readRightEncoder() {
  return rightCounter;
}
