/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus
 * This is the algorithm for my balancing robot/segway.
 * It is controlled by a PS3 Controller via bluetooth.
 * The PS3 Bluetooth Library can be found at the following link: https://github.com/TKJElectronics/USB_Host_Shield_2.0
 * For details, see http://blog.tkjelectronics.dk/2012/02/the-balancing-robot/
 */

#include "BalancingRobot.h"
#include <PS3BT.h> // SS is rerouted to 8 and INT is rerouted to 7
USB Usb;
PS3BT BT(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // Also remember to disable DEBUG in "PS3BT.cpp" to save space

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
  TCCR1A = 0; // clear all
  TCCR1B = 0; // clear all
  TCCR1B |= _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);
    
  /* Enable PWM on pin 9 (OC1A) & pin 10 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
  TCCR1A |= _BV(COM1A1); 
  TCCR1A |= _BV(COM1B1);
  setPWM(leftPWM,0); // Turn off pwm on both pins
  setPWM(rightPWM,0);
  
  /* Setup pin for buzzer to beep when finished calibrating */
  pinMode(buzzer,OUTPUT);  
  
  /* Setup IMU Inputs */
  analogReference(EXTERNAL); // Set voltage reference to 3.3V by connecting AREF to 3.3V
  pinMode(gyroY,INPUT);
  pinMode(accX,INPUT);
  pinMode(accY,INPUT);
  pinMode(accZ,INPUT);

  /* Calibrate the gyro and accelerometer relative to ground */
  calibrateSensors();
  
  if (Usb.Init() == -1) // Check if USB Host Shield is working
    while(1); //halt    

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
  if (pitch < 60 || pitch > 120) // Stop if falling or laying down
    stopAndReset();
  else
    PID(targetAngle,targetOffset,turningOffset);

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
    if (restAngle < 80) // Limit rest Angle
      restAngle = 80;
    else if (restAngle > 100)
      restAngle = 100;
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

  if(BT.PS3BTConnected) {
    if(BT.getButton(PS)) {
      steer(stop);
      BT.disconnect();
    } 
    else if(BT.getButton(SELECT)) {
      stopAndReset();
      while(!BT.getButton(START))
        Usb.Task();        
    }
    if((BT.getAnalogHat(LeftHatY) < 117) || (BT.getAnalogHat(RightHatY) < 117) || (BT.getAnalogHat(LeftHatY) > 137) || (BT.getAnalogHat(RightHatY) > 137)) {
      steer(update);
    } else 
      steer(stop);      
  } 
  else if(BT.PS3NavigationBTConnected) {
    if(BT.getButton(PS)) {
      steer(stop);
      BT.disconnect();
    } 
    if(BT.getAnalogHat(LeftHatX) > 200 || BT.getAnalogHat(LeftHatX) < 55 || BT.getAnalogHat(LeftHatY) > 137 || BT.getAnalogHat(LeftHatY) < 117) {
      steer(update);
    } else 
      steer(stop);  
  } 
  else
    steer(stop);    
}

void steer(steerDirection direction) {
  if(BT.PS3BTConnected) {
    if(BT.getAnalogHat(LeftHatY) < 117 && BT.getAnalogHat(RightHatY) < 117) {
        targetOffset = ((double)(232-(BT.getAnalogHat(LeftHatY)+BT.getAnalogHat(RightHatY))))/33.142857143; // 232/7=33.142857143 - convert from 232-0 to 0-7
        steerForward = true;      
      } else if(BT.getAnalogHat(LeftHatY) > 137 && BT.getAnalogHat(RightHatY) > 137) {
        targetOffset = ((double)((BT.getAnalogHat(LeftHatY)+BT.getAnalogHat(RightHatY))-276))/33.428571429; // 234/7=33.428571429 - convert from 276-510 to 0-7
        steerBackward = true;
      }
      if(((int)BT.getAnalogHat(LeftHatY) - (int)BT.getAnalogHat(RightHatY)) > 15) {
        turningOffset = ((double)abs(BT.getAnalogHat(LeftHatY) - BT.getAnalogHat(RightHatY)))/12.75; // 255/20=12,75 - convert from 0-255 to 0-20
        steerLeft = true;        
      } else if (((int)BT.getAnalogHat(RightHatY) - (int)BT.getAnalogHat(LeftHatY)) > 15) {        
        turningOffset = ((double)abs(BT.getAnalogHat(LeftHatY) - BT.getAnalogHat(RightHatY)))/12.75; // 255/20=12,75 - convert from 0-255 to 0-20  
        steerRight = true;      
      }  
  }
  if(BT.PS3NavigationBTConnected) {
    if(BT.getAnalogHat(LeftHatY) < 117) {
      targetOffset = ((double)(116-BT.getAnalogHat(LeftHatY)))/16.571428571; // 116/7=16.571428571 - convert from 116-0 to 0-7
      steerForward = true;
    } else if(BT.getAnalogHat(LeftHatY) > 137) {
      targetOffset = ((double)(BT.getAnalogHat(LeftHatY)-138))/16.714285714; // 117/7=16.714285714 - convert from 138-255 to 0-7
      steerBackward = true;
    }
    if(BT.getAnalogHat(LeftHatX) < 55) {
      turningOffset = ((double)(54-BT.getAnalogHat(LeftHatX)))/2.7; // 54/20=2.7 - convert from 54-0 to 0-20
      steerLeft = true;            
    } else if(BT.getAnalogHat(LeftHatX) > 200) {
      turningOffset = ((double)(BT.getAnalogHat(LeftHatX)-201))/2.7; // 54/20=2.7 - convert from 201-255 to 0-20
      steerRight = true;
    }
  }
  if(direction == stop) {
    steerStop = true;    
    if(lastDirection != stop) { // Set new stop position
      targetPosition = wheelPosition;
      stopped = false;
    }
  }
  lastDirection = direction;
}
void stopAndReset() {
  stopMotor(both);
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
  // (accAdc-accZero)/Sensitivity (In quids) - Sensitivity = 0.33/3.3*1023=102.3
  double accXval = (double)((double)analogRead(accX) - zeroValues[1]) / 102.3;
  double accYval = (double)((double)analogRead(accY) - zeroValues[2]) / 102.3;
  if(inverted)
    accYval--; // -1g when lying at one of the sides
  else
    accYval++; // +1g when lying at the other side
  double accZval = (double)((double)analogRead(accZ) - zeroValues[3]) / 102.3;

  double R = sqrt((accXval*accXval) + (accYval*accYval) + (accZval*accZval)); // Calculate the length of the force vector
  double angleY = acos(accYval / R) * RAD_TO_DEG;
  return angleY;
}
void calibrateSensors() {
  double adc[4] = { 0 };
  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
    adc[0] += analogRead(gyroY);
    adc[1] += analogRead(accX);
    adc[2] += analogRead(accY);
    adc[3] += analogRead(accZ);
    delay(10);
  }
  zeroValues[0] = adc[0] / 100; // Gyro X-axis
  zeroValues[1] = adc[1] / 100; // Accelerometer X-axis
  zeroValues[2] = adc[2] / 100; // Accelerometer Y-axis
  zeroValues[3] = adc[3] / 100; // Accelerometer Z-axis
  
  if(zeroValues[2] > 500) {// Check which side is lying down
    inverted = false;
    angle = 0; // It starts at 0 degress and 180 when facing the other way
  } else {
    inverted = true;
    angle = 180;
  }
  
  digitalWrite(buzzer,HIGH);
  delay(100);  
  digitalWrite(buzzer,LOW);
}
void moveMotor(steerDirection motor, steerDirection direction, double speedRaw) { // speed is a value in percentage 0-100%
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
  else if (motor == both) {
    setPWM(leftPWM,speed); // Left motor pwm
    setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      cbi(leftPort,leftA);
      sbi(leftPort,leftB);            
      cbi(rightPort,rightA);
      sbi(rightPort,rightB);
    } 
    else if (direction == backward) {
      sbi(leftPort,leftA);
      cbi(leftPort,leftB);          
      sbi(rightPort,rightA);
      cbi(rightPort,rightB);
    }
  }
}
void stopMotor(steerDirection motor) {  
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
  else if (motor == both) {
    setPWM(leftPWM,PWMVALUE); // Set high
    sbi(leftPort,leftA);
    sbi(leftPort,leftB);
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
  if(PIND & _BV(PIND4)) // pin 4
    leftCounter++;
  else
    leftCounter--;    
}
void rightEncoder() {
  if(PIND & _BV(PIND5)) // pin 5
    rightCounter++;
  else
    rightCounter--;  
}
long readLeftEncoder() {
  return leftCounter;
}
long readRightEncoder() {
  return rightCounter;
}
