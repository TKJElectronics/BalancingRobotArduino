#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#include <stdint.h> // Needed for uint8_t

/* Define CPU frequency in Mhz here if not defined in Makefile - this is set in the Arduino IDE */
#ifndef F_CPU
#define F_CPU 16000000UL  // 16 MHz
#endif

#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used for the PS3 Communication and motor functions */
int lastDirection; // This is used set a new targetPosition
enum steerDirection {
  update,
  forward,
  backward,
  stop,
  left,
  right,
  both,
};

/* These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation 
 I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/ */
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* Left motor */
#define leftPort PORTD
#define leftPortDirection DDRD
#define leftPwmPortDirection DDRB

#define leftA PIND0 // PD0 - pin 0
#define leftB PIND1 // PD1 - pin 1
#define leftPWM PINB1 // PB1 - pin 9 (OC1A)

/* Right motor */
#define rightPort PORTC
#define rightPortDirection DDRC
#define rightPwmPortDirection DDRB

#define rightA PINC4 // PC4 - pin A4
#define rightB PINC5 // PC5 - pin A5
#define rightPWM PINB2 // PB2 - pin 10 (OC1B)

/* Encoders */
#define leftEncoder1 2
#define leftEncoder2 4
#define rightEncoder1 3
#define rightEncoder2 5

volatile long leftCounter = 0;
volatile long rightCounter = 0;

/* IMU */
#define gyroY A0
#define accX A1
#define accY A2
#define accZ A3

bool inverted; // This is used to check which side is lying down

#define buzzer 6 // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

// Zero voltage values for the sensors - [0] = gyroY, [1] = accX, [2] = accY, [3] = accZ
double zeroValues[4];

/* Kalman filter variables and constants */
const float Q_angle = 0.001; // Process noise covariance for the accelerometer - Sw
const float Q_gyro = 0.003; // Process noise covariance for the gyro - Sw
const float R_angle = 0.03; // Measurement noise covariance - Sv

double angle; // The angle output from the Kalman filter
double bias = 0; // The gyro bias calculated by the Kalman filter
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
double dt, y, S;
double K_0, K_1;

// Results
double accYangle;
double gyroYrate;
double pitch;

/* PID variables */
double Kp = 8;
double Ki = 2;
double Kd = 9;
double targetAngle = 90;

double lastError; // Store position error
double iTerm; // Store integral term

/* Used for timing */
unsigned long timer;

#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime;

bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set new target position after breaking

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

uint8_t loopCounter = 0; // Used to update wheel velocity and buzzer beep
long wheelPosition;
long lastWheelPosition;
long wheelVelocity;
long targetPosition;
int zoneA = 4000;
int zoneB = 2000;
double positionScaleA = 250; // one resolution is 464 pulses
double positionScaleB = 500; 
double positionScaleC = 1000;
double velocityScaleMove = 40;
double velocityScaleStop = 30;
double velocityScaleTurning = 35;
#endif
