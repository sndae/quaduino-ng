// ============================================
// =          Q  U  A  D  U  I  N  O          =
// =  An Arduino based Quadcopter Controller  =
// =  Copyright (c) 2008 Paul René Jørgensen  =
// ============================================
// = http://quaduino.org | http://paulrene.no =
// ============================================
//
// Algorithm
//
// Calibrate Radio, Gyros and accelerometers
// loop:
//   Read Gyros
//     Use current value for PD
//     Sum up Gyro values for I
//   Read Accelerometers every 20ms
//     Calculate correction for I
//   Read Radio every 20ms
//     Subtract the radio signal from the gyro PD values
//   Feed everything into PID
//   Take PID commands and mix with Radio throttle for each motor
//   goto loop
//
//
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareServo.h>
#include <stdlib.h>
#include <math.h>
#include <ServoDecode.h>

//#define debug

// RC command states
#define RC_NOT_SYNCHED 0
#define RC_ACQUIRING 1
#define RC_READY 2
#define RC_IN_FAILSAFE 3

// Radio Channel data (Roll, Pitch, Throttle, Yaw, Gear, Aux)
int rcValue[] = { 0, 0, 0, 0, 0, 0};
int rcZero[] = { 0, 0, 0, 0, 0, 0};
boolean speccy = true;

// Gyro data - Order: PITCH, ROLL, YAW
unsigned int gyroZero[] = { 0, 0, 0 };
int gyroRateOld[] = { 0, 0, 0};
int gyroRate[] = { 0, 0, 0 };
int gyroSum[] = { 0, 0, 0 };

// Accel data
#define accelUpDownMix -200
long accelZero[] = { 0, 0, 0 };
long accelValue[] = { 0, 0, 0 };
int accelSum[] = { 0, 0, 0 };
int accelAltStab = 0;


// Motors
int motor[] = { 0, 0, 0, 0 };

// PID
//int pGain[] = { 20, 20, 20 };
//int iGain[] = { 0, 0, 0 };
//int dGain[] = { -15, -15, -15 };
int pidCmd[] = { 0, 0, 0 };

// State
boolean flying = false;

// Temp variables
unsigned long tempTime;
int n, i;

// Timing
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned int deltaTime = 0;

void setup() {
  Serial.begin(57600);
  setupRadio();
  setupMotors();
  setupAccel();
  calibrateGyros();
  previousTime = millis();
}

int loopCount = 0;

void loop() {
  // Measure loop rate
  currentTime = millis();
  deltaTime = (int) (currentTime - previousTime);
  previousTime = currentTime;
  
  updateRadio();
  updateGyros();
  updateAccels();
  updatePID();
  updateMotors();

  SoftwareServo::refresh();
  
#ifdef debug
  if(loopCount%20==0) {
    for(n=0;n<6;n++) {
      if(n==0 || n==1 || n==3) {
        Serial.print(rcValue[n]+1500);
      } else {
        Serial.print(rcValue[n]+rcZero[n]);
      }
      Serial.print(":");
    }
    Serial.print(":");
    for(n=0;n<4;n++) {
      Serial.print(motor[n]);
      Serial.print(":");
    }
    for(n=0;n<3;n++) {
      Serial.print(gyroRate[n]);
      Serial.print(":");
      Serial.print(gyroSum[n]);
      Serial.print(":");
      Serial.print(accelValue[n]);
      Serial.print(":");
    }
    Serial.print(accelAltStab);
    Serial.print(":");
    Serial.print(flying?"flying":"ground");
    Serial.print(":");
    Serial.println(deltaTime);
  }
#endif
  
  loopCount++;
}

void wait(int ms) {
  tempTime = millis();
  while(millis()-tempTime<ms) {
    if(!flying) {
      processSerial();
    }
    SoftwareServo::refresh();
  }
}
