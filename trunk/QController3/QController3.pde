// ============================================
// =          Q  U  A  D  U  I  N  O          =
// =  An Arduino based Quadcopter Controller  =
// =  Copyright (c) 2008 Paul René Jørgensen  =
// ============================================
// = http://quaduino.org | http://paulrene.no =
// ============================================
//
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareServo.h>
#include <stdlib.h>
#include <math.h>
#include <ServoDecode.h>

// RADIO command states
#define RC_UNSYNCHED 0
#define RC_ACQUIRING 1
#define RC_READY 2
#define RC_FAILSAFE 3

// Misc
#define EXPECTED_LOOP_TIME 5
boolean flying = false;

// Radio Channel data
int rcValue[] = { 0, 0, 0, 0, 0, 0};
int rcZero[] = { 0, 0, 0, 0, 0, 0};

// Gyro data - Order: PITCH, ROLL, YAW
unsigned int gyroZero[] = { 0, 0, 0 };
int gyroRaw[] = { 0, 0, 0 };
int gyroSum[] = { 0, 0, 0 };
int gyroIntegralLimit[] = { 3840, 3840, 1024 };
// Temp and index variables
unsigned long tempTime;
int n, i;

// Loop timing
unsigned long loopStartTime;
unsigned int loopCount = 0;


void setup() {
  Serial.begin(57600);
  setupRadio();
  calibrateGyros();
}


void loop () {
  loopStartTime = millis();

  updateRadio();
  updateGyros();

  flying = true;
  if(flying) {  

  } else {
    calibrateGyros();
    processSerial();
  }
 
  if(loopCount%50==0) {
/*    Serial.print(rcValue[0]);
    Serial.print(":");
    Serial.print(rcValue[1]);
    Serial.print(":");
    Serial.print(rcValue[2]);
    Serial.print(":");
    Serial.print(rcValue[3]);
    Serial.print(":");
    Serial.print(rcValue[4]);
    Serial.print(":");
    Serial.println(rcValue[5]);*/
    for(n=0;n<3;n++) {
      Serial.print(gyroRaw[n]);
      Serial.print(":");
      Serial.print(gyroSum[n]);
      Serial.print(":");
    }
    Serial.println(millis()-loopStartTime);
  }
  tempTime = millis()-loopStartTime;
  if(tempTime<EXPECTED_LOOP_TIME) {
    delay(EXPECTED_LOOP_TIME-tempTime);
  }
  loopCount++;
}

void wait(int ms) {
  tempTime = millis();
  while(millis()-tempTime<ms) {
    processSerial();
  }
}
