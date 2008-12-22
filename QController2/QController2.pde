#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareServo.h>
#include <stdlib.h>
#include <math.h>
#include <ServoDecode.h>

// Index into the orientation value arrays
#define INDEX_ROLL 0
#define INDEX_PITCH 1
#define INDEX_YAW 2

// Index variables
int n, i;

// Value arrays
int GYRO_ZERO[] = {0, 0, 0};
int GYRO_RAW[] = {0, 0, 0};
float GYRO_ANGLE[] = {0, 0, 0};
float GYRO_BIAS[] = {0, 0, 0};
int ACCEL_ZERO[] = {0, 0, 0};
int ACCEL_RAW[] = {0, 0, 0};
int ACCEL_ANGLE[] = {0, 0};
float ORIENTATION[] = {0, 0, 0};
int RADIO_VALUE[] = { 0, 0, 0, 0, 0, 0};
int RADIO_ZERO[] = { 0, 0, 0, 0, 0, 0};

// Timing variables
#define EXPECTED_LOOP_TIME 3
long loopStartTime = 0;
int lastLoopTime = EXPECTED_LOOP_TIME;
int lastLoopUsefulTime = EXPECTED_LOOP_TIME;


void setup() {
  Serial.begin(38400);
  initSensors();
  initMotors();
  ServoDecode.begin();
  checkRadio();
  calibrateRadio();
}

void loop() {
  loopStartTime = millis();
  
  updateOrientation(lastLoopTime);
  updateRadio();
  decodeMotorCommands();
  
  
  SoftwareServo::refresh();
  
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<EXPECTED_LOOP_TIME) {
    delay(EXPECTED_LOOP_TIME-lastLoopUsefulTime);
  }
  lastLoopTime = millis() - loopStartTime;
}
