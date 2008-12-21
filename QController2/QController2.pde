#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareServo.h>
#include <stdlib.h>
#include <math.h>
#include <ServoDecode.h>

// EEPROM Adresses
#define ACCEL_ZERO_PITCH_ADDRESS 0
#define ACCEL_ZERO_ROLL_ADDRESS 2
#define ACCEL_ZERO_YAW_ADDRESS 4

// Gyro ADC pins
#define PIN_GYRO_PITCH 0
#define PIN_GYRO_ROLL 1
#define PIN_GYRO_YAW 2

// Index into the value arrays
#define INDEX_ROLL 0
#define INDEX_PITCH 1
#define INDEX_YAW 2
#define INDEX_THROTTLE 3

// Index variables
int n, i;

// Value arrays
int GYRO_ZERO[] = {0, 0, 0};
int GYRO_RAW[] = {0, 0, 0};
int ACCEL_ZERO[] = {0, 0, 0};
int ACCEL_RAW[] = {0, 0, 0};

// Timing variables
#define EXPECTED_LOOP_TIME 3
long loopStartTime = 0;
int lastLoopTime = EXPECTED_LOOP_TIME;
int lastLoopUsefulTime = EXPECTED_LOOP_TIME;


void setup() {
  Serial.begin(38400);
  initSensors();
}

void loop() {
  loopStartTime = millis();
  
  updateOrientation(lastLoopTime);

  
  
  
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<EXPECTED_LOOP_TIME) {
    delay(EXPECTED_LOOP_TIME-lastLoopUsefulTime);
  }
  lastLoopTime = millis() - loopStartTime;
}
