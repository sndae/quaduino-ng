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
unsigned int GYRO_ZERO[] = {0, 0, 0};
int GYRO_RAW[] = {0, 0, 0};
float GYRO_ANGLE[] = {0, 0, 0};
float GYRO_BIAS[] = {0, 0, 0};
int ACCEL_ZERO[] = {0, 0, 0};
int ACCEL_ANGLE[] = {0, 0};
int ORIENTATION[] = {0, 0, 0};
int RADIO_VALUE[] = { 0, 0, 0, 0, 0, 0};
int RADIO_ZERO[] = { 0, 0, 0, 0, 0, 0};
int WANTED_ANGLE[] = {0, 0, 0};

// Timing variables
#define EXPECTED_LOOP_TIME 3
unsigned long loopStartTime = 0;
int lastLoopTime = EXPECTED_LOOP_TIME;
int lastLoopUsefulTime = EXPECTED_LOOP_TIME;


void setup() {
  Serial.begin(57600);
  ServoDecode.begin();
  initSensors();
  initMotors();
  checkRadio();
  calibrateRadio();
}

int count = 0;
void loop() {
  loopStartTime = millis();
  
  updateOrientation(lastLoopTime);
  updateRadio();
  decodeMotorCommands();
  updatePID(0.0, 0.0);
  updatePID(0.0, 0.0);
  updatePID(0.0, 0.0);
  
  if(count++>20) {
    Serial.print(RADIO_VALUE[0]);
    Serial.print(":");
    Serial.print(RADIO_VALUE[1]);
    Serial.print(":");
    Serial.print(RADIO_VALUE[2]);
    Serial.print(":");
    Serial.print(RADIO_VALUE[3]);
    Serial.print(":");
    Serial.print(RADIO_VALUE[4]);
    Serial.print(":");
    Serial.print(RADIO_VALUE[5]);
    Serial.print(":");
    Serial.print(int(GYRO_ANGLE[0]));
    Serial.print(":");
    Serial.print(int(GYRO_ANGLE[1]));
    Serial.print(":");
    Serial.print(int(GYRO_ANGLE[2]));
    Serial.print(":");
    Serial.print(ACCEL_ANGLE[0]);
    Serial.print(":");
    Serial.print(ACCEL_ANGLE[1]);
    Serial.print(":");
    Serial.print(ORIENTATION[0]);
    Serial.print(":");
    Serial.print(ORIENTATION[1]);
    Serial.print(":");
    Serial.print(ORIENTATION[2]);
    Serial.print(":");
    Serial.print(WANTED_ANGLE[0]);
    Serial.print(":");
    Serial.print(WANTED_ANGLE[1]);
    Serial.print(":");
    Serial.print(WANTED_ANGLE[2]);
    Serial.print(":");
    Serial.print(ServoDecode.getState());
    Serial.print(":");
    Serial.print(lastLoopTime);
    Serial.print(":");
    Serial.print(millis() - loopStartTime);
    Serial.print(":");
    Serial.println(millis());
    count = 0;
  }
  
  SoftwareServo::refresh();
  
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<EXPECTED_LOOP_TIME) {
    delay(EXPECTED_LOOP_TIME-lastLoopUsefulTime);
  }
  lastLoopTime = millis() - loopStartTime;
}
