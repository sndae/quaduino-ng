// =================================================
// =             Q  U  A  D  U  I  N  O            =
// =     An Arduino based Quadcopter Controller    =
// =  Copyright (c) 2008-2009 Paul Ren\u00e9 J\u00f8rgensen  =
// =================================================
// =    http://quaduino.org | http://paulrene.no   =
// =================================================
// = 2009-09                                       = 
//
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareServo.h>
#include <ServoDecode.h>

// GLOBAL DEFINES
#define LEDPIN 13
#define LEDON digitalWrite(LEDPIN, HIGH)
#define LEDOFF digitalWrite(LEDPIN, LOW)
#define RX_ROLL 0
#define RX_PITCH 1
#define RX_YAW 2
#define RX_THROTTLE 3
#define RX_GEAR 4
#define RX_AUX 5
#define SENSOR_PITCH 0
#define SENSOR_ROLL 1
#define SENSOR_YAW 2
#define SENSOR_ZAXIS 2
#define MOTOR_FRONT 0
#define MOTOR_REAR 1
#define MOTOR_LEFT 2
#define MOTOR_RIGHT 3
#define MINDELTA 200
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100

// GLOBAL STRUCTS
#include "WProgram.h"
void setup();
void loop();
void configRead();
int readInt(int address);
void writeInt(int value, int address);
void setupMotors();
void wait(int ms);
void setAllMotors(int cmd);
void updateMotors();
int updatePID(int targetPosition, int currentPosition, struct PID *p);
void zeroIntegralError();
void setupRX();
void updateRX();
void processRXCommands();
int readRX(int channel);
void zeroGyros();
void zeroAccelerometers();
void updateSensors();
int findMode(int *data, int arraySize);
void setupAccel();
int accelRead(int axis, boolean compensate);
void i2cSend(byte address, byte reg, byte value);
int i2cReadAccel(int adr);
void updateSerial();
void repeat(int count, int value);
void comma();
int readIntSerial();
int smooth(int current, int previous, int factor);
struct Timing {
  unsigned long current;
  unsigned long previous;
  unsigned long serial;
  unsigned long control;
  unsigned long sensor;
  unsigned long rx;
  int delta;
} time = { 0, 0, 0, 0, 0, 0, 0};

struct RX {
  int raw[6];
  int value[6]; // Smoothed
  int command[6]; // speccyFactor applied to ROLL, PITCH and YAW
  int factor[6];
  int center[2];
  int speccyFactor;
} rx = {
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 16, 16, 8, 16, 16, 16 }, // Smooth factors in 1/16th (int) = 0-16 maps to 0-100%
  { 1500, 1500 },
  3 // Speccy factor to dumb down the sticks 3/16th = 0.18
};

struct Motor {
  int command[4];
  int axisCommand[3];
  boolean armed;
  int minimum;
} motor = {
  { MINCOMMAND, MINCOMMAND, MINCOMMAND, MINCOMMAND },
  { 0, 0, 0 },
  false,
  1000
};

struct GyroData {
  int zero[3];
  int raw[3];
  int value[3];
  int factor;
} gyro = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 3 };

struct AccelData {
  int zero[3];
  int raw[3];
  int value[3];
  int factor;
} accel = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 3 };

struct PID {
  int P, I, D;
  int lastPosition;
  int integratedError;
} pid[3];

struct Level {
  int rollRatio;
  int pitchRatio;
  int rollCount;
  int pitchCount;
  int rollAdjust;
  int pitchAdjust;
  boolean enabled;
} level = { 0, 0, 0, 0, 0, 0, false };

int windupGuard = 1000;

void setup() {
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);

  configRead();
  setupRX();
  setupMotors();
  setupAccel();
  
  zeroGyros();
//  zeroAccelerometers();

  time.previous = millis();
  LEDON;
}


void loop() {
  // Calculate timing
  time.current = millis();
  time.delta = (int) (time.current - time.previous);
  time.previous = time.current;
  
  // RX LOOP
  if(time.current > (time.rx + 100)) { // 10Hz
    updateRX();
    time.rx = time.current;
  }

  // SENSOR LOOP
  if(time.current > (time.sensor + 2)) {
    updateSensors();
  }
  
  // CONTROL LOOP
  if(time.current > time.control + 2) { // 500Hz
    // Auto level on?
    level.enabled = false;
    if(rx.raw[RX_GEAR] > MIDCOMMAND) {
      if((abs(rx.value[RX_PITCH] - rx.center[RX_PITCH]) <= 50)) {
        if((abs(rx.value[RX_ROLL] - rx.center[RX_ROLL]) <= 50)) {
          level.enabled = true;
        }
      }
    }
    // Calculate auto level
    level.rollCount = constrain(level.rollCount + level.rollRatio/15, -20, 20);
    level.pitchCount = constrain(level.pitchCount + level.pitchRatio/15, -20, 20);
    if(level.rollCount>0) { level.rollCount--; }
    if(level.rollCount<0) { level.rollCount++; }
    if(level.pitchCount>0) { level.pitchCount--; }
    if(level.pitchCount<0) { level.pitchCount++; }
    
    if(level.enabled) {
      level.rollAdjust = level.rollCount;
      level.pitchAdjust = level.pitchCount;
    } else {
      level.rollAdjust = 0;
      level.pitchAdjust = 0;
    }

    motor.axisCommand[RX_ROLL] = updatePID(rx.command[RX_ROLL]-level.rollAdjust, gyro.value[SENSOR_ROLL] + MIDCOMMAND, &pid[RX_ROLL]);
    motor.axisCommand[RX_PITCH] = updatePID(rx.command[RX_PITCH]-level.pitchAdjust, -gyro.value[SENSOR_PITCH] + MIDCOMMAND, &pid[RX_PITCH]);
    motor.axisCommand[RX_YAW] = updatePID(rx.command[RX_YAW], gyro.value[SENSOR_YAW] + MIDCOMMAND, &pid[RX_YAW]);

    if(motor.armed) {
      motor.command[MOTOR_FRONT] = constrain(rx.command[RX_THROTTLE] - motor.axisCommand[RX_PITCH] + motor.axisCommand[RX_YAW], motor.minimum, MAXCOMMAND);
      motor.command[MOTOR_REAR] = constrain(rx.command[RX_THROTTLE] + motor.axisCommand[RX_PITCH] + motor.axisCommand[RX_YAW], motor.minimum, MAXCOMMAND);
      motor.command[MOTOR_RIGHT] = constrain(rx.command[RX_THROTTLE] + motor.axisCommand[RX_ROLL] - motor.axisCommand[RX_YAW], motor.minimum, MAXCOMMAND);
      motor.command[MOTOR_LEFT] = constrain(rx.command[RX_THROTTLE] - motor.axisCommand[RX_ROLL] - motor.axisCommand[RX_YAW], motor.minimum, MAXCOMMAND);
      
      // If throttle in minimum position, don't apply yaw
      if(rx.command[RX_THROTTLE] < MINCHECK) {
        setAllMotors(motor.minimum); 
      }
      
    } else {
      setAllMotors(MINCOMMAND);
    }
    updateMotors();
  }

  // SERIAL PORT LOOP
  if(time.current > (time.serial + 100)) { // 10Hz
    updateSerial();
    time.serial = time.current;
  }
  
  // Update Motor PWM
  SoftwareServo::refresh();
}
// EEPROM storage addresses
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define ACCEL_PITCH_ZERO_ADR 60
#define ACCEL_ROLL_ZERO_ADR 64
#define ACCEL_ZAXIS_ZERO_ADR 68
#define GYRO_ROLL_ZERO_ADR 96
#define GYRO_PITCH_ZERO_ADR 100
#define GYRO_YAW_ZERO_ADR 104
#define PITCH_PGAIN_ADR 124
#define PITCH_IGAIN_ADR 128
#define PITCH_DGAIN_ADR 132

void configRead() {
  accel.zero[SENSOR_ROLL] = readInt(ACCEL_ROLL_ZERO_ADR);
  accel.zero[SENSOR_PITCH] = readInt(ACCEL_PITCH_ZERO_ADR);
  accel.zero[SENSOR_ZAXIS] = readInt(ACCEL_ZAXIS_ZERO_ADR);
  gyro.zero[SENSOR_ROLL] = readInt(GYRO_ROLL_ZERO_ADR);
  gyro.zero[SENSOR_PITCH] = readInt(GYRO_PITCH_ZERO_ADR);
  gyro.zero[SENSOR_YAW] = readInt(GYRO_YAW_ZERO_ADR);
 
  pid[RX_ROLL].P = readInt(PGAIN_ADR);
  pid[RX_ROLL].I = readInt(IGAIN_ADR);
  pid[RX_ROLL].D = readInt(DGAIN_ADR);
  pid[RX_PITCH].P = readInt(PITCH_PGAIN_ADR);
  pid[RX_PITCH].I = readInt(PITCH_IGAIN_ADR);
  pid[RX_PITCH].D = readInt(PITCH_DGAIN_ADR);
  pid[RX_YAW].P = readInt(YAW_PGAIN_ADR);
  pid[RX_YAW].I = readInt(YAW_IGAIN_ADR);
  pid[RX_YAW].D = readInt(YAW_DGAIN_ADR);
  
  windupGuard = readInt(WINDUPGUARD_ADR);
  rx.speccyFactor = readInt(XMITFACTOR_ADR);
  gyro.factor = readInt(GYROSMOOTH_ADR);
  accel.factor = readInt(ACCSMOOTH_ADR);
}

// Utilities for writing and reading from the EEPROM
int readInt(int address) {
  union intStore { byte intByte[2]; int intVal; } intOut;
  for(int i=0;i<2;i++) {
    intOut.intByte[i] = EEPROM.read(address+i);
  }
  return intOut.intVal;
}

void writeInt(int value, int address) {
  union intStore { byte intByte[2]; int intVal; } intIn;

  intIn.intVal = value;
  for(int i=0;i<2;i++) {
    EEPROM.write(address+i, intIn.intByte[i]);
  }
}
// Motors
#define PIN_FRONT 4
#define PIN_REAR 5
#define PIN_LEFT 6
#define PIN_RIGHT 7

SoftwareServo motorFront;
SoftwareServo motorRear;
SoftwareServo motorLeft;
SoftwareServo motorRight;

void setupMotors() {
  motorFront.attach(PIN_FRONT);
  motorRear.attach(PIN_REAR);
  motorLeft.attach(PIN_LEFT);
  motorRight.attach(PIN_RIGHT);

  // Arm ESCs
  setAllMotors(MINCOMMAND);
  updateMotors();
  unsigned long tempTime = millis();
  while(millis()<tempTime+1000*4) {
    SoftwareServo::refresh();
  }
}

void wait(int ms) {
  int temp = millis();
  while(millis()-temp<ms) {
    SoftwareServo::refresh();
  }
}

void setAllMotors(int cmd) {
  motor.command[MOTOR_FRONT] = cmd;
  motor.command[MOTOR_REAR] = cmd;
  motor.command[MOTOR_LEFT] = cmd;
  motor.command[MOTOR_RIGHT] = cmd;
}

void updateMotors() {
  motorFront.write(motor.command[MOTOR_FRONT]);
  motorRear.write(motor.command[MOTOR_REAR]);
  motorLeft.write(motor.command[MOTOR_LEFT]);
  motorRight.write(motor.command[MOTOR_RIGHT]);
}
// PID

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
int updatePID(int targetPosition, int currentPosition, struct PID *p) {
  int error = targetPosition - currentPosition;  

  p->integratedError += error;

  // Limit the integrated error by the windupGuard
  if (p->integratedError < -windupGuard) {
    p->integratedError = -windupGuard;
  } else if (p->integratedError > windupGuard) {
    p->integratedError = windupGuard;
  }
  
  int dTerm = (p->D * (currentPosition - p->lastPosition)) / 10;
  p->lastPosition = currentPosition;
  return ((p->P * error) / 10) + ((p->I * p->integratedError) / 10) + dTerm;
}

void zeroIntegralError() {
  for(int n=0;n<3;n++) {
    pid[n].integratedError = 0;
  }
}

//
//  RX channel order
// ==================
//  1: Roll
//  2: Pitch
//  3: Throttle
//  4: Yaw
//  5: Gear
//  6: Aux
//

// RX command states
#define RX_NOT_SYNCHED 0
#define RX_ACQUIRING 1
#define RX_READY 2
#define RX_IN_FAILSAFE 3

void setupRX() {
  ServoDecode.begin();
//  Serial.print("Searching for RC: ");
  do {
    delay(250);
  } while(ServoDecode.getState()!=RX_READY);
//  Serial.println("Found!");
}

void updateRX() {
  for(int n=0;n<6;n++) {
    rx.raw[n] = readRX(n);
    rx.value[n] = smooth(rx.raw[n], rx.value[n], rx.factor[n]);
    // Applying speccy on ROLL, PITCH and YAW to dumb down the sticks a bit
    if(n<3) {
      rx.command[n] = (((rx.value[n] - MIDCOMMAND) * rx.speccyFactor) / 16) + MIDCOMMAND;
    } else {
      rx.command[n] = rx.value[n];
    }
  }

  // Only process RX commands when throttle is off
  if(rx.raw[RX_THROTTLE] < MINCHECK) {
    zeroIntegralError();
    processRXCommands();
  }
  
  // Prevents too little power applied to motors during hard manuevers
  if(rx.value[RX_THROTTLE] > (MIDCOMMAND - MINDELTA)) {
    motor.minimum = rx.value[RX_THROTTLE] - MINDELTA;
  }
  if(rx.value[RX_THROTTLE] < MINTHROTTLE) {
    motor.minimum = MINTHROTTLE;
  }
  // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
  //if((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
  //  minCommand = MINTHROTTLE;
}

void processRXCommands() {
  // Arm motors
  if(rx.raw[RX_YAW] > MAXCHECK && !motor.armed) {
    motor.armed = true;
    motor.minimum = MINTHROTTLE;
    rx.center[RX_ROLL] = rx.value[RX_ROLL];
    rx.center[RX_PITCH] = rx.value[RX_PITCH];
  }
  
  if(rx.raw[RX_YAW] < MINCHECK && motor.armed) {
    motor.armed = false;
    setAllMotors(MINCOMMAND);
    updateMotors();
  }
  
  // Zero sensors (left stick lower left, right stick lower right corner)
  if(rx.raw[RX_YAW] < MINCHECK && rx.raw[RX_ROLL] > MAXCHECK && rx.raw[RX_PITCH] < MINCHECK) {
    zeroGyros();
    zeroAccelerometers();
    zeroIntegralError();
  }
}

int readRX(int channel) {
  if(channel==3) {
    channel = 2;
  } else if(channel==2) {
    channel = 3;
  }
  return ServoDecode.GetChannelPulseWidth(channel+1);
}
// Calibration parameters
#define FINDZERO 50
int findZero[FINDZERO];

// Allows user to zero gyros on command
void zeroGyros() {
  for(int n=0;n<3;n++) {
    for(int i=0;i<FINDZERO;i++) {
      findZero[i] = analogRead(n);
    }
    gyro.zero[n] = findMode(findZero, FINDZERO);
    writeInt(gyro.zero[SENSOR_ROLL], GYRO_ROLL_ZERO_ADR);
    writeInt(gyro.zero[SENSOR_PITCH], GYRO_PITCH_ZERO_ADR);
    writeInt(gyro.zero[SENSOR_YAW], GYRO_YAW_ZERO_ADR);
  }
}

// Allows user to zero accelerometers on command
void zeroAccelerometers() {
  for(int n=0;n<3;n++) {
    for(int i=0;i<FINDZERO; i++) {
      findZero[i] = accelRead(n, false);
    }
    accel.zero[n] = findMode(findZero, FINDZERO);
  }
  writeInt(accel.zero[SENSOR_ROLL], ACCEL_ROLL_ZERO_ADR);
  writeInt(accel.zero[SENSOR_PITCH], ACCEL_PITCH_ZERO_ADR);
  writeInt(accel.zero[SENSOR_ZAXIS], ACCEL_ZAXIS_ZERO_ADR);
}

void updateSensors() {
  for(int n=0;n<3;n++) {
    gyro.raw[n] = analogRead(n) - gyro.zero[n];
    gyro.value[n] = smooth(gyro.raw[n], gyro.value[n], gyro.factor);
    accel.raw[n] = accelRead(n, true) - accel.zero[n];
    accel.value[n] = smooth(accel.raw[n], accel.value[n], accel.factor);
  }
  level.rollRatio = (int) ( ((long) accel.value[SENSOR_ROLL] * 1000) / accel.value[SENSOR_ZAXIS]);
  level.pitchRatio = (int) ( ((long) accel.value[SENSOR_PITCH] * 1000) / accel.value[SENSOR_ZAXIS]);
}

int findMode(int *data, int arraySize) {
  int currentFrequency = 0;
  int maxNumber = 0;
  int maxFrequency = 0;

  for(int n=0;n<arraySize;n++) {
    currentFrequency = 0;
    for(int i=0;i<arraySize;i++) {
      if(data[i] == data[n]) {
        currentFrequency++; 
      }
    }
    if(currentFrequency>maxFrequency) {
      maxFrequency = currentFrequency;
      maxNumber = data[n];
    }
  }
  return maxNumber;
}

void setupAccel() {
   // Join I2C BUS as master
  Wire.begin();
  // Initialize the LIS3LV02DQ
  i2cSend(0x1d, 0x21, 0b01000000); // CTRL_REG2 = +-2g, BDU=1, i2c
  i2cSend(0x1d, 0x20, 0b11010111); // CTRL_REG1 = 2560Hz, Decimate by 128, enable all axis's
}

int accelRead(int axis, boolean compensate) {
  switch(axis) {
    case 0: return i2cReadAccel(0x28);
    case 1: return i2cReadAccel(0x2c);
    case 2: return i2cReadAccel(0x2a)+(compensate?1024:0);
  } 
}

void i2cSend(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.send(reg);
  Wire.send(value);
  Wire.endTransmission(); 
}

int i2cReadAccel(int adr) {
  int r = 0;
  Wire.beginTransmission(0x1d); Wire.send(adr+1); Wire.endTransmission(); // HIGH
  Wire.requestFrom(0x1d, 1); while(Wire.available()) { r = Wire.receive(); } r <<= 8;
  Wire.beginTransmission(0x1d); Wire.send(adr); Wire.endTransmission();
  Wire.requestFrom(0x1d, 1); while(Wire.available()) { r += Wire.receive(); } // LOW
  return r;
}
// Serial interface
//
// Trying to be compatible with the AeroQuad
//
char query = 'X';

void updateSerial() {
  // Check for serial message
  if(Serial.available()) {
    query = Serial.read();
    switch(query) {
//      case 'A': // Receive roll and pitch gyro PID
/*        pid[RX_ROLL].P = readIntSerial();
        pid[RX_ROLL].I = readIntSerial();
        pid[RX_ROLL].D = readIntSerial();
        pid[RX_ROLL].lastPosition = 0;
        pid[RX_ROLL].integratedError = 0;
        pid[RX_PITCH].P = readIntSerial();
        pid[RX_PITCH].I = readIntSerial();
        pid[RX_PITCH].D = readIntSerial();
        pid[RX_PITCH].lastPosition = 0;
        pid[RX_PITCH].integratedError = 0;*/
//        break;
//      case 'C': // Receive yaw PID
/*        pid[RX_YAW].P = readIntSerial();
        pid[RX_YAW].I = readIntSerial();
        pid[RX_YAW].D = readIntSerial();
        pid[RX_YAW].lastPosition = 0;
        pid[RX_YAW].integratedError = 0;*/
//        break;
//      case 'E': // Receive roll and pitch auto level PID
/*        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();*/
//        break;
//      case 'G': // Receive auto level configuration
//        readIntSerial(); // Ignore for now
//        readIntSerial();
//        break;
//      case 'I': // Receive flight control configuration
//        windupGuard = readIntSerial();
//        readIntSerial();
//        rx.speccyFactor = readIntSerial();
//        break;
//      case 'K': // Receive data filtering values
//        gyro.factor = readIntSerial();
//        accel.factor = readIntSerial();
//        readIntSerial(); // Ignore for now
        break;
      case 'Y': // Initialize config with default values
        pid[RX_ROLL].P = 9;
        pid[RX_ROLL].I = 0;
        pid[RX_ROLL].D = -30;
        pid[RX_PITCH].P = 9;
        pid[RX_PITCH].I = 0;
        pid[RX_PITCH].D = -30;
        pid[RX_YAW].P = -30;
        pid[RX_YAW].I = 0;
        pid[RX_YAW].D = 0;
        windupGuard = 1000;
        rx.speccyFactor = 3;
        gyro.factor = 3;
        accel.factor = 3;
        zeroGyros();
        zeroAccelerometers();
        zeroIntegralError();
        break;
      case 'W': // Write config to EEPROM
        writeInt(pid[RX_ROLL].P, PGAIN_ADR);
        writeInt(pid[RX_ROLL].I, IGAIN_ADR);
        writeInt(pid[RX_ROLL].D, DGAIN_ADR);
        writeInt(pid[RX_PITCH].P, PITCH_PGAIN_ADR);
        writeInt(pid[RX_PITCH].I, PITCH_IGAIN_ADR);
        writeInt(pid[RX_PITCH].D, PITCH_DGAIN_ADR);
        writeInt(pid[RX_YAW].P, YAW_PGAIN_ADR);
        writeInt(pid[RX_YAW].I, YAW_IGAIN_ADR);
        writeInt(pid[RX_YAW].D, YAW_DGAIN_ADR);
        writeInt(windupGuard, WINDUPGUARD_ADR);
        writeInt(rx.speccyFactor, XMITFACTOR_ADR);
        writeInt(gyro.factor, GYROSMOOTH_ADR);
        writeInt(accel.factor, ACCSMOOTH_ADR);
        break;
    }
  }

  switch(query) {
    case 'B': // Send roll and pitch gyro PID values
      Serial.print(pid[RX_ROLL].P);
      comma();
      Serial.print(pid[RX_ROLL].I);
      comma();
      Serial.print(pid[RX_ROLL].D);
      comma();
      Serial.print(pid[RX_PITCH].P);
      comma();
      Serial.print(pid[RX_PITCH].I);
      comma();
      Serial.println(pid[RX_PITCH].D);
      query = 'X';
      break;
    case 'D': // Send yaw PID values
      Serial.print(pid[RX_YAW].P);
      comma();
      Serial.print(pid[RX_YAW].I);
      comma();
      Serial.println(pid[RX_YAW].D);
      query = 'X';
      break;
    case 'F': // Send roll and pitch auto level PID values
      repeat(5, 0);
      Serial.println(0);
      query = 'X';
      break;
    case 'H': // Send auto level configuration values
      Serial.print(0);
      comma();
      Serial.println(0);
      query = 'X';
      break;
    case 'J': // Send flight control configuration values
      Serial.print(2000);
      comma();
      Serial.println(3);
      query = 'X';
      break;
    case 'L': // Send data filtering values
      Serial.print(gyro.factor);
      comma();
      Serial.print(accel.factor);
      comma();
      Serial.println(0);
      query = 'X';
      break;


    case 'S': // Send all flight data
      Serial.print(time.delta);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(gyro.value[n]);
        comma();
      }
      Serial.print(rx.value[RX_THROTTLE]);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(motor.axisCommand[n]);
        comma();
      }
      for(int n=0;n<4;n++) {
        Serial.print(motor.command[n]);
        comma();
      }
      Serial.print(motor.armed, BIN);
      comma();
      Serial.println(rx.value[RX_GEAR]);
      break;
    case 'Q': // Send sensor data
      Serial.print(gyro.value[SENSOR_ROLL]);
      comma();
      Serial.print(gyro.value[SENSOR_PITCH]);
      comma();
      Serial.print(gyro.value[SENSOR_YAW]);
      comma();
      Serial.print(accel.value[SENSOR_ROLL]);
      comma();
      Serial.print(accel.value[SENSOR_PITCH]);
      comma();
      Serial.print(accel.value[SENSOR_ZAXIS]);
      comma();
      Serial.print(level.rollAdjust);
      comma();
      Serial.print(level.rollAdjust);
      comma();
      Serial.print(level.rollRatio);
      comma();
      Serial.println(level.pitchRatio);
      break;
    case 'T': // Send processed transmitter values
      Serial.print(rx.speccyFactor);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(rx.command[n]);
        comma();
      }
      repeat(2, 0);
      Serial.print(motor.axisCommand[RX_ROLL]);
      comma();
      Serial.print(motor.axisCommand[RX_PITCH]);
      comma();
      Serial.println(motor.axisCommand[RX_YAW]);
      break;
    case 'U': // Send receiver values
      for (int n=0;n<5;n++) {
        Serial.print(rx.value[n]);
        comma();
      }
      Serial.println(rx.value[5]);
      break;
    case '!': // Send flight software version
      Serial.println("1.3");
      query = 'X';
      break;
    default:
      break;
  }
}

void repeat(int count, int value) {
  for(int n=0;n<count;n++) {
    Serial.print(value);
    comma();
  }
}

void comma() {
  Serial.print(',');
}

// Used to read integer values from the serial port
int readIntSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return (int) atof(data);
}
// Misc Utils

// Smooth - factor in 1/16th (int) = 0-16 maps to 0-100%
int smooth(int current, int previous, int factor) {
  return ((previous * (16 - factor) + (current * factor))) / 16;
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

