// =================================================
// =             Q  U  A  D  U  I  N  O            =
// =     An Arduino based Quadcopter Controller    =
// =  Copyright (c) 2008-2009 Paul René Jørgensen  =
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
