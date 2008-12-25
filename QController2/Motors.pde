// Motors
#define PIN_NORTH 4
#define PIN_SOUTH 5
#define PIN_WEST 6
#define PIN_EAST 7
#define OFFCOMMAND 1000
#define MAXCOMMAND 2000
#define LOWCOMMAND 1100
SoftwareServo motorNorth;
SoftwareServo motorSouth;
SoftwareServo motorWest;
SoftwareServo motorEast;
boolean motorsRunning;

void initMotors() {
  motorNorth.attach(PIN_NORTH); motorNorth.setMinimumPulse(1000); motorNorth.setMaximumPulse(2000);
  motorSouth.attach(PIN_SOUTH); motorSouth.setMinimumPulse(1000); motorSouth.setMaximumPulse(2000);
  motorWest.attach(PIN_WEST); motorWest.setMinimumPulse(1000); motorWest.setMaximumPulse(2000);
  motorEast.attach(PIN_EAST); motorEast.setMinimumPulse(1000); motorEast.setMaximumPulse(2000);
  commandAllMotors(OFFCOMMAND);
  long time = millis();
  while(millis()<(time+4000)) { // Wait 4000ms to allow the ESCs to arm
    SoftwareServo::refresh();
  }
  motorsRunning = false;
}

void decodeMotorCommands() {
  if(RADIO_VALUE[2] < 210 && ServoDecode.getState()==2) {
    // Disarm motors(throttle down, yaw left)
    if(RADIO_VALUE[3] < -200 && motorsRunning) {
      commandAllMotors(OFFCOMMAND);
      motorsRunning = false;
    }
    
    // Arm motors (throttle down, yaw right)  
    if(RADIO_VALUE[3] > 200 && !motorsRunning) {
      commandAllMotors(LOWCOMMAND);
      motorsRunning = true;
    }

    // Calibrate gyros (throttle down, yaw left, pitch up)
    if(RADIO_VALUE[3] < -200 && RADIO_VALUE[1] > 200) {
      calibrateGyros();
    }

    // Zero gyro angles (throttle down, yaw left, pitch down)
    if(RADIO_VALUE[3] < -200 && RADIO_VALUE[1] < -200) {
      zeroGyros();
    }

    // Calibrate accelerometers (throttle down, yaw left, roll left)
    if(RADIO_VALUE[3] < -200 && RADIO_VALUE[0] < -200) {
      calibrateAccel();
    }
    
  }
}

void commandAllMotors(int value) {
  // (180 - 0) / (2000 - 1000) = 0.18
  motorNorth.write(value*0.18);
  motorSouth.write(value*0.18);
  motorWest.write(value*0.18);
  motorEast.write(value*0.18);
}
