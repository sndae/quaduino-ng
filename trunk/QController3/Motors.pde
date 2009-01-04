// Motors
#define PIN_NORTH 4
#define PIN_SOUTH 5
#define PIN_WEST 6
#define PIN_EAST 7

#define MINCOMMAND 1000
#define MAXCOMMAND 2000

SoftwareServo motorFront;
SoftwareServo motorRear;
SoftwareServo motorLeft;
SoftwareServo motorRight;

void setupMotors() {
  motorFront.attach(PIN_NORTH); motorFront.setMinimumPulse(1000); motorFront.setMaximumPulse(2000);
  motorRear.attach(PIN_SOUTH); motorRear.setMinimumPulse(1000); motorRear.setMaximumPulse(2000);
  motorLeft.attach(PIN_WEST); motorLeft.setMinimumPulse(1000); motorLeft.setMaximumPulse(2000);
  motorRight.attach(PIN_EAST); motorRight.setMinimumPulse(1000); motorRight.setMaximumPulse(2000);
  commandMotors(MINCOMMAND, MINCOMMAND, MINCOMMAND, MINCOMMAND);
  flying = false;
  tempTime = millis();
  while(millis()<tempTime+1000*4) {
    SoftwareServo::refresh();
  }
}

void updateMotors() {
  interpreteRadioCommands();
  
  // Calculate motor commands
  motor[0] = constrain((rcValue[2]+rcZero[2]) + pidCmd[0] + pidCmd[2], MINCOMMAND, MAXCOMMAND);
  motor[1] = constrain((rcValue[2]+rcZero[2]) - pidCmd[0] + pidCmd[2], MINCOMMAND, MAXCOMMAND);
  motor[2] = constrain((rcValue[2]+rcZero[2]) + pidCmd[1] - pidCmd[2], MINCOMMAND, MAXCOMMAND);
  motor[3] = constrain((rcValue[2]+rcZero[2]) - pidCmd[1] - pidCmd[2], MINCOMMAND, MAXCOMMAND);
  if(flying) {
    // Command motors
    commandMotors(motor[0], motor[1], motor[2], motor[3]);
  } else {
    commandMotors(MINCOMMAND, MINCOMMAND, MINCOMMAND, MINCOMMAND);
  }  
}

void interpreteRadioCommands() {
  // Send configuration commands from transmitter
  if((rcValue[2]+rcZero[2]) < 1100 && ServoDecode.getState()==RC_READY) {
    // Disarm motors (throttle down, yaw left)
    if ((rcValue[3]+rcZero[3]) < 1400 && flying) {
      flying = false;
    }
    
    // Zero gyros (throttle down, yaw left, pitch up)
    if((rcValue[3]+rcZero[3]) < 1400 && ((rcValue[1]+rcZero[1]) > 1600)) {
      calibrateGyros();
    }
    
    // Arm motors (throttle down, yaw right)  
    if((rcValue[3]+rcZero[3]) > 1600 && !flying) {
      flying = true;
    }
  }
  
  /*    if(gear>1500) {
      if(abs(roll - 1500)<50 && abs(pitch - 1500)<50) {
        updateAccel();
        roll+=(ACCEL_ANGLE[INDEX_ROLL]*3);
        pitch-=(ACCEL_ANGLE[INDEX_PITCH]*3);
      }
    }*/
}

void commandMotors(int front, int rear, int left, int right) {
  // (180 - 0) / (2000 - 1000) = 0.18
  motorFront.write((front-MINCOMMAND)*0.18);
  motorRear.write((rear-MINCOMMAND)*0.18);
  motorLeft.write((left-MINCOMMAND)*0.18);
  motorRight.write((right-MINCOMMAND)*0.18);
}
