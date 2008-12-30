// Motors
#define PIN_FRONT 4
#define PIN_REAR 5
#define PIN_LEFT 6
#define PIN_RIGHT 7
#define MIN_COMMAND 1000
#define LOW_COMMAND 1100
#define MAX_COMMAND 2000

SoftwareServo motorFront, motorRear, motorLeft, motorRight;
int frontCmd, rearCmd, leftCmd, rightCmd;
boolean motorsRunning;


void setupMotors() {
  motorFront.attach(PIN_FRONT); motorFront.setMinimumPulse(MIN_COMMAND); motorFront.setMaximumPulse(MAX_COMMAND);
  motorRear.attach(PIN_REAR); motorRear.setMinimumPulse(MIN_COMMAND); motorRear.setMaximumPulse(MAX_COMMAND);
  motorLeft.attach(PIN_LEFT); motorLeft.setMinimumPulse(MIN_COMMAND); motorLeft.setMaximumPulse(MAX_COMMAND);
  motorRight.attach(PIN_RIGHT); motorRight.setMinimumPulse(MIN_COMMAND); motorRight.setMaximumPulse(MAX_COMMAND);
  commandMotors(LOW_COMMAND, LOW_COMMAND, LOW_COMMAND, LOW_COMMAND);
  tempTime = millis();
  while(millis()<(tempTime+4000)) { // Wait 4000ms to allow the ESCs to arm
    SoftwareServo::refresh();
  }
  motorsRunning = false;
}

void updateMotors() {

  motor[0] = rcValue[2] - pidCmd[0]  - pidCmd[2];
  motor[1] = rcValue[2] + pidCmd[0]  - pidCmd[2];
  motor[2] = rcValue[2] + pidCmd[1]  + pidCmd[2];
  motor[3] = rcValue[2] - pidCmd[1]  + pidCmd[2];
  
  commandMotors(motor[0], motor[1], motor[2], motor[3]);
}

void commandMotors(int front, int rear, int left, int right) {
  // (180 - 0) / (2000 - 1000) = 0.18
  motorFront.write((front-MIN_COMMAND)*0.18);
  motorRear.write((rear-MIN_COMMAND)*0.18);
  motorLeft.write((left-MIN_COMMAND)*0.18);
  motorRight.write((right-MIN_COMMAND)*0.18);
}
