// Motors
#define PIN_FRONT 4
#define PIN_REAR 5
#define PIN_LEFT 6
#define PIN_RIGHT 7
#define MIN_COMMAND 1000
#define MAX_COMMAND 2000

SoftwareServo motorFront, motorRear, motorLeft, motorRight;
int frontCmd, rearCmd, leftCmd, rightCmd;

void setupMotors() {
  motorFront.attach(PIN_FRONT); motorFront.setMinimumPulse(MIN_COMMAND); motorFront.setMaximumPulse(MAX_COMMAND);
  motorRear.attach(PIN_REAR); motorRear.setMinimumPulse(MIN_COMMAND); motorRear.setMaximumPulse(MAX_COMMAND);
  motorLeft.attach(PIN_LEFT); motorLeft.setMinimumPulse(MIN_COMMAND); motorLeft.setMaximumPulse(MAX_COMMAND);
  motorRight.attach(PIN_RIGHT); motorRight.setMinimumPulse(MIN_COMMAND); motorRight.setMaximumPulse(MAX_COMMAND);
  commandMotors(MIN_COMMAND, MIN_COMMAND, MIN_COMMAND, MIN_COMMAND);
  tempTime = millis();
  while(millis()<(tempTime+8000)) { // Wait 4000ms to allow the ESCs to arm
    SoftwareServo::refresh();
#ifdef debug
    if(millis()%1000==0) Serial.println(millis()-tempTime);
#endif
  }
}

void updateMotors() {
  
  motor[0] = rcValue[2] - pidCmd[0]  - pidCmd[2];
  motor[1] = rcValue[2] + pidCmd[0]  - pidCmd[2];
  motor[2] = rcValue[2] + pidCmd[1]  + pidCmd[2];
  motor[3] = rcValue[2] - pidCmd[1]  + pidCmd[2];
  
  for(n=0;n<4;n++) {
    motor[n] = constrain(motor[n], 30, 255);
  }
  
  commandMotors(motor[0], motor[1], motor[2], motor[3]);
}

void commandMotors(int front, int rear, int left, int right) {
  // (180 - 0) / (255 - 0) = 0.7058, 141 / 200
  motorFront.write((front * 141) / 200);
  motorRear.write(  (rear * 141) / 200);
  motorLeft.write(  (left * 141) / 200);
  motorRight.write((right * 141) / 200);
}
