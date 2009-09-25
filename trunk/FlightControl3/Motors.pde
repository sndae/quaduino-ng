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
