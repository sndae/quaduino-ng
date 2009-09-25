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
