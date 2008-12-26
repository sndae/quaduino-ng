/*
** RC channel order
**
** 1: Roll
** 2: Pitch
** 3: Throttle
** 4: Yaw
** 5: Gear
** 6: Aux
**
*/

// RADIO command states
#define RC_NOT_SYNCHED 0
#define RC_ACQUIRING 1
#define RC_READY 2
#define RC_IN_FAILSAFE 3

void checkRadio() {
  while(ServoDecode.getState()!=RC_READY) {
    // TODO: Beeeeep!
  }
}

void calibrateRadio() {
  for(n=0;n<6;n++) { // Init the array
    RADIO_ZERO[n] = ServoDecode.GetChannelPulseWidth(n+1);
  }
  time = millis();
  while(millis()-time<500) { // Do as many samples as we can for 500ms
    for(n=0;n<6;n++) {
      RADIO_ZERO[n] = (RADIO_ZERO[n] + ServoDecode.GetChannelPulseWidth(n+1)) / 2;
    }
  }
}

long lastRcUpdate = 0;

void updateRadio() {
  // Update RC data every 20 ms
  if(millis()-lastRcUpdate>20) {
    for(n=0;n<6;n++) {
      RADIO_VALUE[n] = (((RADIO_VALUE[n]+RADIO_ZERO[n]) + ServoDecode.GetChannelPulseWidth(n+1)) / 2) - RADIO_ZERO[n];
    }
    lastRcUpdate = millis();
  }
}
