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

void checkRadio() {
  while(ServoDecode.getState()!=RC_READY) {
    // TODO: Beeeeep!
  }
}

void calibrateRadio() {
  for(n=0;n<6;n++) { // Init the array
    RADIO_ZERO[n] = ServoDecode.GetChannelPulseWidth(n+1);
  }
  unsigned long time = millis();
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
    
    // Max stick +-500 = +-512 quids = +-180 degrees
    // 180/10 = 18 degrees : 500/10
    // 
    WANTED_ANGLE[INDEX_ROLL] = RADIO_VALUE[0] / 10;
    WANTED_ANGLE[INDEX_PITCH] = RADIO_VALUE[1] / 10;
    WANTED_ANGLE[INDEX_YAW] += RADIO_VALUE[3] / 100;
    // Correct for yaw wrap around
    if(WANTED_ANGLE[INDEX_YAW]>512) WANTED_ANGLE[INDEX_YAW]-=1024;
    if(WANTED_ANGLE[INDEX_YAW]<-512) WANTED_ANGLE[INDEX_YAW]+=1024;
    
    lastRcUpdate = millis();
  }
}
