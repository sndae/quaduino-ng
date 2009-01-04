//
//  RC channel order
// ==================
//  1: Roll
//  2: Pitch
//  3: Throttle
//  4: Yaw
//  5: Gear
//  6: Aux
//
unsigned long lastRcUpdate = 0;

void setupRadio() {
  ServoDecode.begin();
  Serial.print("Searching for RC: ");
  do {
    wait(250);
  } while(ServoDecode.getState()!=RC_READY);
  Serial.println("Found!");
  calibrateRadio();
}

void calibrateRadio() {
  // Make sure the throttle is not open!
  startCalibration:
  if(ServoDecode.GetChannelPulseWidth(3)>1100) {
    Serial.print("Error: Throttle is open! (");
    Serial.print(ServoDecode.GetChannelPulseWidth(3));
    Serial.println(">1100)");
    wait(1000);
    goto startCalibration;
  } else {
    wait(250);
  }
  // Calculate each channel's zero point
  for(n=0;n<6*100;n++) { // Moving average of 100 samples, first sample stored without averaging
    if(n<6) {
      rcZero[n] = ServoDecode.GetChannelPulseWidth(n+1);
    } else {
      rcZero[n%6] = (rcZero[n%6] + ServoDecode.GetChannelPulseWidth((n%6)+1)) / 2;
    }
    processSerial();
  }
}

void updateRadio() {
  // Update RC data every 20ms if signal is OK
  if(millis() - lastRcUpdate > 20 && ServoDecode.getState()==RC_READY) {
    for(n=0;n<6;n++) {
      if(speccy && (n==0 || n==1 || n==3)) { // Reduce RC sensitivity
        rcValue[n] = (rcValue[n]*3 + ((ServoDecode.GetChannelPulseWidth(n+1) - rcZero[n]) / 2)) / 4;
      } else {
        rcValue[n] = (rcValue[n]*3 + ((ServoDecode.GetChannelPulseWidth(n+1) - rcZero[n]) )) / 4;
      }
    }
    lastRcUpdate = millis();
  }
}
