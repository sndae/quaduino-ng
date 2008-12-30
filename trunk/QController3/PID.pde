
void clearPID() {
    for(n=0;n<3;n++) {
      pidCmd[n] = 0;
    }
}

void updatePID() {
  for(n=0;n<3;n++) {
    // PD
    pidCmd[n] = -gyroValue[n];
    pidCmd[n] += gyroValueOld[n];
    pidCmd[n] *= dGain[n];
    pidCmd[n] += (gyroValue[n]*pGain[n]);
    pidCmd[n] += 8;
    pidCmd[n] /= 16;

    pidCmd[n] += accelCorrection[n]; // Acceleration sensor correction

    // I
    pidCmd[n] += (((gyroSum[n]*iGain[n])+128) / 256);
    
    // RC Signal
    switch(n) {
      case 0: pidCmd[n] -= rcValue[1] ; break; // PITCH
      case 1: pidCmd[n] -= rcValue[0]; break; // ROLL
      case 2: pidCmd[n] -= rcValue[3]; break; // YAW
    }
        
    gyroValueOld[n] = gyroValue[n];
  }
}
