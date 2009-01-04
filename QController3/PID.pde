

void updatePID() {
  for(n=0;n<3;n++) {
    pidCmd[n] = gyroRate[n] * 18; // P
    pidCmd[n]+= (gyroRate[n] - gyroRateOld[n]) * -15; // D
    pidCmd[n]/=10;
    
    if(rcValue[4]+rcZero[4]>1500) {
      pidCmd[n]+= (gyroSum[n] * 4) / 100; // I
    }

    switch(n) {
      case 0: pidCmd[n] -= rcValue[1]; break; // PITCH
      case 1: pidCmd[n] += rcValue[0]; break; // ROLL
      case 2: pidCmd[n] -= rcValue[3]; break; // YAW
    }
  }
}
