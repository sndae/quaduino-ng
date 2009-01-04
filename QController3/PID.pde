

void updatePID() {
  for(n=0;n<3;n++) {
    pidCmd[n] = gyroRate[n] * 18; // P
    pidCmd[n]+= (gyroRate[n] - gyroRateOld[n]) * -15; // D
    pidCmd[n]/=10;
    
    pidCmd[n]+= (gyroSum[n] * 4) / 100; // I

    switch(n) {
      case 0: pidCmd[n] -= rcValue[1]; break; // PITCH
      case 1: pidCmd[n] += rcValue[0]; break; // ROLL
      case 2: pidCmd[n] -= rcValue[3]; break; // YAW
    }
  }
}

/*
void updatePID() {
//  pidCmd[0] = updatePID((mMotorRate * gyroRate[0]) + bMotorRate, (rcValue[1]+1500), &lastPitchPosition, &iPitchState);
//  pidCmd[1] = updatePID((mMotorRate * gyroRate[1]) + bMotorRate, (-rcValue[0]+1500), &lastRollPosition, &iRollState);
//  pidCmd[2] = updatePID((mMotorRate * gyroRate[2]) + bMotorRate, (rcValue[3]+1500), &lastYawPosition, &iYawState);   
  pidCmd[0] = updatePID(gyroRate[0], rcValue[1], &lastPitchPosition, &iPitchState);
  pidCmd[1] = updatePID(gyroRate[1], -rcValue[0], &lastRollPosition, &iRollState);
  pidCmd[2] = updatePID(gyroRate[2], rcValue[3], &lastYawPosition, &iYawState);   
}

float updatePID(float targetPosition, float currentPosition, float *lastPosition, float *iState) {
  // these local variables can be factored out if memory is an issue, 
  // but they make it more readable
  float error;
  float windupGaurd;

  // determine how badly we are doing
  error = targetPosition - currentPosition;

  // the pTerm is the view from now, the pgain judges 
  // how much we care about error we are this instant.
  pTerm = pGain * error;

  // iState keeps changing over time; it's 
  // overall "performance" over time, or accumulated error
  *iState += error;

  // to prevent the iTerm getting huge despite lots of 
  //  error, we use a "windup guard" 
  // not necessary, but this makes windup guard values 
  // relative to the current iGain
  windupGaurd = WINDUP_GUARD_GAIN / iGain;  

  if (*iState > windupGaurd) 
    *iState = windupGaurd;
  else if (*iState < -windupGaurd) 
    *iState = -windupGaurd;
  iTerm = iGain * (*iState);

  // the dTerm, the difference between the current
  //  and last reading, indicated the "speed," 
  // how quickly the reading is changing. (aka. Differential)
  dTerm = (dGain * (currentPosition - *lastPosition));
  *lastPosition = currentPosition;
  return  pTerm + iTerm - dTerm;
}
*/
