

void updateOrientation(int deltaTime) {
  updateGyros();
  
  for(n=0;n<3;n++) {
    GYRO_ANGLE[n] += ((GYRO_RAW[n] / 1000.0) * deltaTime);
  }
  
  // Update bias every 10 ms
  if(millis()-lastAccelUpdate>20) {
    updateAccel();
    for(n=0;n<2;n++) {
      GYRO_BIAS[n] = (GYRO_BIAS[n]*0.8 + (GYRO_ANGLE[n] - ACCEL_ANGLE[n])*0.2);
    }
    lastAccelUpdate = millis();
  }
  
  for(n=0;n<3;n++) {
    // YAW BIAS is 0 and therefore the final result is correct
    ORIENTATION[n] = GYRO_ANGLE[n] - GYRO_BIAS[n];
    // Make sure the angle is within -180 and 180 degrees
    if(ORIENTATION[n]>180) {
      ORIENTATION[n] -= 360;
    } else if(ORIENTATION[n]<-180) {
      ORIENTATION[n] += 360;
    }
  }
}
