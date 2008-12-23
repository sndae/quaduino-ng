
int lastAccelUpdate = 0;

void updateOrientation(int deltaTime) {
  updateGyros();
  
  for(n=0;n<3;n++) {
    // GYRO_RAW is ADC values for change in degree / second. Divide by 1000 to get degree / ms and 
    // multiply with elapsed time to get rotation since last update
    // 5000mV / 1024 (10-bit ADC resolution) = 4.8828125 mV for each ADC value
    // 5mv / degree / second
    float degPerSec = ((GYRO_RAW[n] * 4.8828125) / 5);
    GYRO_ANGLE[n] += ((degPerSec / 1000.0) * deltaTime);
//    GYRO_ANGLE[n] += ((GYRO_RAW[n] / 1000.0) * deltaTime);
  }
  
  // Update bias every 20 ms
  if(millis()-lastAccelUpdate>20) {
    updateAccel();
    for(n=0;n<2;n++) {
      GYRO_BIAS[n] = (GYRO_BIAS[n]*6 + (GYRO_ANGLE[n] - ACCEL_ANGLE[n])*2) / 8;
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
