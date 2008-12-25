
int lastAccelUpdate = 0;
float degPerSec = 0;

void updateOrientation(int deltaTime) {
  updateGyros();
  
  for(n=0;n<3;n++) {
    // GYRO_RAW is ADC values for change in degree / second. Divide by 1000 to get degree / ms and 
    // multiply with elapsed time to get rotation since last update
    // 5000mV / 1024 (10-bit ADC resolution) = 4.8828125 mV for each ADC value
    // 5mv / degree / second
//    degPerSec = ((GYRO_RAW[n] * 4.8828125) / 5) * 2.84444; // Convert from degrees into quids
//    GYRO_ANGLE[n] += int((degPerSec / 1000.0) * deltaTime);

      GYRO_ANGLE[n] += ((GYRO_RAW[n] * 2.7777) / 1000.0) * deltaTime;
  }
  
  // Update bias every 500ms
  if(millis()-lastAccelUpdate>500 ) {
    updateAccel();
    for(n=0;n<2;n++) {
      GYRO_BIAS[n] = (GYRO_BIAS[n] + (GYRO_ANGLE[n] - ACCEL_ANGLE[n])) / 2;
    }
    lastAccelUpdate = millis();
  }
  
  for(n=0;n<3;n++) {
    // YAW BIAS is 0 and therefore the final result is correct
    ORIENTATION[n] = GYRO_ANGLE[n] - GYRO_BIAS[n];
    // Make sure the angle is within -512 and 512 quids
    if(ORIENTATION[n]>512) {
      ORIENTATION[n] -= 1024;
    } else if(ORIENTATION[n]<-512) {
      ORIENTATION[n] += 1024;
    }
  }
}
