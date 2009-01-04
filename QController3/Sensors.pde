/*
** Sensor wiring diagram
**
** ADXRS300 gyros are hooked up on Analog pins 0-2. 
**  - Analog 0 : Pitch Gyro
**  - Analog 1 : Roll Gyro
**  - Analog 2 : Yaw Gyro
**
** LIS3LV02DQ accelerometer is hooked up through
** the I2C interface using the official Wire library
**  - Analog 4 : SDA (data line)
**  - Analog 5 : SCL (clock line)
**
*/

// EEPROM Adresses
#define ACCEL_ZERO_PITCH_ADDRESS 0
#define ACCEL_ZERO_ROLL_ADDRESS 2
#define ACCEL_ZERO_YAW_ADDRESS 4

// Gyro ADC pins
#define PIN_GYRO_PITCH 0
#define PIN_GYRO_ROLL 1
#define PIN_GYRO_YAW 2


void calibrateGyros() {
  for(n=0;n<33*3;n++) {
    if(n<3) {
//      gyroRaw[n] = 0;
//      gyroSum[n] = 0;
      gyroZero[n] = 0;
      gyroRateOld[n] = 0;
    } else {
      gyroZero[n%3] += analogRead(n%3);
      if((n/3)>31) {
        gyroZero[n%3] += 16;
        gyroZero[n%3] /= 32;
      }
    }
  }
#ifdef debug
  Serial.print("Gyro zero calibrated: ");
  Serial.print(gyroZero[0]);
  Serial.print(", ");
  Serial.print(gyroZero[1]);
  Serial.print(", ");
  Serial.println(gyroZero[2]);
#endif
}

void updateGyros() {
  for(n=0;n<3;n++) {
    gyroRateOld[n] = gyroRate[n];
    gyroRate[n] = (((long) gyroRate[n])*7 + ((long) analogRead(n) - gyroZero[n])) / 8;
  }
}
