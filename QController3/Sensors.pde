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
  for(n=0;n<3*128;n++) {
    if(n<4) {
      gyroRaw[n] = 0;
      gyroSum[n] = 0;
      gyroZero[n] = analogRead(n);
    } else {
      gyroZero[n%3] = (gyroZero[n%3]*7 + analogRead(n%3)) / 8;
    }
  }
  Serial.print("Gyro zero calibrated: ");
  Serial.print(gyroZero[0]);
  Serial.print(", ");
  Serial.print(gyroZero[1]);
  Serial.print(", ");
  Serial.println(gyroZero[2]);
}


void updateGyros() {
  for(n=0;n<3;n++) {
    gyroRaw[n] = (analogRead(n) + analogRead(n)) >> 1;
    gyroRaw[n] -= gyroZero[n];
    if(abs(gyroRaw[n])<4) gyroRaw[n] = 0;
    gyroSum[n] += (gyroRaw[n]/2);
    if(gyroSum[n]>gyroIntegralLimit[n]) {
      gyroSum[n] = gyroIntegralLimit[n];
    } else if(gyroSum[n]<-gyroIntegralLimit[n]) {
      gyroSum[n] = -gyroIntegralLimit[n];
    }
    gyroRaw[n] /=4;
  }
}
