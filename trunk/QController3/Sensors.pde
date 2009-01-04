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

unsigned long lastAccelsUpdate = 0;


void calibrateGyros() {
  for(n=0;n<33*3;n++) {
    if(n<3) {
//      gyroRaw[n] = 0;
      gyroSum[n] = 0;
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
    
    gyroSum[n] += ((gyroRate[n]+2) / 4);
    if(gyroSum[n]>1000) {
      gyroSum[n] = 1000;
    } else if(gyroSum[n]<-1000) {
      gyroSum[n] = -1000;
    }
    if(n<2) { // Auto zeroing for pitch and roll
      if(gyroSum[n]>0) gyroSum[n]--;
      if(gyroSum[n]<0) gyroSum[n]++;
    }
  }
}


void setupAccel() {
   // Join I2C BUS as master
  Wire.begin();
  // Initialize the LIS3LV02DQ
  i2cSend(0x1d, 0x21, 0b01000000); // CTRL_REG2 = +-2g, BDU=1, i2c
  i2cSend(0x1d, 0x20, 0b11010111); // CTRL_REG1 = 2560Hz, Decimate by 128, enable all axis's
  // Read previously calibrated zero point for accelerometers
  accelZero[0] = readInt(ACCEL_ZERO_PITCH_ADDRESS);
  accelZero[2] = readInt(ACCEL_ZERO_ROLL_ADDRESS);
  accelZero[1] = readInt(ACCEL_ZERO_YAW_ADDRESS);
#ifdef debug
  Serial.print("Read accelZero from EEPROM: ");
  Serial.print(accelZero[0]);
  Serial.print(", ");
  Serial.print(accelZero[2]);
  Serial.print(", ");
  Serial.println(accelZero[1]);
#endif
}

void calibrateAccel() {
  accelZero[0] = i2cReadAccel(0x28);
  accelZero[2] = i2cReadAccel(0x2a);
  accelZero[1] = i2cReadAccel(0x2c);
  for(n=0;n<100;n++) {
    accelZero[0] = (accelZero[0]*7 + i2cReadAccel(0x28)) / 8;
    accelZero[2] = (accelZero[2]*7 + i2cReadAccel(0x2a)) / 8;
    accelZero[1] = (accelZero[1]*7 + i2cReadAccel(0x2c)) / 8;
  }
//  accelZero[2] -= 1024; // Subtract gravity on Y-axis
    // Write values to EEPROM
  writeInt(accelZero[0], ACCEL_ZERO_PITCH_ADDRESS);
  writeInt(accelZero[2], ACCEL_ZERO_ROLL_ADDRESS);
  writeInt(accelZero[1], ACCEL_ZERO_YAW_ADDRESS);
#ifdef debug
  Serial.print("accelZero written to EEPROM: ");
  Serial.print(accelZero[0]);
  Serial.print(", ");
  Serial.print(accelZero[2]);
  Serial.print(", ");
  Serial.println(accelZero[1]);
#endif
}

void updateAccels() {
  if(millis() - lastAccelsUpdate > 20) {
    accelValue[0] = (accelValue[0]*7 + (i2cReadAccel(0x28) - accelZero[0])) / 8;
    accelValue[2] = (accelValue[2]*7 + (i2cReadAccel(0x2a) - accelZero[2])) / 8;
    accelValue[1] = (accelValue[1]*7 + (i2cReadAccel(0x2c) - accelZero[1])) / 8;
    
    for(n=0;n<3;n++) {
      accelSum[n] += accelValue[n];
    }
    
    
    // Altitude stabilization
    if(((((accelSum[2] + 8) / 16) * accelUpDownMix) + 128) / 256 > accelAltStab) {
      accelAltStab++;
    } else {
      accelAltStab--;
    }
    accelAltStab = constrain(accelAltStab, -100, 100);
    if(accelSum[2] > 10) accelSum[2] -= 10;
    if(accelSum[2] < -10) accelSum[2] += 10;
    
    
    lastAccelsUpdate = millis();
  }
}

void i2cSend(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.send(reg);
  Wire.send(value);
  Wire.endTransmission(); 
}

int i2cReadAccel(int adr) {
  int r = 0;
  Wire.beginTransmission(0x1d); Wire.send(adr+1); Wire.endTransmission(); // HIGH
  Wire.requestFrom(0x1d, 1); while(Wire.available()) { r = Wire.receive(); } r <<= 8;
  Wire.beginTransmission(0x1d); Wire.send(adr); Wire.endTransmission();
  Wire.requestFrom(0x1d, 1); while(Wire.available()) { r += Wire.receive(); } // LOW
  return r;
}


