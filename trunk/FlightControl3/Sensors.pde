// Calibration parameters
#define FINDZERO 50
int findZero[FINDZERO];

// Allows user to zero gyros on command
void zeroGyros() {
  for(int n=0;n<3;n++) {
    for(int i=0;i<FINDZERO;i++) {
      findZero[i] = analogRead(n);
    }
    gyro.zero[n] = findMode(findZero, FINDZERO);
    writeInt(gyro.zero[SENSOR_ROLL], GYRO_ROLL_ZERO_ADR);
    writeInt(gyro.zero[SENSOR_PITCH], GYRO_PITCH_ZERO_ADR);
    writeInt(gyro.zero[SENSOR_YAW], GYRO_YAW_ZERO_ADR);
  }
}

// Allows user to zero accelerometers on command
void zeroAccelerometers() {
  for(int n=0;n<3;n++) {
    for(int i=0;i<FINDZERO; i++) {
      findZero[i] = accelRead(n, false);
    }
    accel.zero[n] = findMode(findZero, FINDZERO);
  }
  writeInt(accel.zero[SENSOR_ROLL], ACCEL_ROLL_ZERO_ADR);
  writeInt(accel.zero[SENSOR_PITCH], ACCEL_PITCH_ZERO_ADR);
  writeInt(accel.zero[SENSOR_ZAXIS], ACCEL_ZAXIS_ZERO_ADR);
}

void updateSensors() {
  for(int n=0;n<3;n++) {
    gyro.raw[n] = analogRead(n) - gyro.zero[n];
    gyro.value[n] = smooth(gyro.raw[n], gyro.value[n], gyro.factor);
    accel.raw[n] = accelRead(n, true) - accel.zero[n];
    accel.value[n] = smooth(accel.raw[n], accel.value[n], accel.factor);
  }
  level.rollRatio = (int) ( ((long) accel.value[SENSOR_ROLL] * 1000) / accel.value[SENSOR_ZAXIS]);
  level.pitchRatio = (int) ( ((long) accel.value[SENSOR_PITCH] * 1000) / accel.value[SENSOR_ZAXIS]);
}

int findMode(int *data, int arraySize) {
  int currentFrequency = 0;
  int maxNumber = 0;
  int maxFrequency = 0;

  for(int n=0;n<arraySize;n++) {
    currentFrequency = 0;
    for(int i=0;i<arraySize;i++) {
      if(data[i] == data[n]) {
        currentFrequency++; 
      }
    }
    if(currentFrequency>maxFrequency) {
      maxFrequency = currentFrequency;
      maxNumber = data[n];
    }
  }
  return maxNumber;
}

void setupAccel() {
   // Join I2C BUS as master
  Wire.begin();
  // Initialize the LIS3LV02DQ
  i2cSend(0x1d, 0x21, 0b01000000); // CTRL_REG2 = +-2g, BDU=1, i2c
  i2cSend(0x1d, 0x20, 0b11010111); // CTRL_REG1 = 2560Hz, Decimate by 128, enable all axis's
}

int accelRead(int axis, boolean compensate) {
  switch(axis) {
    case 0: return i2cReadAccel(0x28);
    case 1: return i2cReadAccel(0x2c);
    case 2: return i2cReadAccel(0x2a)+(compensate?1024:0);
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
