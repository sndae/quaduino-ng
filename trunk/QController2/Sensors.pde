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

void initSensors() {
   // Join I2C BUS as master
  Wire.begin();
  // Initialize the LIS3LV02DQ
  i2cSend(0x1d, 0x21, 0b01000000); // CTRL_REG2 = +-2g, BDU=1, i2c
  i2cSend(0x1d, 0x20, 0b11010111); // CTRL_REG1 = 2560Hz, Decimate by 128, enable all axis's
  // Read previously calibrated zero point for accelerometers
  ACCEL_ZERO[0] = readInt(ACCEL_ZERO_PITCH_ADDRESS);
  ACCEL_ZERO[1] = readInt(ACCEL_ZERO_ROLL_ADDRESS);
  ACCEL_ZERO[2] = readInt(ACCEL_ZERO_YAW_ADDRESS);
  Serial.print("Accel Zero from EEPROM: ");
  Serial.print(ACCEL_ZERO[0]);
  Serial.print(", ");
  Serial.print(ACCEL_ZERO[1]);
  Serial.print(", ");
  Serial.println(ACCEL_ZERO[2]);
  Serial.print("Zeroing gyros: ");
  calibrateGyros();
  zeroGyros();
  Serial.println("Completed");
}

long outX, outY, outZ;

void updateAccel() {
  outX = i2cReadAccel(0x28) - ACCEL_ZERO[0]; 
  outZ = i2cReadAccel(0x2a) - ACCEL_ZERO[1];
  outY = i2cReadAccel(0x2c) - ACCEL_ZERO[2];

  // Pitch angle (using non-optimized version just now)
  // atan2(gx, sqrt(gy*gy + gz*gz)) 
  // ACCEL_ANGLE[1] = (atan2(ACCEL_RAW[0], ACCEL_RAW[1]) * 180) / PI;
  ACCEL_ANGLE[INDEX_PITCH] = int(round(atan2(outX, sqrt(outY*outY+outZ*outZ))*512.0/PI));

  // Roll angle
  // atan2(gy, sqrt(gx*gx + gz*gz))
  // ACCEL_ANGLE[0] = (atan2(ACCEL_RAW[2], ACCEL_RAW[1]) * 180) / PI;
  ACCEL_ANGLE[INDEX_ROLL] = int(round(atan2(outY, sqrt(outX*outX+outZ*outZ))*512.0/PI));
}

void updateGyros() {
  GYRO_RAW[INDEX_PITCH] = -(analogRead(PIN_GYRO_PITCH) - GYRO_ZERO[INDEX_PITCH]);
  GYRO_RAW[INDEX_ROLL] = analogRead(PIN_GYRO_ROLL) - GYRO_ZERO[INDEX_ROLL];
  GYRO_RAW[INDEX_YAW] = analogRead(PIN_GYRO_YAW) - GYRO_ZERO[INDEX_YAW];
  for(n=0;n<3;n++) if(abs(GYRO_RAW[n])<4) GYRO_RAW[n] = 0;
}

void zeroGyros() {
  for(n=0;n<3;n++) {
    GYRO_ANGLE[n] = 0;
    GYRO_BIAS[n] = 0;
  }
  Serial.println("Gyro angles zeroed");
}

void calibrateGyros() {
  GYRO_ZERO[INDEX_PITCH] = analogRead(PIN_GYRO_PITCH);
  GYRO_ZERO[INDEX_ROLL] = analogRead(PIN_GYRO_ROLL);
  GYRO_ZERO[INDEX_YAW] = analogRead(PIN_GYRO_YAW);
  for(n=0;n<100;n++) {
    GYRO_ZERO[INDEX_PITCH] = (GYRO_ZERO[INDEX_PITCH] + analogRead(PIN_GYRO_PITCH)) / 2;
    GYRO_ZERO[INDEX_ROLL] = (GYRO_ZERO[INDEX_ROLL] + analogRead(PIN_GYRO_ROLL)) / 2;
    GYRO_ZERO[INDEX_YAW] = (GYRO_ZERO[INDEX_YAW] + analogRead(PIN_GYRO_YAW)) / 2;
  }
  Serial.print("Gyro zero calibrated: ");
  Serial.print(GYRO_ZERO[0]);
  Serial.print(", ");
  Serial.print(GYRO_ZERO[1]);
  Serial.print(", ");
  Serial.println(GYRO_ZERO[2]);
}

void calibrateAccel() {
  ACCEL_ZERO[0] = i2cReadAccel(0x28);
  ACCEL_ZERO[1] = i2cReadAccel(0x2a);
  ACCEL_ZERO[2] = i2cReadAccel(0x2c);
  for(n=0;n<100;n++) {
    ACCEL_ZERO[0] = (ACCEL_ZERO[0]*3 + i2cReadAccel(0x28)) / 4;
    ACCEL_ZERO[1] = (ACCEL_ZERO[1]*3 + i2cReadAccel(0x2a)) / 4;
    ACCEL_ZERO[2] = (ACCEL_ZERO[2]*3 + i2cReadAccel(0x2c)) / 4;
  }
  ACCEL_ZERO[1] -= 1024; // Subtract gravity on Y-axis
    // Write values to EEPROM
  writeInt(ACCEL_ZERO[0], ACCEL_ZERO_PITCH_ADDRESS);
  writeInt(ACCEL_ZERO[1], ACCEL_ZERO_ROLL_ADDRESS);
  writeInt(ACCEL_ZERO[2], ACCEL_ZERO_YAW_ADDRESS);
  Serial.print("Accel Zero written to EEPROM: ");
  Serial.print(ACCEL_ZERO[0]);
  Serial.print(", ");
  Serial.print(ACCEL_ZERO[1]);
  Serial.print(", ");
  Serial.println(ACCEL_ZERO[2]);
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
