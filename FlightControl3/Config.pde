// EEPROM storage addresses
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define ACCEL_PITCH_ZERO_ADR 60
#define ACCEL_ROLL_ZERO_ADR 64
#define ACCEL_ZAXIS_ZERO_ADR 68
#define GYRO_ROLL_ZERO_ADR 96
#define GYRO_PITCH_ZERO_ADR 100
#define GYRO_YAW_ZERO_ADR 104
#define PITCH_PGAIN_ADR 124
#define PITCH_IGAIN_ADR 128
#define PITCH_DGAIN_ADR 132

void configRead() {
  accel.zero[SENSOR_ROLL] = readInt(ACCEL_ROLL_ZERO_ADR);
  accel.zero[SENSOR_PITCH] = readInt(ACCEL_PITCH_ZERO_ADR);
  accel.zero[SENSOR_ZAXIS] = readInt(ACCEL_ZAXIS_ZERO_ADR);
  gyro.zero[SENSOR_ROLL] = readInt(GYRO_ROLL_ZERO_ADR);
  gyro.zero[SENSOR_PITCH] = readInt(GYRO_PITCH_ZERO_ADR);
  gyro.zero[SENSOR_YAW] = readInt(GYRO_YAW_ZERO_ADR);
 
  pid[RX_ROLL].P = readInt(PGAIN_ADR);
  pid[RX_ROLL].I = readInt(IGAIN_ADR);
  pid[RX_ROLL].D = readInt(DGAIN_ADR);
  pid[RX_PITCH].P = readInt(PITCH_PGAIN_ADR);
  pid[RX_PITCH].I = readInt(PITCH_IGAIN_ADR);
  pid[RX_PITCH].D = readInt(PITCH_DGAIN_ADR);
  pid[RX_YAW].P = readInt(YAW_PGAIN_ADR);
  pid[RX_YAW].I = readInt(YAW_IGAIN_ADR);
  pid[RX_YAW].D = readInt(YAW_DGAIN_ADR);
  
  windupGuard = readInt(WINDUPGUARD_ADR);
  rx.speccyFactor = readInt(XMITFACTOR_ADR);
  gyro.factor = readInt(GYROSMOOTH_ADR);
  accel.factor = readInt(ACCSMOOTH_ADR);
}

// Utilities for writing and reading from the EEPROM
int readInt(int address) {
  union intStore { byte intByte[2]; int intVal; } intOut;
  for(int i=0;i<2;i++) {
    intOut.intByte[i] = EEPROM.read(address+i);
  }
  return intOut.intVal;
}

void writeInt(int value, int address) {
  union intStore { byte intByte[2]; int intVal; } intIn;

  intIn.intVal = value;
  for(int i=0;i<2;i++) {
    EEPROM.write(address+i, intIn.intByte[i]);
  }
}
