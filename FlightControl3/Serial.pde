// Serial interface
//
// Trying to be compatible with the AeroQuad
//
char query = 'X';

void updateSerial() {
  // Check for serial message
  if(Serial.available()) {
    query = Serial.read();
    switch(query) {
//      case 'A': // Receive roll and pitch gyro PID
/*        pid[RX_ROLL].P = readIntSerial();
        pid[RX_ROLL].I = readIntSerial();
        pid[RX_ROLL].D = readIntSerial();
        pid[RX_ROLL].lastPosition = 0;
        pid[RX_ROLL].integratedError = 0;
        pid[RX_PITCH].P = readIntSerial();
        pid[RX_PITCH].I = readIntSerial();
        pid[RX_PITCH].D = readIntSerial();
        pid[RX_PITCH].lastPosition = 0;
        pid[RX_PITCH].integratedError = 0;*/
//        break;
//      case 'C': // Receive yaw PID
/*        pid[RX_YAW].P = readIntSerial();
        pid[RX_YAW].I = readIntSerial();
        pid[RX_YAW].D = readIntSerial();
        pid[RX_YAW].lastPosition = 0;
        pid[RX_YAW].integratedError = 0;*/
//        break;
//      case 'E': // Receive roll and pitch auto level PID
/*        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();*/
//        break;
//      case 'G': // Receive auto level configuration
//        readIntSerial(); // Ignore for now
//        readIntSerial();
//        break;
//      case 'I': // Receive flight control configuration
//        windupGuard = readIntSerial();
//        readIntSerial();
//        rx.speccyFactor = readIntSerial();
//        break;
//      case 'K': // Receive data filtering values
//        gyro.factor = readIntSerial();
//        accel.factor = readIntSerial();
//        readIntSerial(); // Ignore for now
        break;
      case 'Y': // Initialize config with default values
        pid[RX_ROLL].P = 9;
        pid[RX_ROLL].I = 0;
        pid[RX_ROLL].D = -30;
        pid[RX_PITCH].P = 9;
        pid[RX_PITCH].I = 0;
        pid[RX_PITCH].D = -30;
        pid[RX_YAW].P = -30;
        pid[RX_YAW].I = 0;
        pid[RX_YAW].D = 0;
        windupGuard = 1000;
        rx.speccyFactor = 3;
        gyro.factor = 3;
        accel.factor = 3;
        zeroGyros();
        zeroAccelerometers();
        zeroIntegralError();
        break;
      case 'W': // Write config to EEPROM
        writeInt(pid[RX_ROLL].P, PGAIN_ADR);
        writeInt(pid[RX_ROLL].I, IGAIN_ADR);
        writeInt(pid[RX_ROLL].D, DGAIN_ADR);
        writeInt(pid[RX_PITCH].P, PITCH_PGAIN_ADR);
        writeInt(pid[RX_PITCH].I, PITCH_IGAIN_ADR);
        writeInt(pid[RX_PITCH].D, PITCH_DGAIN_ADR);
        writeInt(pid[RX_YAW].P, YAW_PGAIN_ADR);
        writeInt(pid[RX_YAW].I, YAW_IGAIN_ADR);
        writeInt(pid[RX_YAW].D, YAW_DGAIN_ADR);
        writeInt(windupGuard, WINDUPGUARD_ADR);
        writeInt(rx.speccyFactor, XMITFACTOR_ADR);
        writeInt(gyro.factor, GYROSMOOTH_ADR);
        writeInt(accel.factor, ACCSMOOTH_ADR);
        break;
    }
  }

  switch(query) {
    case 'B': // Send roll and pitch gyro PID values
      Serial.print(pid[RX_ROLL].P);
      comma();
      Serial.print(pid[RX_ROLL].I);
      comma();
      Serial.print(pid[RX_ROLL].D);
      comma();
      Serial.print(pid[RX_PITCH].P);
      comma();
      Serial.print(pid[RX_PITCH].I);
      comma();
      Serial.println(pid[RX_PITCH].D);
      query = 'X';
      break;
    case 'D': // Send yaw PID values
      Serial.print(pid[RX_YAW].P);
      comma();
      Serial.print(pid[RX_YAW].I);
      comma();
      Serial.println(pid[RX_YAW].D);
      query = 'X';
      break;
    case 'F': // Send roll and pitch auto level PID values
      repeat(5, 0);
      Serial.println(0);
      query = 'X';
      break;
    case 'H': // Send auto level configuration values
      Serial.print(0);
      comma();
      Serial.println(0);
      query = 'X';
      break;
    case 'J': // Send flight control configuration values
      Serial.print(2000);
      comma();
      Serial.println(3);
      query = 'X';
      break;
    case 'L': // Send data filtering values
      Serial.print(gyro.factor);
      comma();
      Serial.print(accel.factor);
      comma();
      Serial.println(0);
      query = 'X';
      break;


    case 'S': // Send all flight data
      Serial.print(time.delta);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(gyro.value[n]);
        comma();
      }
      Serial.print(rx.value[RX_THROTTLE]);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(motor.axisCommand[n]);
        comma();
      }
      for(int n=0;n<4;n++) {
        Serial.print(motor.command[n]);
        comma();
      }
      Serial.print(motor.armed, BIN);
      comma();
      Serial.println(rx.value[RX_GEAR]);
      break;
    case 'Q': // Send sensor data
      Serial.print(gyro.value[SENSOR_ROLL]);
      comma();
      Serial.print(gyro.value[SENSOR_PITCH]);
      comma();
      Serial.print(gyro.value[SENSOR_YAW]);
      comma();
      Serial.print(accel.value[SENSOR_ROLL]);
      comma();
      Serial.print(accel.value[SENSOR_PITCH]);
      comma();
      Serial.print(accel.value[SENSOR_ZAXIS]);
      comma();
      Serial.print(level.rollAdjust);
      comma();
      Serial.print(level.rollAdjust);
      comma();
      Serial.print(level.rollRatio);
      comma();
      Serial.println(level.pitchRatio);
      break;
    case 'T': // Send processed transmitter values
      Serial.print(rx.speccyFactor);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(rx.command[n]);
        comma();
      }
      repeat(2, 0);
      Serial.print(motor.axisCommand[RX_ROLL]);
      comma();
      Serial.print(motor.axisCommand[RX_PITCH]);
      comma();
      Serial.println(motor.axisCommand[RX_YAW]);
      break;
    case 'U': // Send receiver values
      for (int n=0;n<5;n++) {
        Serial.print(rx.value[n]);
        comma();
      }
      Serial.println(rx.value[5]);
      break;
    case '!': // Send flight software version
      Serial.println("1.3");
      query = 'X';
      break;
    default:
      break;
  }
}

void repeat(int count, int value) {
  for(int n=0;n<count;n++) {
    Serial.print(value);
    comma();
  }
}

void comma() {
  Serial.print(',');
}

// Used to read integer values from the serial port
int readIntSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return (int) atof(data);
}
