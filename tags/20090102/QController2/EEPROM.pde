// Utilities for writing and reading from the EEPROM

int readInt(int address) {
  union intStore { byte intByte[2]; int intVal; } intOut;
  for(i=0;i<2;i++) {
    intOut.intByte[i] = EEPROM.read(address+i);
  }
  return intOut.intVal;
}

void writeInt(int value, int address) {
  union intStore { byte intByte[2]; int intVal; } intIn;
  intIn.intVal = value;
  for(i=0;i<2;i++) {
    EEPROM.write(address+i, intIn.intByte[i]);
  }
}
