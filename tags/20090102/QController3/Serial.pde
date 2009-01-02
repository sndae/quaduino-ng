//
// Serial protocol
//
char hello[]= "\r\nQ  U  A  D  U  I  N  O\r\nAn Arduino based Quadcopter Controller\r\nCopyright (c) 2008 Paul Rene Joergensen\r\n";
char help[] = "\r\nCommands:\r\nL...List param\r\nM...Modify param\r\n";

int in;

void helloWorld() {
  Serial.println(hello);
}

void processSerial() {
  if(Serial.available()) {
    in = Serial.read();
    switch(in) {
      case 'v': Serial.println(hello); break;
      case '?': Serial.println(help); break; 
    }
  }
}
