#include <SoftwareServo.h>

SoftwareServo servo1;

void setup()
{
  servo1.attach(2);
  servo1.setMaximumPulse(2200);
}

void loop()
{
  static int value = 0;
  millisSet(0);
  while(1)
  {
    value=millis()/100;
    if (value<=180)
    {
      servo1.write(value);
    }
    else if (value<360)
    {
      servo1.write(360-value);
    }
    else millisSet(0);
    SoftwareServo::refresh();
  }
}
