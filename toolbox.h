#ifndef TOOLBOX
  #define TOOLBOX

#include <Arduino.h>

  void blink_led()
  {
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }

void print_char_bitwise(char ch)
{
  for (int i = 8; i >= 0; --i)
  {
    Serial.print((int)(ch >> i) & 0x01);
  }
}


#endif

