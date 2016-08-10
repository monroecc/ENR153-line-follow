#ifndef TOOLBOX
  #define TOOLBOX

#include <Arduino.h>

void print_char_bitwise(char ch)
{
  for (int i = 8; i >= 0; --i)
  {
    Serial.print((int)(ch >> i) & 0x01);
  }
}


#endif

