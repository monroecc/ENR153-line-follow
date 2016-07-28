#ifndef TOOLBOX
  #define TOOLBOX

#include <Arduino.h>

  int sign_f(float val)
  {
    if(val < 0){return -1;}
    else if(val > 0){return 1;}
    else{return 0;}
  }

  int sign_i(int val)
  {
    if(val < 0){return -1;}
    else if(val > 0){return 1;}
    else{return 0;}
  }

  void blink_led()
  {
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }

void print_arrf(float * arr, int n)
{
  for (int i = 0; i < n; ++i)
  {
    Serial.print(arr[i]);
    if(i != (n-1))
      Serial.print(",");
  }
}

void print_arri(int * arr, int n)
{
  for (int i = 0; i < n; ++i)
  {
    Serial.print(arr[i]);
    if(i != (n-1))
      Serial.print(",");
  }
}

void print_char_bitwise(char ch)
{
  for (int i = 8; i >= 0; --i)
  {
    Serial.print((int)(ch >> i) & 0x01);
  }
}


#endif

