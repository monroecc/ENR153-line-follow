#include <Servo.h>
#include "control.h"

#define SERVO_PIN 5

handler h;
line l;
int adjust;
Servo steer;

void setup() {
  Serial.begin(9600);
  h.state = 0;

  analog_sensor sensors[NUM_A_SENSORS];

  for(int i = 0; i < NUM_A_SENSORS; i++)
  {
    sensors[i].init(i);
  }

  l.init(sensors, NUM_A_SENSORS, &adjust);

  steer.attach(SERVO_PIN);

  pinMode(ANALOG_TRANSISTOR_PIN, OUTPUT);
  digitalWrite(ANALOG_TRANSISTOR_PIN, LOW);
}

void loop() {
  resolve_state(h);
  switch(h.state)
  {
     case 0:
        l.follow();
        steer.write(90 - adjust);
        break;
     case 1:
        l.calibrate();
        display_line(l);
        break;
  }
}
