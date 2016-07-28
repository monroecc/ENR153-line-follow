#include <Servo.h>
#include "control.h"

line l;
long adjust = 5;
Servo steer;

int s_cal, state;

analog_sensor sensors[NUM_A_SENSORS];
int pin_map[NUM_A_SENSORS] = {A0, A1, A2, A3, A4, A5};

void setup() {
  Serial.begin(9600);
  state = 1;

  for (int i = 0; i < NUM_A_SENSORS; i++)
  {
    sensors[i].init(pin_map[i]);
  }

  l.init(sensors, NUM_A_SENSORS, &adjust);

  l.load();

  steer.attach(SERVO_PIN);

  pinMode(ANALOG_TRANSISTOR_PIN, OUTPUT);
  digitalWrite(ANALOG_TRANSISTOR_PIN, LOW);
}



void loop() {

  //if((millis() % 100) < 5)
    s_cal = digitalRead(CALIBRATE_PIN);

  switch (state)
  {
    case 1:
      l.follow();
      adjust = constrain(adjust, -41, 58);
      steer.write(90 - adjust);
      state = s_cal;
      break;
    case 0:
      l.clear_calibration();
      state = 2;
    case 2:
      l.calibrate();
      display_line(l);
      if (s_cal == HIGH)
        state = 3;
      break;
    case 3:
      l.save();
      state = 1;
      break;
  }
}

