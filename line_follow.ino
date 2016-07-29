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
  state = 0;

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


//int ostate = 0;
void loop() {

  //if((millis() % 100) < 5)
    s_cal = digitalRead(CALIBRATE_PIN);

//  if(state != ostate)
//    Serial.println(state);
//  ostate = state;

  switch (state)
  {    
    case 0:
      l.follow();
      adjust = constrain(adjust, MAX_ADJ_RIGHT, MAX_ADJ_LEFT);
      steer.write(SERVO_CENTER - adjust);
      state = s_cal;
      break;
    case 1:
      l.clear_calibration();
      state = 2;
    case 2:
      l.calibrate();
      display_line(l);
      if (s_cal == LOW)
        state = 3;
      break;
    case 3:
      l.save();
      state = 0;
      break;
  }
}

