#ifndef CONTROL
    #define CONTROL

    #include "definitions.h"

    //#define DEBUG       // Uncommenting this line prints the binary representation of the line sensor to the console during calibration
    //#define DEBUG_EACH  // Uncommenting this line prints the min and max line sensor values to the console during calibration

    #ifdef DEBUG
        #include "toolbox.h" //debug tools
    #endif

    #define NUM_A_SENSORS 6             // Number of analog sensors, used as array length
    #define CALIBRATE_PIN 2             // Connected to the calibration pin
    #define ANALOG_TRANSISTOR_PIN 10    // Pin connected to the transistor base, used to cut the bar led out of the line follow circuit
    #define SERVO_PIN 3                 // Servo is connected to this pin, it is a PWM pin
    
    #include <Arduino.h>
    #include <EEPROMex.h> //Make sure you install this via the Arduino IDE, allows writing of things other than bytes

    //GENERAL PID
    class PID
    {
    public:
        float pidd[4]; //err, last error, running integral, adjust
        float w[3];    //holds pid constants

        float circ_i[I_BUFFER_LEN]; // only worry about summing the last 30 terms of remond sum
        int head;         // head for circuilar array

        void set_pid(float p, float i, float d)
        {
            w[0] = p;w[1] = i;w[2] = d;
            pidd[0] = 0; pidd[1] = 0; pidd[2] = 0;
            head = 0;

            memset(circ_i, 0x00, sizeof(float)*I_BUFFER_LEN);
        }

        float slice(float err)
        {
            pidd[1] = pidd[0];                      // set last err pidd[1]
            pidd[0] = err;                          // set current err pidd[0]

            if( abs(pidd[0]) > abs(pidd[1]) ){
                pidd[2] -= circ_i[head];                // Subract off the element of circ_i you are about to replace
                circ_i[head] = (pidd[0] + pidd[1])/2;   // Set the element of the cirular array to this "slice" of the integral
                pidd[2] += circ_i[head];                // Add to integral element pidd[2]
                head++;                                // increments head
                if(head == I_BUFFER_LEN)
                    head = 0;  
            }else{
                pidd[2] = 0;
                if(circ_i[head] != 0)
                    memset(circ_i, 0x00, sizeof(float)*I_BUFFER_LEN);
            }                             


            pidd[3] = pidd[0]*w[0] + pidd[2]*w[1] + (pidd[0] - pidd[1]) * w[2]; // Compute PID output and save it in pidd[3]

            return pidd[3];                                                     // return adjustment
        }
    };

    class analog_sensor
    {
    public:
        int pin, min, max; //analog pin [0, 5], absolute min for this sensor, absolute max for this sensor
        float scale;      // Scale value, used to fit any reading to a value between 0-100 (normalize sensor)
        static int eeprom_head; // The current eeprom position, keeps sensors from overwriting each others calibration
        void init(int arg_pin)
        {
            this->pin = arg_pin;
            this->scale = 1.0;
            this->min = 2000;
            this->max = 0;
        }

        int read_sensor()
        {
            // Nomalize sensor, contrain, and return
            int ret = (int)((analogRead(pin) - min)*scale); //calculate normalized sensor value
            ret = constrain(ret, 0, 100);
            return ret;
        }

        void calibrate_point()
        {
            int cal = analogRead(pin);
            if(cal < min){
                min = cal; // Set new absolute min
            }
            else if(cal > max){
                max = cal;  // Set new absolute max
            }
            scale = 100.0/(max - min);  // Compute scale
        }

        static void reset_head()
        {
            analog_sensor::eeprom_head = EEPROM_OFF;
        }

        void reset_calibration()
        {
            this->scale = 1.0;
            this->min = 1025;
            this->max = 0;
        }

        void save() //save cailbration (min, max) for this sensor
        {
            EEPROM.writeInt(eeprom_head, min);
            eeprom_head += sizeof(int);
            EEPROM.writeInt(eeprom_head, max);
            eeprom_head += sizeof(int);
        }

        void load() //load calibration for this sensor
        {
            min = EEPROM.readInt(eeprom_head);
            eeprom_head += sizeof(int);
            max = EEPROM.readInt(eeprom_head);
            eeprom_head += sizeof(int);

            scale = 100.0/(max - min);
        }
    };

    int analog_sensor::eeprom_head = 0;

    class line
    {
    private:
        analog_sensor * sensors;    // Array of sensors to compute line position from
        PID pidlf;                  
        int nsensors, last_line;    // number of sensors, last_line is the last place the sensor saw the line, either 0 (left) or 1 (right)
        long * adjust;              // pointer to variable to be adjusted by pid
        long w, wsum, pos;          //weights, weighted sum, position on line
        char linechar;              //character representation of line
        char flag;                  //a flag
    public:
        void init(analog_sensor * a_sensors, int len, long * adj)
        {
            pidlf.set_pid(P, I, D); //set pid values, TODO: this should be somewhere else
            adjust = adj;
            sensors = a_sensors;
            nsensors = len;
            this->read_line();
            flag = 0;
            last_line = 1;
        }

        void clear_calibration() //everytime the car enters calibrate mode the previous calibration is erased
        {
            for (int i = 0; i < nsensors; ++i)
            {
                sensors[i].reset_calibration();
            }
        }

        void calibrate()
        {
            for (int i = 0; i < nsensors; ++i)
            {
                //reads the sensor once and incoperates that into the calibration
                sensors[i].calibrate_point();
            }

            #ifdef DEBUG_EACH //prints out usefull stuff if DEBUG_EACH is defined
                Serial.print("min: ");
                for (int i = 0; i < 6; ++i)
                {
                    Serial.print(sensors[i].min);
                    Serial.print(", ");
                }
                Serial.print("|||");
                Serial.print(" max: ");
                for (int i = 0; i < 6; ++i)
                {
                    Serial.print(sensors[i].max);
                    Serial.print(", ");
                }
                Serial.println();
            #endif
        }

        long read_line()
        {            
            w = 0; wsum = 0;
            linechar = 0x00;

            long sens = 0;

            char bin = 0; // binary represntion of sensor, > 50 is 1 <= 50 = 0;


            for (int i = 0; i < nsensors; ++i)
            {
                sens = (long)sensors[i].read_sensor();

                w += sens;
                wsum += sens * i * 100;

                bin = sens > 50;

                /* linechar:                    0    0    0    0    0    0    0    0 
                *  (bin << (nsensors - 1 - 1)): 0    0   bin   0    0    0    0    0 
                *  (bin << (nsensors - 1 - 2)): 0    0    0   bin   0    0    0    0 
                *  (bin << (nsensors - 1 - 3)): 0    0    0    0   bin   0    0    0 
                * ...
                * final output:                 0    0   b0   b1   b2    b3   b4   b5
                */

                if(bin)
                {
                    last_line = i;
                }

                linechar |= ( (bin) << (nsensors - 1 - i)); // Character represnation of line, written drectly to PORTC when displaying line on bar led
            }


            if(linechar == 0x00){ // Are we off the line?
                pos = (last_line >= 4)*500; // if we went off to the right set the position to 500 if we went off to the left set position to zero
                // this will result in maximum error and maxiumum adjustment back towards the line
            }
            else{
                pos = wsum/w; // we are still on the line, compute position normally. This weighted average method is used in pololu's library as well
            }

            return pos;
        }

        char get_linechar()
        {
            //return character represntation of the line
            this->read_line();
            return linechar;
        }

        void follow()
        {
            //
            this->read_line();
            *adjust = (long)(pidlf.slice(250.0-(float)pos)); //set the adjustment pointer to the output of the PID, 250 is the center position.
        }

        void load() //load in calibration from eeprom for all sensors
        {
            analog_sensor::reset_head();
            for (int i = 0; i < nsensors; ++i)
            {
                sensors[i].load();
            }
        }

        void save() //save the calibration to eeprom for all sensors
        {
            analog_sensor::reset_head();
            for (int i = 0; i < nsensors; ++i)
            {
                sensors[i].save();
            }
        }
    };

    void display_line(line &l) //display the line on bar led, this is chip specific
    {
        char disp = l.get_linechar();

        #ifdef DEBUG
            print_char_bitwise(disp);
            Serial.println();
        #endif

        DDRC |= 0x3F; //DDR for analog input pins, set to output
        PORTC |= (disp & 0x3F); // set 1's
        PORTC &= (disp | 0xC0); // set 0's

        PORTB |= 0x04; // set transistor pin (10) high
        delay(10);     // duty cycle adjust
        PORTB &= 0xFB; // set transistor pin (10) low
        DDRC &= 0xC0;  // set analog pin to input        
        delay(10);     // wait for line follow
    }

#endif

