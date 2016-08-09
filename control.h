#ifndef CONTROL
    #define CONTROL

    #include "definitions.h"

    //#define DEBUG
    //#define DEBUG_EACH

    #ifdef DEBUG
        #include "toolbox.h"
    #endif

    #define NUM_A_SENSORS 6
    #define CALIBRATE_PIN 2
    #define ANALOG_TRANSISTOR_PIN 10
    #define SERVO_PIN 3
    #define EEPROM_OFF 0 * 12  //offset for EEPROM, can be used if the first 12 bytes are written too many times
    
    #include <Arduino.h>
    #include <EEPROMex.h>
    //GENERAL PID
    class PID
    {
    public:
        float pidd[4]; //err, last error, running integral, adjust
        float w[3];

        void set_pid(float p, float i, float d)
        {
            w[0] = p;w[1] = i;w[2] = d;
            pidd[0] = 0; pidd[1] = 0; pidd[2] = 0;
        }

        float slice(float err)
        {
            pidd[1] = pidd[0];
            pidd[0] = err;
            

            if( abs(pidd[0]) < abs(pidd[1]) ){ //ignore i term when coming back towards the zero error
                pidd[2] = 0;
            }else{
                pidd[2] += (pidd[0] + pidd[1])/2;
            }

            pidd[3] = pidd[0]*w[0] + pidd[2]*w[1] + (pidd[0] - pidd[1]) * w[2];

            return pidd[3];
        }
    };

    class analog_sensor
    {
    public:
        int pin, min, max, val;
        float scale;
        static int eeprom_head;
        void init(int arg_pin)
        {
            this->pin = arg_pin;
            this->scale = 1.0;
            this->min = 2000;
            this->max = 0;
            this->val = 0;
        }

        int read_sensor()
        {
            int ret = (int)((analogRead(pin) - min)*scale); //calculate normalized sensor value
            ret = constrain(ret, 0, 100);
            return ret;
        }

        void calibrate_point()
        {
            int cal = analogRead(pin);
            if(cal < min){
                min = cal;
            }
            else if(cal > max){
                max = cal;
            }
            scale = 100.0/(max - min);
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
        analog_sensor * sensors;
        PID pidlf;
        int nsensors, last_line;
        long * adjust;
        long w, wsum, pos; //weights, weighted sum, position on line
        char linechar; //character representation of line
        char flag; //a flag
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

            char eval = 0x00;
            long sens = 0;


            for (int i = 0; i < nsensors; ++i)
            {
                sens = (long)sensors[i].read_sensor();

                w += sens;
                wsum += sens * i * 100;

                /* linechar:                    0    0    0    0    0    0    0    0 
                *  (bin << (nsensors - 1 - 1)): 0    0   bin   0    0    0    0    0 
                *  (bin << (nsensors - 1 - 2)): 0    0    0   bin   0    0    0    0 
                *  (bin << (nsensors - 1 - 3)): 0    0    0    0   bin   0    0    0 
                * ...
                * final output:                 0    0   b0   b1   b2    b3   b4   b5
                */

                linechar |= ( (sens > 50) << (nsensors - 1 - i));
            }

            eval = linechar & 0x21;

            if(eval == 0x01){
                last_line = 1;
            }
            else if(eval == 0x20){
                last_line = 0;
            }


            if(linechar == 0x00){
                pos = last_line*500;
            }
            else{
                pos = wsum/w;
            }

            return pos;
        }

        char get_linechar()
        {
            this->read_line();
            return linechar;
        }

        void follow()
        {
            this->read_line();
            *adjust = (long)(pidlf.slice(250.0-(float)pos)); //set the adjustment pointer
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

