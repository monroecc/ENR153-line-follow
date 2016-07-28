#ifndef CONTROL
    #define CONTROL

    //#define DEBUG
    //#define DEBUG_EACH

    #ifdef DEBUG
        #include "toolbox.h"
    #endif

    #define NUM_A_SENSORS 6
    #define CALIBRATE_PIN 2
    #define ANALOG_TRANSISTOR_PIN 10
    #define SERVO_PIN 3
    
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
            if(abs(pidd[0]) < abs(pidd[1]))
                pidd[2] = 0;

            pidd[1] = pidd[0];
            pidd[0] = err;
            pidd[2] += (pidd[0] + pidd[1])/2;
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
            int ret = (int)((analogRead(pin) - min)*scale);
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
            analog_sensor::eeprom_head = 0;
        }

        void reset_calibration()
        {
            this->scale = 1.0;
            this->min = 1025;
            this->max = 0;
        }

        void save()
        {
            EEPROM.writeInt(eeprom_head, min);
            eeprom_head += sizeof(int);
            EEPROM.writeInt(eeprom_head, max);
            eeprom_head += sizeof(int);
        }

        void load()
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
        int density, nsensors;
        long * adjust;
        long w, wsum, pos;
        char linechar;
        char flag;
    public:
        void init(analog_sensor * a_sensors, int len, long * adj)
        {
            pidlf.set_pid(.5, 0, 0);
            adjust = adj;
            sensors = a_sensors;
            nsensors = len;
            this->read_line();
            flag = 0;
        }

        void clear_calibration()
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
                sensors[i].calibrate_point();
            }

            #ifdef DEBUG_EACH
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
            char bin = 0;
            density = 0;
            w = 0; 
            wsum = 0;
            linechar = 0x00;
            long sens = 0;

            for (int i = 0; i < nsensors; ++i)
            {
                sens = (long)sensors[i].read_sensor();

                w += sens;
                wsum += sens * i * 100;

                bin = sens > 50;
                density += bin;
                linechar |= (bin << (nsensors - 1 - i));

            }
            pos = wsum/w;

            return pos;
        }

        char get_linechar()
        {
            this->read_line();
            return linechar;
        }

        int get_density()
        {
            return density;
        }

        void follow()
        {
            this->read_line();
            if(density != 0)
                *adjust = (long)(pidlf.slice(250.0-(float)pos));
        }

        void load()
        {
            analog_sensor::reset_head();
            for (int i = 0; i < nsensors; ++i)
            {
                sensors[i].load();
            }
        }

        void save()
        {
            analog_sensor::reset_head();
            for (int i = 0; i < nsensors; ++i)
            {
                sensors[i].save();
            }
        }
    };

    void display_line(line &l)
    {
        char disp = l.get_linechar();

        #ifdef DEBUG
            print_char_bitwise(disp);
            Serial.println();
        #endif

        DDRC |= 0x3F;
        PORTC |= (disp & 0x3F);
        PORTC &= (disp | 0xC0);

        digitalWrite(ANALOG_TRANSISTOR_PIN, HIGH);
        delay(10);
        digitalWrite(ANALOG_TRANSISTOR_PIN, LOW);
        DDRC &= 0xC0;
    }


#endif

