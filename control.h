#ifndef CONTROL
    #define CONTROL

    #define NUM_A_SENSORS 6
    #define CALIBRATE_PIN 12
    #define ANALOG_TRANSISTOR_PIN 5
    
    #include <Arduino.h>
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
            pidd[2] += (pidd[0] + pidd[1])/2;
            pidd[3] = pidd[0]*w[0] + pidd[2]*w[1] + (pidd[0] - pidd[1]) * w[2];

            return pidd[3];
        }
    };

    class analog_sensor
    {
    private:
        int pin, min, max;
        float scale, value;
    public:
        void init(int arg_pin)
        {
            pin = arg_pin;
            scale = 1.0;
            min = 1025;
            max = 0;
            value = 0;
        }

        float read_sensor()
        {
            value = analogRead(pin);
            return this->get_value();
        }

        float get_value()
        {
            return value;
        }

        void calibrate_point()
        {
            this->read_sensor();

            if(this->get_value() < min){
                min = this->get_value();
            }
            else if(this->get_value() > max){
                max = this->get_value();
            }
            scale = 100/(max-min);
        }
    };

    class line
    {
    private:
        analog_sensor * sensors;
        PID pidlf;
        int density, nsensors;
        int * adjust;
        long w, wsum, pos;
        char linechar;
    public:
        void init(analog_sensor * a_sensors, int len, int * adj)
        {
            pidlf.set_pid(.5, 0, 0);
            adjust = adj;
            sensors = a_sensors;
            nsensors = len;
            this->read_line();
        }

        void calibrate()
        {
            for (int i = 0; i < nsensors; ++i)
            {
                sensors[i].calibrate_point();
            }
        }

        long read_line()
        {
            char bin = 0;
            linechar = 0xFF;
            for (int i = 0; i < nsensors; ++i)
            {
                w += sensors[i].read_sensor();
                wsum += sensors[i].get_value() * i;

                bin = sensors[i].get_value() > 50;
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

        void follow()
        {
            this->read_line();
            if(density != 0)
                *adjust = pidlf.slice(350-pos);
        }
    };

    typedef struct
    {
        int state;

    }handler;

    void resolve_state(handler &h)
    {
        if(digitalRead(CALIBRATE_PIN) == LOW)
        {
            h.state = 1;
        }
        else
        {
            h.state = 0;
        }
    }

    void display_line(line &l)
    {
        char disp = l.get_linechar();

        DDRC |= 0x3F;
        PORTC |= (disp & 0x3F);
        PORTC &= (disp | 0xC0);

        digitalWrite(ANALOG_TRANSISTOR_PIN, HIGH);
        delay(10);
        digitalWrite(ANALOG_TRANSISTOR_PIN, LOW);
        DDRC &= 0xC0;
    }


#endif