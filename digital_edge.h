#ifndef DIGITAL_EDGE
	#define DIGITAL_EDGE

	#include <Arduino.h>
	
	class edge
	{
	private:
		void wait_pin_settle(int pin, u_long mils)
		{
			value = digitalRead(pin);
	
			count = millis();
			int hold;
			while((millis() - count) < mils)
			{
				hold = digitalRead(pin);
				if(value != hold){
					value = hold;
					count = millis();
				}
			}
		}
	public:
		int rising, falling, toggle, state, value, pin;
		u_long count;

		edge(int arg_pin)
		{
			pin = arg_pin;
			this->eval();
			value = -1;
			state = -1;
		}

		void eval()
		{
			wait_pin_settle(pin, 10);
			
			rising = (state == 0 && value == 1);
			falling = (state == 1 && value == 0);
			toggle = (rising || falling);
			state = value;
		}
	};

	class do_once
	{
	private:
		int fc_flag;
	public:
		do_once()
		{
			this->reset();
		}

		int check()
		{
			int tmp = fc_flag;
			fc_flag = 0;
			return tmp;
		}

		void reset()
		{
			fc_flag = 1;
		}
	};
#endif
