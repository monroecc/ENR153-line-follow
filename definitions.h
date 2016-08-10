/*
 * The purpose of this file is to allow a user to adjust various aspects of the control system without any programming knowlege and without the risk of introducing errors.
 */

// This is the position the servo will assume when the PID adjust is zero
#define SERVO_CENTER 90 //BOUNDS: [0, 180]

 //Limits the max deflection angle that can be set by the PID
#define MAX_ADJ_LEFT 51 //BOUNDS: [0, 90]
#define MAX_ADJ_RIGHT -32 //BOUNDS: [0, 90]

//PID control contants, be very careful adjusting these
#define P 0.23
#define I 0.03
#define D 0.0
 
//offset for EEPROM, can be used if the first 12 bytes are written too many times
#define EEPROM_OFF 0 * 12

 //Integral buffer length
 #define I_BUFFER_LEN 30