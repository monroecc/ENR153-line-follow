/*
 * The purpose of this file is to allow a user to adjust various aspects of the control system without any programming knowlege and without the risk of introducing errors.
 */

// This is the position the servo will assume when the PID adjust is zero, the car should drive straight at this position
#define SERVO_CENTER 90 //BOUNDS: [0, 180]

 //Limits the max deflection angle that can be set by the PID
#define MAX_ADJ_LEFT 51 //BOUNDS: [0, 90]
#define MAX_ADJ_RIGHT -32 //BOUNDS: [0, 90]

// PID control contants, be very careful adjusting these
// For a simplified explanation of pid read the pid_basiscs.pdf file in the pid_info folder
// More advanced: https://en.wikipedia.org/wiki/PID_controller
// Tuning instructions: how_to_tune.pdf in pid_info folder
// NOTE: We have found that since this system is controlled by a servo which has a specific response delay that adding a derivative term does not improve the pid at all.
// Adding an integral term however does seem to improve the loop.
#define P 0.23
#define I 0.03
#define D 0.0

 
//offset for EEPROM, can be used if the first 12 bytes are written too many times. This will result in the board being incapable of holding onto a calibration after a reset or power-down.
#define EEPROM_OFF 0 * 12

 //Integral buffer length: The integral term of the PID is only the summation of the last I_BUFFER_LEN terms. The number was choosen arbitrarly. Just remember, a bigger number results in more memory used and increased loop time.
 #define I_BUFFER_LEN 30