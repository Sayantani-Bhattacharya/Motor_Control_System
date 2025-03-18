#include "nu32dip.h"           // config bits, constants, funcs for startup and UART
#include "isense.h"
// #include "utilities.h"


// Function Declarations
// void position_ISR_Setup(void);
// void makeWaveform(void);
float getKp_position(void);
float getKi_position(void);
float getKd_position(void);
void set_position_gains(float kp, float ki, float kd);
// void perform_i_test(void);








