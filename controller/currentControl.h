#include "nu32dip.h"           // config bits, constants, funcs for startup and UART
#include "isense.h"

// Function Declarations

void position_ISR_Setup(void);
void makeWaveform(void);
float getKp_current(void);
float getKi_current(void);
void set_gains(float kp, float ki);
void set_direction(int dir);
void makeWaveform();
void perform_i_test(void);

// void ADC_Startup(void);