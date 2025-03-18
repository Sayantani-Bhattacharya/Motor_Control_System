#include "nu32dip.h"           // config bits, constants, funcs for startup and UART
#include "isense.h"
#include "utilities.h"
#include "positionControl.h"


// Function Declarations
void current_ISR_Setup(void);
void makeWaveform(void);
float getKp_current(void);
float getKi_current(void);
void set_gains(float kp, float ki);
void set_direction(int dir);
void perform_i_test(void);
void plot_current_data(void);
float set_desired_angle_in_cc(float angle);
